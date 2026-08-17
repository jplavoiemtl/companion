#include "video_stream.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ESP32_JPEG_Library.h>
#include <esp_heap_caps.h>

#include "secrets_private.h"
#include "ui.h"

#define USBSerial Serial

namespace {

// --- Burst configuration -----------------------------------------------------
constexpr uint32_t FRAMES_PER_VARIANT = 15;                 // 15 each way
constexpr uint32_t TOTAL_FRAMES = FRAMES_PER_VARIANT * 2;
constexpr uint16_t REQ_HEIGHT = 360;                        // 640x360 from Frigate
constexpr uint8_t  REQ_QUALITY = 25;
constexpr size_t   MAX_FRAME_BYTES = 64000;                 // frames measure ~21 KB
constexpr uint32_t HTTP_TIMEOUT_MS = 15000;

// 640x360 is the smallest size that both satisfies ESP32_JPEG's
// divisible-by-8 rotation constraint and still nearly fills the 368 px screen
// width after a 90 degree rotation.
constexpr uint16_t SRC_W = 640;
constexpr uint16_t SRC_H = 360;

// --- State -------------------------------------------------------------------
bool active = false;
uint32_t frames = 0;
uint32_t lastFrameUs = 0;
uint32_t startUs = 0;

// Per-variant accumulators. Index 0 = rotate in decode, 1 = LVGL rotates.
uint64_t sumHttp[2]   = {0, 0};
uint64_t sumDecode[2] = {0, 0};
uint64_t sumBlit[2]   = {0, 0};
uint64_t sumFrame[2]  = {0, 0};
uint32_t count[2]     = {0, 0};

uint8_t*  jpegBuf = nullptr;
size_t    jpegLen = 0;
uint16_t* decodeBuf = nullptr;
size_t    decodeBufSize = 0;
uint16_t  frameW = 0;
uint16_t  frameH = 0;

lv_img_dsc_t imgDsc{};
VideoStreamConfig cfg{};

// NOTE: the HTTP and TLS clients are deliberately NOT globals here. A second
// persistent WiFiClientSecure alongside the image fetcher's own costs internal
// RAM this board does not have - free heap is only ~98 KB even at rest, and
// mbedTLS wants roughly 40 KB for a handshake. They are constructed per fetch so
// the TLS context exists only while it is needed.
//
// The cost is a TLS handshake per frame, which inflates the http figure. That is
// acceptable here: this spike measures decode and blit, and Phase 2 will use a
// single long-lived connection.

// True while the first half of the burst is running.
inline bool rotateInDecode() { return frames < FRAMES_PER_VARIANT; }
inline int variantIndex() { return rotateInDecode() ? 0 : 1; }

}  // namespace

static bool fetchFrame(uint32_t* httpUs);
static bool decodeFrame(bool rotate, uint32_t* decodeUs);
static void displayFrame(bool rotated, uint32_t* blitUs);
static void printSummary();
static void returnHome();

//***************************************************************************************************
void videoStreamInit(const VideoStreamConfig& config) {
  cfg = config;
  active = false;
  memset(&imgDsc, 0, sizeof(imgDsc));
}

//***************************************************************************************************
bool videoStreamActive() {
  return active;
}

//***************************************************************************************************
// Blocking GET. Deliberately simple: this spike measures decode and blit, and a
// prefetching client would only blur those numbers. Phase 2 adds prefetch.
static bool fetchFrame(uint32_t* httpUs) {
  const uint32_t t0 = micros();

  // Scoped so the TLS context is released when this returns - see the note at
  // the top of this file about internal RAM.
  HTTPClient httpClient;
  WiFiClientSecure httpsClient;

  // The companion runs on a phone hotspot, so it always uses the remote HTTPS
  // path. The token is appended but never logged - see the note in
  // image_fetcher.cpp about serial output being copied into chat.
  String url = String(IMAGE_SERVER_REMOTE) + "live"
             + "?height=" + String(REQ_HEIGHT)
             + "&quality=" + String(REQ_QUALITY)
             + "&token=" + String(API_TOKEN);

  const uint32_t heapBefore = ESP.getFreeHeap();

  httpsClient.setCACert(remote_server_ca_cert);
  if (!httpClient.begin(httpsClient, url)) {
    USBSerial.println("[VTEST] httpClient.begin() failed");
    return false;
  }
  // Match the timeouts the image fetcher uses on this same host and certificate,
  // since that path is known to work. The handshake timeout in particular was
  // missing here, and its default is very different.
  httpClient.setTimeout(8000);          // socket
  httpClient.setConnectTimeout(5000);   // TCP connect
  httpsClient.setHandshakeTimeout(5);   // seconds, per the setter's units

  const int code = httpClient.GET();
  if (code != HTTP_CODE_OK) {
    // Report enough to tell a RAM problem from a TLS or server problem.
    // largest free block matters: mbedTLS needs a contiguous allocation, so
    // fragmentation can defeat it even when total free looks sufficient.
    char tlsErr[128] = {0};
    httpsClient.lastError(tlsErr, sizeof(tlsErr));
    USBSerial.printf("[VTEST] HTTP %d | heap before %u, now %u, largest block %u | TLS: %s\n",
                     code, heapBefore, ESP.getFreeHeap(),
                     heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                     tlsErr[0] ? tlsErr : "(none reported)");
    USBSerial.printf("[VTEST] url host/path: %slive?height=%u&quality=%u&token=***\n",
                     IMAGE_SERVER_REMOTE, REQ_HEIGHT, REQ_QUALITY);
    httpClient.end();
    return false;
  }

  const int len = httpClient.getSize();
  if (len <= 0 || len > static_cast<int>(MAX_FRAME_BYTES)) {
    USBSerial.printf("[VTEST] bad Content-Length: %d\n", len);
    httpClient.end();
    return false;
  }

  WiFiClient* stream = httpClient.getStreamPtr();
  size_t got = 0;
  const uint32_t deadline = millis() + HTTP_TIMEOUT_MS;
  while (got < static_cast<size_t>(len) && millis() < deadline) {
    const int avail = stream->available();
    if (avail > 0) {
      const size_t want = static_cast<size_t>(len) - got;
      got += stream->readBytes(jpegBuf + got,
                               (static_cast<size_t>(avail) < want) ? avail : want);
    } else {
      yield();
    }
  }
  httpClient.end();

  if (got < static_cast<size_t>(len)) {
    USBSerial.printf("[VTEST] short read: %u of %d\n", got, len);
    return false;
  }

  jpegLen = got;
  *httpUs = micros() - t0;
  return true;
}

//***************************************************************************************************
// Decode with ESP32_JPEG. Output is RGB565 little-endian to match
// LV_COLOR_16_SWAP 0 in this project's lv_conf.h - the home panel uses _BE
// because it has that set to 1. Getting this wrong gives wrong colours rather
// than an obvious failure.
static bool decodeFrame(bool rotate, uint32_t* decodeUs) {
  const uint32_t t0 = micros();

  jpeg_dec_config_t config;
  config.output_type = JPEG_RAW_TYPE_RGB565_LE;
  config.rotate = rotate ? JPEG_ROTATE_90D : JPEG_ROTATE_0D;

  jpeg_dec_handle_t* dec = jpeg_dec_open(&config);
  if (!dec) {
    USBSerial.println("[VTEST] jpeg_dec_open failed");
    return false;
  }

  // ~2.7 KB of Huffman and quantiser tables; the loop task stack is only 8 KB.
  static jpeg_dec_header_info_t info;
  memset(&info, 0, sizeof(info));

  jpeg_dec_io_t io;
  memset(&io, 0, sizeof(io));
  io.inbuf = jpegBuf;
  io.inbuf_len = jpegLen;

  if (jpeg_dec_parse_header(dec, &io, &info) != JPEG_ERR_OK) {
    USBSerial.println("[VTEST] parse_header failed");
    jpeg_dec_close(dec);
    return false;
  }

  // Rotation swaps the output dimensions.
  frameW = rotate ? static_cast<uint16_t>(info.height) : static_cast<uint16_t>(info.width);
  frameH = rotate ? static_cast<uint16_t>(info.width)  : static_cast<uint16_t>(info.height);

  size_t needed = static_cast<size_t>(info.width) * info.height * sizeof(uint16_t);
  needed = (needed + 15u) & ~static_cast<size_t>(15u);
  if (needed > decodeBufSize) {
    if (decodeBuf) heap_caps_free(decodeBuf);
    // The library requires outbuf to be 16-byte aligned; ps_malloc does not
    // guarantee that.
    decodeBuf = static_cast<uint16_t*>(heap_caps_aligned_alloc(16, needed, MALLOC_CAP_SPIRAM));
    decodeBufSize = decodeBuf ? needed : 0;
  }
  if (!decodeBuf) {
    USBSerial.println("[VTEST] PSRAM alloc failed for decode buffer");
    jpeg_dec_close(dec);
    return false;
  }

  io.outbuf = reinterpret_cast<uint8_t*>(decodeBuf);
  const jpeg_error_t err = jpeg_dec_process(dec, &io);
  jpeg_dec_close(dec);

  if (err != JPEG_ERR_OK) {
    USBSerial.printf("[VTEST] decode failed: %d (rotate=%d)\n",
                     static_cast<int>(err), rotate ? 1 : 0);
    return false;
  }

  *decodeUs = micros() - t0;
  return true;
}

//***************************************************************************************************
// Display and time the blit.
//
// Variant A (rotated in decode): frame is 360x640, shown at ROT_NONE in a
//   368x448 space. Crop the height with offset_y.
// Variant B (not rotated): frame is 640x360, shown at ROT_90 in a 448x368
//   space, so LVGL performs the rotation. Crop the width with offset_x.
//
// Either way the widget's own size is never touched: SquareLine sizes it with
// lv_pct(100), and setting an explicit pixel size converts it to fixed sizing,
// which caused a white flash on the home panel.
static void displayFrame(bool rotated, uint32_t* blitUs) {
  lv_disp_t* disp = lv_disp_get_default();
  if (disp) {
    lv_disp_set_rotation(disp, rotated ? LV_DISP_ROT_NONE : LV_DISP_ROT_90);
  }

  imgDsc.header.always_zero = 0;
  imgDsc.header.cf = LV_IMG_CF_TRUE_COLOR;
  imgDsc.header.w = frameW;
  imgDsc.header.h = frameH;
  imgDsc.data_size = static_cast<uint32_t>(frameW) * frameH * LV_COLOR_DEPTH / 8;
  imgDsc.data = reinterpret_cast<const uint8_t*>(decodeBuf);

  if (!cfg.imgVideoBackground) { *blitUs = 0; return; }

  lv_img_set_src(cfg.imgVideoBackground, &imgDsc);
  lv_obj_set_style_opa(cfg.imgVideoBackground, LV_OPA_COVER, LV_PART_MAIN);

  // Visible area depends on the rotation currently applied.
  const int visibleW = rotated ? cfg.screenWidth  : cfg.screenHeight;
  const int visibleH = rotated ? cfg.screenHeight : cfg.screenWidth;

  const int offX = (frameW > visibleW) ? -((frameW - visibleW) / 2) : 0;
  const int offY = (frameH > visibleH) ? -((frameH - visibleH) / 2) : 0;
  lv_img_set_offset_x(cfg.imgVideoBackground, offX);
  lv_img_set_offset_y(cfg.imgVideoBackground, offY);

  // The descriptor pointer does not change between frames, so LVGL must be told
  // explicitly that the content is dirty.
  lv_obj_invalidate(cfg.imgVideoBackground);

  const uint32_t t0 = micros();
  lv_refr_now(NULL);
  *blitUs = micros() - t0;
}

//***************************************************************************************************
static void returnHome() {
  lv_disp_t* disp = lv_disp_get_default();
  if (disp) {
    lv_disp_set_rotation(disp, LV_DISP_ROT_90);  // UI orientation
  }
  if (cfg.screen1) {
    lv_disp_load_scr(cfg.screen1);
  }
}

//***************************************************************************************************
static void printSummary() {
  USBSerial.println("\n[VTEST] ===== SUMMARY =====");
  const char* label[2] = {"A rotate-in-decode (ROT_NONE)", "B LVGL rotates  (ROT_90)   "};

  for (int v = 0; v < 2; v++) {
    if (count[v] == 0) {
      USBSerial.printf("[VTEST] %s : no frames\n", label[v]);
      continue;
    }
    const float http   = sumHttp[v]   / 1000.0f / count[v];
    const float decode = sumDecode[v] / 1000.0f / count[v];
    const float blit   = sumBlit[v]   / 1000.0f / count[v];
    const float frame  = sumFrame[v]  / 1000.0f / count[v];
    USBSerial.printf("[VTEST] %s : n=%2u | http %6.1f | decode %6.1f | blit %6.1f | frame %6.1f ms | %.1f fps\n",
                     label[v], count[v], http, decode, blit, frame,
                     (frame > 0.0f) ? (1000.0f / frame) : 0.0f);
  }

  if (count[0] && count[1]) {
    const float a = (sumDecode[0] + sumBlit[0]) / 1000.0f / count[0];
    const float b = (sumDecode[1] + sumBlit[1]) / 1000.0f / count[1];
    USBSerial.printf("[VTEST] decode+blit: A %.1f ms vs B %.1f ms -> %s is cheaper by %.1f ms\n",
                     a, b, (a < b) ? "A" : "B", (a < b) ? (b - a) : (a - b));
    USBSerial.printf("[VTEST] gate: <80 proceed | 80-150 lower target | >150 reconsider\n");
  }
  USBSerial.printf("[VTEST] free PSRAM %u, free heap %u\n",
                   ESP.getFreePsram(), ESP.getFreeHeap());
}

//***************************************************************************************************
bool videoStreamStart() {
  if (active) return true;

  if (!jpegBuf) {
    jpegBuf = static_cast<uint8_t*>(ps_malloc(MAX_FRAME_BYTES));
    if (!jpegBuf) {
      USBSerial.println("[VTEST] PSRAM alloc failed for JPEG buffer");
      return false;
    }
  }

  USBSerial.printf("\n[VTEST] burst start: %u frames (%u rotate-in-decode, then %u LVGL-rotate)\n",
                   TOTAL_FRAMES, FRAMES_PER_VARIANT, FRAMES_PER_VARIANT);
  USBSerial.printf("[VTEST] source %ux%u, height=%u quality=%u, screen %ux%u\n",
                   SRC_W, SRC_H, REQ_HEIGHT, REQ_QUALITY, cfg.screenWidth, cfg.screenHeight);
  // Internal heap is the constraint on this board, not PSRAM. mbedTLS wants
  // roughly 40 KB for a handshake, so this number decides whether HTTPS can work.
  USBSerial.printf("[VTEST] free heap %u, largest free block %u, free PSRAM %u\n",
                   ESP.getFreeHeap(),
                   heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                   ESP.getFreePsram());

  for (int v = 0; v < 2; v++) {
    sumHttp[v] = sumDecode[v] = sumBlit[v] = sumFrame[v] = 0;
    count[v] = 0;
  }
  frames = 0;
  startUs = micros();
  lastFrameUs = startUs;

  if (cfg.screenVideo && lv_scr_act() != cfg.screenVideo) {
    lv_disp_load_scr(cfg.screenVideo);
  }
  lv_refr_now(NULL);

  active = true;
  return true;
}

//***************************************************************************************************
void videoStreamStop() {
  if (!active) return;

  printSummary();
  active = false;

  if (cfg.imgVideoBackground) {
    lv_obj_set_style_opa(cfg.imgVideoBackground, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_img_set_offset_x(cfg.imgVideoBackground, 0);
    lv_img_set_offset_y(cfg.imgVideoBackground, 0);
  }

  // Buffers are retained: the LVGL widget still references imgDsc, which points
  // into decodeBuf. Freeing here would leave a dangling pointer, and clearing it
  // with lv_img_set_src(NULL) makes LVGL log a warning on every exit. They are
  // released only when the whole burst facility is torn down, which for a spike
  // is never.
}

//***************************************************************************************************
void videoStreamLoop() {
  if (!active) return;

  const bool rotate = rotateInDecode();
  const int v = variantIndex();

  uint32_t httpUs = 0, decodeUs = 0, blitUs = 0;

  if (!fetchFrame(&httpUs)) {
    USBSerial.println("[VTEST] fetch failed, aborting burst");
    videoStreamStop();
    returnHome();
    return;
  }

  if (!decodeFrame(rotate, &decodeUs)) {
    USBSerial.println("[VTEST] decode failed, aborting burst");
    videoStreamStop();
    returnHome();
    return;
  }

  displayFrame(rotate, &blitUs);

  const uint32_t now = micros();
  const uint32_t frameUs = (frames == 0) ? (now - startUs) : (now - lastFrameUs);
  lastFrameUs = now;

  sumHttp[v]   += httpUs;
  sumDecode[v] += decodeUs;
  sumBlit[v]   += blitUs;
  sumFrame[v]  += frameUs;
  count[v]++;
  frames++;

  USBSerial.printf("[VTEST] %c %2u: http %6.1f | decode %6.1f | blit %6.1f | frame %6.1f ms | %ux%u\n",
                   rotate ? 'A' : 'B', frames,
                   httpUs / 1000.0f, decodeUs / 1000.0f,
                   blitUs / 1000.0f, frameUs / 1000.0f, frameW, frameH);

  if (frames == FRAMES_PER_VARIANT) {
    USBSerial.println("[VTEST] --- switching to variant B: LVGL rotates at ROT_90 ---");
  }

  if (frames >= TOTAL_FRAMES) {
    videoStreamStop();
    returnHome();
  }
}
