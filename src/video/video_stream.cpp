#include "video_stream.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ESP32_JPEG_Library.h>
#include <esp_heap_caps.h>

#include "HWCDC.h"
#include "secrets_private.h"
#include "ui.h"
#include "../image/image_fetcher.h"  // for the shared TLS client

// Project module pattern - see CLAUDE.md. The other modules do the same.
extern HWCDC USBSerial;

namespace {

// --- Feed configuration ------------------------------------------------------
constexpr unsigned long VIDEO_DURATION_MS = 60000;          // feed length
// 392, not 360. Frigate honours this exactly and derives the width from the
// source aspect ratio, so 392 yields 696x392, which the decoder rotates to
// 392x696. That covers the 368x448 panel with 12 px trimmed from each side.
//
// 360 gave 640x360 -> 360x640, eight pixels narrower than the panel, leaving an
// uncovered strip the screen background showed through along one edge.
//
// The obvious 368 does NOT work: it returns 654x368, and 654 % 8 != 0. The
// ESP32_JPEG header states rotation is only supported when both dimensions are
// multiples of 8, so 368 would silently disable it. 392 -> 696x392 satisfies
// both. Verified against the server; do not change without re-checking the
// returned width against that rule.
//
// Cost is small: the JPEG grows ~4% (the cellular round trip dominates and
// scales with bytes) and decode ~18%. The blit is unaffected - it is clipped to
// the visible area, so surplus pixels are never read.
constexpr uint16_t REQ_HEIGHT = 392;                        // 696x392 from Frigate
constexpr uint8_t  REQ_QUALITY = 25;

// Horizontal pan of the visible window, in pixels, as the viewer sees it.
//
// The frame is 392x696 against a 368x448 panel, so 248 px - about a third of the
// camera's width - is cropped away. Centred, that is 124 px lost from each side.
// This shifts which slice is kept, to favour one side of the scene.
//
// It is offY that pans horizontally, not offX: the decoder rotates the frame
// 270 degrees, so the camera's wide axis becomes the frame's tall axis. offX
// moves the image up and down and has only 24 px of travel.
//
//    0   centred, 124 px lost each side
// -124   hard against the door side - measured on the bench, this is the
//        direction that reveals the door frame
//  124   hard against the far side
//
// Values beyond +/-124 are clamped and simply do nothing. Reduce the magnitude
// to trade some of the door view back for the far side.
constexpr int PAN_X = -30;  //-124 max towards the door, 0 centred, +124 max towards far side

// Vertical pan, same idea on the other axis - and far more limited.
//
// The frame is only 392 wide against a 368 px panel, so just 24 px is cropped
// on this axis: 12 from the top and 12 from the bottom. PAN_Y has +/-12 px of
// travel, about 3 % of the view. Enough for a nudge, not a reframe.
//
//   12   hard against one edge, keeping frame columns 0-367
//    0   centred
//  -12   hard against the other edge, keeping columns 24-391
//
// This one is offX, because the 270 degree rotation puts the camera's short
// axis along the frame's width. Sign not derived - flip it if it nudges the
// wrong way, exactly as PAN_X needed.
//
// For real vertical travel the frame has to be taller relative to the panel,
// which means a larger REQ_HEIGHT: 432 gives +/-32 px, 464 gives +/-48. Both
// satisfy the divisible-by-8 rule, and both cost bytes and decode time.
constexpr int PAN_Y = 12; 
constexpr size_t   MAX_FRAME_BYTES = 64000;                 // frames measure ~23 KB
constexpr uint32_t HTTP_TIMEOUT_MS = 15000;

// --- State -------------------------------------------------------------------
bool active = false;
unsigned long startMs = 0;
uint32_t frames = 0;
uint32_t lastFrameUs = 0;
uint32_t startUs = 0;

// --- Heap corruption diagnostic ----------------------------------------------
// DIAGNOSTIC ONLY - set VIDEO_HEAP_DEBUG to 0 to remove.
//
// The feed dies after 10-20 s inside the WiFi task's own free(), with TLSF
// following a corrupt free-list pointer (StoreProhibited at prev_free, block
// pointer = garbage). The internal heap is therefore already damaged by the time
// WiFi trips over it, so the crash site identifies the victim, not the culprit.
//
// These checks walk the internal heap at each stage boundary to find the first
// moment it is damaged. The check is slow - it visits every block - but at ~2 fps
// that is affordable, and it is far cheaper than guessing.
//
// Reports once and stops the feed, so the message reaches the serial monitor
// before the WiFi task panics.
#define VIDEO_HEAP_DEBUG 1

#if VIDEO_HEAP_DEBUG
bool heapReported = false;

bool heapCheck(const char* where) {
  if (heapReported) return true;                  // already reported, stop nagging
  if (heap_caps_check_integrity(MALLOC_CAP_INTERNAL, true)) return true;
  heapReported = true;
  USBSerial.printf("Video: *** INTERNAL HEAP CORRUPT after %s | frame %u | free %u | largest %u ***\n",
                   where, frames, ESP.getFreeHeap(),
                   heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
  return false;
}
#define HEAP_CHECK(w) heapCheck(w)
#else
#define HEAP_CHECK(w) (true)
#endif

uint64_t sumHttp = 0;
uint64_t sumDecode = 0;
uint64_t sumBlit = 0;
uint64_t sumFrame = 0;

uint8_t*  jpegBuf = nullptr;
size_t    jpegLen = 0;
uint16_t* decodeBuf = nullptr;
size_t    decodeBufSize = 0;
uint16_t  frameW = 0;
uint16_t  frameH = 0;

lv_img_dsc_t imgDsc{};
VideoStreamConfig cfg{};

// NOTE: this module owns no TLS client. It borrows the image fetcher's via
// imageFetcherSecureClient().
//
// Measured on this board: ~48 KB free heap but only ~32 KB in the largest
// contiguous block. mbedTLS needs contiguous ~16 KB in/out buffers, so a second
// WiFiClientSecure - whether global or function-local - fails its handshake with
// "SSL - Memory allocation failed". The image fetcher's client is constructed at
// startup while the heap is still unfragmented, which is why it succeeds.
//
// This is a real constraint on the port, not a workaround: Phase 2's prefetching
// client must share the same TLS client rather than open its own.
//
// The HTTPClient itself is cheap in RAM, so it IS a module global - it has to be,
// because keep-alive only works if the same instance persists across frames.
// Measured: with a fresh handshake per frame, http was 906 ms over cellular and
// dominated 84% of the frame time.
HTTPClient httpClient;

}  // namespace

static bool fetchFrame(uint32_t* httpUs);
static bool decodeFrame(uint32_t* decodeUs);
static void displayFrame(uint32_t* blitUs);
static void printSummary();
static void returnHome();
static void screenVideo_event_handler(lv_event_t* e);

//***************************************************************************************************
void videoStreamInit(const VideoStreamConfig& config) {
  cfg = config;
  active = false;
  memset(&imgDsc, 0, sizeof(imgDsc));

  // Leaving the screen must stop the feed. Without this the feed kept running
  // after the user navigated away, and displayFrame() went on forcing
  // LV_DISP_ROT_NONE, so Screen 1 appeared rotated 90 degrees until the feed's
  // 60 seconds expired and returnHome() restored ROT_90.
  if (cfg.screenVideo) {
    lv_obj_add_event_cb(cfg.screenVideo, screenVideo_event_handler,
                        LV_EVENT_SCREEN_UNLOAD_START, NULL);
  }
}

//***************************************************************************************************
// Any departure from the video screen ends the feed - the back button, or
// anything else that changes screens.
static void screenVideo_event_handler(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_SCREEN_UNLOAD_START) return;
  if (active) {
    USBSerial.println("Video: screen left, stopping feed");
    videoStreamStop();
  }
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

  // Borrow the image fetcher's TLS client rather than making a second one.
  // Measured: with ~48 KB free heap but only ~32 KB in the largest contiguous
  // block, a second WiFiClientSecure fails its handshake with
  // "SSL - Memory allocation failed". The shared one is constructed at startup
  // when the heap is unfragmented, which is why it works. Safe because the two
  // modules never fetch at the same time.
  WiFiClientSecure* secure = imageFetcherSecureClient();
  if (!secure) {
    USBSerial.println("Video: no shared TLS client available");
    return false;
  }

  WiFiClientSecure& httpsClient = *secure;

  // The companion runs on a phone hotspot, so it always uses the remote HTTPS
  // path. The token is appended but never logged - see the note in
  // image_fetcher.cpp about serial output being copied into chat.
  String url = String(IMAGE_SERVER_REMOTE) + "live"
             + "?height=" + String(REQ_HEIGHT)
             + "&quality=" + String(REQ_QUALITY)
             + "&token=" + String(API_TOKEN);

  const uint32_t heapBefore = ESP.getFreeHeap();

  // Keep the TLS session open between frames. Must be set before begin(), which
  // latches the flag. This is the single biggest win available: a handshake per
  // frame cost 906 ms over cellular.
  httpClient.setReuse(true);

  httpsClient.setCACert(remote_server_ca_cert);
  if (!httpClient.begin(httpsClient, url)) {
    USBSerial.println("Video: httpClient.begin() failed");
    return false;
  }
  // Match the timeouts the image fetcher uses on this same host and certificate,
  // since that path is known to work. The handshake timeout in particular was
  // missing here, and its default is very different.
  if (!HEAP_CHECK("http begin")) return false;

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
    USBSerial.printf("Video: HTTP %d | heap before %u, now %u, largest block %u | TLS: %s\n",
                     code, heapBefore, ESP.getFreeHeap(),
                     heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                     tlsErr[0] ? tlsErr : "(none reported)");
    USBSerial.printf("Video: url host/path: %slive?height=%u&quality=%u&token=***\n",
                     IMAGE_SERVER_REMOTE, REQ_HEIGHT, REQ_QUALITY);
    httpClient.end();
    return false;
  }

  if (!HEAP_CHECK("http GET")) { httpClient.end(); return false; }

  const int len = httpClient.getSize();
  if (len <= 0 || len > static_cast<int>(MAX_FRAME_BYTES)) {
    USBSerial.printf("Video: bad Content-Length: %d\n", len);
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
  if (!HEAP_CHECK("body read")) { httpClient.end(); return false; }

  httpClient.end();

  if (!HEAP_CHECK("http end")) return false;

  if (got < static_cast<size_t>(len)) {
    USBSerial.printf("Video: short read: %u of %d\n", got, len);
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
static bool decodeFrame(uint32_t* decodeUs) {
  const uint32_t t0 = micros();

  jpeg_dec_config_t config;
  config.output_type = JPEG_RAW_TYPE_RGB565_LE;
  // 270 rather than 90: measured on hardware, 90D came out upside down for this
  // panel's orientation. Same divisible-by-8 constraint applies either way.
  config.rotate = JPEG_ROTATE_270D;

  jpeg_dec_handle_t* dec = jpeg_dec_open(&config);
  if (!dec) {
    USBSerial.println("Video: jpeg_dec_open failed");
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
    USBSerial.println("Video: parse_header failed");
    jpeg_dec_close(dec);
    return false;
  }

  // Use the reported dimensions as-is. ESP32_JPEG already accounts for rotation
  // in out_info, so a 640x360 source decoded with JPEG_ROTATE_90D reports
  // 360x640. Swapping them here as well double-swapped the descriptor: LVGL was
  // told 640x360 while the buffer held 360x640, which rendered as an
  // unrecognisable image with wrong colours.
  frameW = static_cast<uint16_t>(info.width);
  frameH = static_cast<uint16_t>(info.height);

  // Geometry report, to size the request against this panel. Frigate derives the
  // width from the source aspect ratio and knows nothing about the screen, so the
  // rotated frame can land narrower than the panel; that shortfall is the
  // uncovered strip the background shows through. Positive gap means uncovered,
  // negative means cropped away. Printed only when the dimensions change, so it
  // costs one line per feed rather than one per frame.
  //
  // This sits inside the decode timing window, so whichever frame prints it
  // reports an inflated decode time. Discard that frame when measuring fps.
  static uint16_t loggedW = 0;
  static uint16_t loggedH = 0;
  if (frameW != loggedW || frameH != loggedH) {
    loggedW = frameW;
    loggedH = frameH;
    USBSerial.printf("Video: frame %ux%u, panel %ux%u -> gap x=%d y=%d, pan x=%d y=%d\n",
                     frameW, frameH, cfg.screenWidth, cfg.screenHeight,
                     static_cast<int>(cfg.screenWidth) - static_cast<int>(frameW),
                     static_cast<int>(cfg.screenHeight) - static_cast<int>(frameH),
                     PAN_X, PAN_Y);
  }

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
    USBSerial.println("Video: PSRAM alloc failed for decode buffer");
    jpeg_dec_close(dec);
    return false;
  }

  io.outbuf = reinterpret_cast<uint8_t*>(decodeBuf);
  const jpeg_error_t err = jpeg_dec_process(dec, &io);
  jpeg_dec_close(dec);

  if (err != JPEG_ERR_OK) {
    USBSerial.printf("Video: decode failed: %d\n", static_cast<int>(err));
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
static void displayFrame(uint32_t* blitUs) {
  // Never touch the display while another screen owns it. Forcing rotation from
  // here after the user had navigated away is what left Screen 1 sideways.
  if (cfg.screenVideo && lv_scr_act() != cfg.screenVideo) {
    *blitUs = 0;
    return;
  }

  // The frame is already rotated by the decoder, so the display stays at
  // ROT_NONE - the same orientation the still images use.
  lv_disp_t* disp = lv_disp_get_default();
  if (disp) {
    lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);
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

  // At ROT_NONE the visible area is the screen's natural orientation.
  const int visibleW = cfg.screenWidth;
  const int visibleH = cfg.screenHeight;

  // Centre each axis, then apply the pan and clamp. The clamps are what make
  // PAN_X and PAN_Y safe to edit: neither offset may go positive or past the
  // far edge of the frame, either of which would leave part of the widget
  // unpainted and bring back the uncovered strip.
  int offX = 0;
  if (frameW > visibleW) {
    const int minOff = -(static_cast<int>(frameW) - visibleW);    // -24 at 392x696
    offX = minOff / 2 + PAN_Y;
    if (offX > 0) offX = 0;
    if (offX < minOff) offX = minOff;
  }

  int offY = 0;
  if (frameH > visibleH) {
    const int minOff = -(static_cast<int>(frameH) - visibleH);   // -248 at 392x696
    offY = minOff / 2 + PAN_X;
    if (offY > 0) offY = 0;
    if (offY < minOff) offY = minOff;
  }
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
  const float secs = (millis() - startMs) / 1000.0f;
  if (frames == 0 || secs <= 0.0f) {
    USBSerial.println("Video: stopped with no frames");
    return;
  }
  USBSerial.printf("Video: %u frames in %.1fs (%.1f fps) | http %.0f | decode %.0f | blit %.0f | frame %.0f ms\n",
                   frames, secs, frames / secs,
                   sumHttp / 1000.0f / frames, sumDecode / 1000.0f / frames,
                   sumBlit / 1000.0f / frames, sumFrame / 1000.0f / frames);
  USBSerial.printf("Video: free PSRAM %u, free heap %u\n",
                   ESP.getFreePsram(), ESP.getFreeHeap());
}

//***************************************************************************************************
bool videoStreamStart() {
  if (active) return true;

  // Allocated on first use and retained - see videoStreamStop().
  if (!jpegBuf) {
    jpegBuf = static_cast<uint8_t*>(ps_malloc(MAX_FRAME_BYTES));
    if (!jpegBuf) {
      USBSerial.println("Video: PSRAM alloc failed for JPEG buffer");
      return false;
    }
  }

  sumHttp = sumDecode = sumBlit = sumFrame = 0;
  frames = 0;
  startMs = millis();
  startUs = micros();
  lastFrameUs = startUs;

  if (cfg.screenVideo && lv_scr_act() != cfg.screenVideo) {
    lv_disp_load_scr(cfg.screenVideo);
  }
  lv_refr_now(NULL);

  // Baseline. If this fires, the damage predates the feed entirely and the
  // still-image fetch or the UI is the place to look, not the video path.
  HEAP_CHECK("feed start, before any frame");

  active = true;
  return true;
}

//***************************************************************************************************
void videoStreamStop() {
  if (!active) return;

  printSummary();
  active = false;

  // Release the kept-alive connection so the shared TLS client is idle again for
  // the still-image path.
  httpClient.setReuse(false);
  httpClient.end();

  if (cfg.imgVideoBackground) {
    lv_obj_set_style_opa(cfg.imgVideoBackground, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_img_set_offset_x(cfg.imgVideoBackground, 0);
    lv_img_set_offset_y(cfg.imgVideoBackground, 0);
  }

  // Buffers are deliberately retained. The LVGL widget still references imgDsc,
  // which points into decodeBuf, so freeing it would leave a dangling pointer -
  // and clearing it with lv_img_set_src(NULL) makes LVGL log a warning on every
  // exit. Retaining also avoids cycling ~460 KB through PSRAM on every press.
  // Nothing is allocated at all if the feed is never used.
}

//***************************************************************************************************
void videoStreamLoop() {
  if (!active) return;

  uint32_t httpUs = 0, decodeUs = 0, blitUs = 0;

  if (!fetchFrame(&httpUs)) {
    USBSerial.println("Video: fetch failed, stopping");
    videoStreamStop();
    returnHome();
    return;
  }

  // Let LVGL run between the blocking stages. The touch controller is only
  // sampled inside lv_timer_handler(), and the fetch above blocks for ~350 ms,
  // so with one call per frame a quick tap on the back button could land
  // entirely inside a blocking section and never be seen. Servicing LVGL here
  // and after the decode gives three sampling opportunities per frame instead of
  // one.
  //
  // A tap may change screens, which fires screenVideo_event_handler and stops
  // the feed - hence the active check before continuing.
  if (!HEAP_CHECK("fetch")) { videoStreamStop(); returnHome(); return; }

  lv_timer_handler();
  if (!active) return;

  if (!decodeFrame(&decodeUs)) {
    USBSerial.println("Video: decode failed, stopping");
    videoStreamStop();
    returnHome();
    return;
  }

  if (!HEAP_CHECK("decode")) { videoStreamStop(); returnHome(); return; }

  lv_timer_handler();
  if (!active) return;

  displayFrame(&blitUs);

  if (!HEAP_CHECK("blit")) { videoStreamStop(); returnHome(); return; }

  const uint32_t now = micros();
  sumFrame += (frames == 0) ? (now - startUs) : (now - lastFrameUs);
  lastFrameUs = now;
  sumHttp += httpUs;
  sumDecode += decodeUs;
  sumBlit += blitUs;
  frames++;

  if (millis() - startMs >= VIDEO_DURATION_MS) {
    videoStreamStop();
    returnHome();
  }
}
