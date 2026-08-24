#include "video_stream.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ESP32_JPEG_Library.h>
#include <esp_heap_caps.h>
#include <string.h>
#include <stdlib.h>
#include <strings.h>                 // strncasecmp, for the header scan

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
// 432 -> 768x432 from Frigate, rotated by the decoder to 432x768.
//
// TWO rules constrain this value, and missing the second one cost a day of
// crash hunting:
//
//  1. Both dimensions must be multiples of 8, or ESP32_JPEG silently disables
//     rotation ("Under width % 8 == 0. height % 8 = 0 conditions, rotation
//     enabled. Otherwise unsupported" - esp_jpeg_dec.h).
//
//  2. The SOURCE WIDTH must be a multiple of 16. These JPEGs are 4:2:0, so the
//     decoder works in 16x16 MCU blocks and pads the image up to that grid.
//     392 gave a 696 px source width - 43.5 MCUs - and the board began dying
//     inside tlsf_free a few seconds into every feed, with internal-heap block
//     headers overwritten. 360 (640 px wide, 40 MCUs exactly) was stable, and
//     reverting to it made the crashes stop. Height need not be aligned: both
//     360 and 392 have unaligned heights and only 392 crashed.
//
// 768 is 48 MCUs exactly. Verified against the server; re-check BOTH rules
// against the returned width before ever changing this.
constexpr uint16_t REQ_HEIGHT = 432;                        // 768x432 from Frigate
// Frigate's JPEG quality. The Node-RED endpoint clamps it to 10-80, so 10 is
// the floor, not 0.
//
// Bytes are the largest term in the frame budget. Measured on device at q25:
// `xfer` 252 ms of a 396 ms `http`, moving 25.0 KB at 99 KB/s, so 64% of the
// network time is spent shifting the body. `ttfb` (144 ms) does not change with
// frame size, so quality only ever acts on `xfer`.
//
// Sizes measured against this scene at height=432, roughly 800 bytes per point:
//   q10 15.9 KB   q15 20.7 KB   q20 24.7 KB   q25 28.5 KB   q40 38.0 KB
//
// 15 rather than 10: at q10 the brick courses, the mortar lines and the doormat
// grid all smear, and this camera exists to identify who is at the door. q15
// kept them.
constexpr uint8_t  REQ_QUALITY = 15;                        // frames ~18 KB

// Horizontal pan of the visible window, in pixels, as the viewer sees it.
//
// At 432 the frame is 432x768 against a 368x448 panel, so 320 px is cropped on
// this axis - centred, that is 160 px lost from each side. This shifts which
// slice is kept, to favour one side of the scene.
//
// It is offY that pans horizontally, not offX: the decoder rotates the frame
// 270 degrees, so the camera's wide axis becomes the frame's tall axis.
//
//    160   hard against the far side, keeping frame rows 0-447
//      0   centred, 160 px lost each side, rows 160-607
//   -160   hard against the door side, rows 320-767
//
// Negative is the door direction - measured on the bench, not derived.
// Values beyond +/-160 are clamped and simply do nothing.
constexpr int PAN_X = -15;  //-160 max towards the door, 0 centred, +160 max towards far side

// Vertical pan, same idea on the other axis.
//
// The frame is 432 wide against a 368 px panel, so 64 px is cropped here: 32
// from each edge. Travel is +/-32 px.
//
//     32   hard against one edge, keeping frame columns 0-367
//      0   centred, columns 32-399
//    -32   hard against the other edge, columns 64-431
//
// This one is offX, because the 270 degree rotation puts the camera's short
// axis along the frame's width.
//
// Both ranges scale with REQ_HEIGHT, but do not raise it just to gain travel:
// the next values up (440, 448, 464) all fail the multiple-of-16 source width
// rule above and corrupt the heap. 432 is the only legal size in that region.
constexpr int PAN_Y = 32;   //-32 max one way, 0 centred, +32 max the other way
constexpr size_t   MAX_FRAME_BYTES = 64000;                 // frames measure ~18 KB
constexpr uint32_t HTTP_TIMEOUT_MS = 15000;                 // per request, from send

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
// Set to 1 to re-enable if heap corruption is ever suspected again. Costs
// roughly 8 ms per frame. Left in place because it earned its keep once.
#define VIDEO_HEAP_DEBUG 0

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
// `http` split into its two halves. `ttfb` is everything up to the first byte of
// the response - the round trip plus the Synology -> Pi proxy -> Node-RED ->
// Frigate chain - and does not change with frame size. `xfer` is draining the
// body off the socket, and is the only part that scales with bytes.
//
// Kept permanently rather than behind a VIDEO_HEAP_DEBUG-style toggle: it costs
// two micros() calls per frame and two more numbers on a line already printed
// once per feed. The home panel measured per-frame Serial output at 11 ms a
// frame, so this must never grow into per-frame logging.
uint64_t sumTtfb = 0;
uint64_t sumXfer = 0;
uint64_t sumBytes = 0;
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
// Prefetch does not need a second connection, only the one it already has driven
// without blocking, so that constraint costs nothing here.

// --- Non-blocking client -----------------------------------------------------
// HTTPClient is gone from this path. GET() blocks until the response headers
// arrive, which is exactly what prefetch must avoid: the request for frame N+1 is
// issued before frame N is decoded and blitted, so ~280 ms of network wait
// overlaps ~182 ms of rendering instead of following it.
//
// The TLS client is driven as a plain byte stream. available() and read() return
// decrypted bytes and never block - the previous blocking implementation already
// relied on exactly that for its body read - so mbedTLS does all the hard work
// and this code only has to speak HTTP/1.1.
//
// Kept deliberately minimal because we control the server: the Node-RED endpoint
// answers 200 with Content-Length and keep-alive, never chunked, no redirects.
//
// There is no second JPEG buffer. While frame N is decoded and blitted, the
// response for N+1 accumulates in lwIP's socket buffer, not ours.
WiFiClientSecure* vidClient = nullptr;      // borrowed, never owned or deleted

// Endpoint split out of IMAGE_SERVER_REMOTE once per feed. A raw client takes
// host and port separately where HTTPClient took the whole URL. Parsed rather
// than duplicated into secrets_private.h so the two cannot drift apart.
char     epHost[96] = {0};
uint16_t epPort = 443;
char     epPath[96] = {0};

bool     reqInFlight = false;
// Set when the mid-frame drain completes the NEXT frame while the current one is
// still on screen. Without it videoStreamLoop() would poll again on the following
// pass, get 0 because nothing is in flight any more, and stall forever.
bool     frameReady = false;
bool     hdrDone = false;
int      contentLen = -1;
size_t   jpegReceived = 0;
char     hdrBuf[512];
size_t   hdrLen = 0;

uint32_t reqSentMs = 0;                     // timeout base
uint32_t reqSentUs = 0;                     // ttfb base
uint32_t hdrDoneUs = 0;                     // ttfb / xfer boundary
uint32_t bodyDoneUs = 0;

}  // namespace

static bool parseEndpoint();
static bool ensureConnected();
static void closeConnection();
static int  headerInt(const char* name);
static bool sendRequest();
static int  pollResponse();
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
// Split IMAGE_SERVER_REMOTE ("https://host:port/path/") into its parts, once per
// feed. A raw client needs host and port separately; parsing beats duplicating
// them into secrets_private.h, where the copy could drift from the original.
static bool parseEndpoint() {
  const char* p = IMAGE_SERVER_REMOTE;
  if (strncmp(p, "https://", 8) != 0) {
    USBSerial.println("Video: IMAGE_SERVER_REMOTE is not an https:// URL");
    return false;
  }
  p += 8;

  const char* slash = strchr(p, '/');
  const char* colon = strchr(p, ':');
  if (colon && slash && colon > slash) colon = nullptr;   // a colon inside the path

  const size_t hostLen = colon ? static_cast<size_t>(colon - p)
                               : (slash ? static_cast<size_t>(slash - p) : strlen(p));
  if (hostLen == 0 || hostLen >= sizeof(epHost)) {
    USBSerial.println("Video: cannot parse host from IMAGE_SERVER_REMOTE");
    return false;
  }
  memcpy(epHost, p, hostLen);
  epHost[hostLen] = '\0';

  epPort = colon ? static_cast<uint16_t>(atoi(colon + 1)) : 443;
  if (epPort == 0) epPort = 443;

  // The path keeps its trailing slash; "live" is appended to it per request.
  if (slash) {
    if (strlen(slash) >= sizeof(epPath)) {
      USBSerial.println("Video: path too long in IMAGE_SERVER_REMOTE");
      return false;
    }
    strcpy(epPath, slash);
  } else {
    strcpy(epPath, "/");
  }

  // Host deliberately not logged - serial output gets pasted into chat, which is
  // the same reasoning that keeps the token out of any log below.
  USBSerial.printf("Video: endpoint parsed, port %u, path %slive\n", epPort, epPath);
  return true;
}

//***************************************************************************************************
// Connect and handshake. Blocking, but only once per feed - the connection is
// then held open for all 130-odd frames. A handshake per frame cost 906 ms and
// dominated 84% of the frame time.
static bool ensureConnected() {
  if (!vidClient) return false;
  if (vidClient->connected()) return true;

  const uint32_t heapBefore = ESP.getFreeHeap();

  vidClient->stop();
  vidClient->setCACert(remote_server_ca_cert);
  vidClient->setHandshakeTimeout(5);        // seconds, per the setter's units

  if (!vidClient->connect(epHost, epPort)) {
    // Report enough to tell a RAM problem from a TLS or server problem. Largest
    // free block matters: mbedTLS needs a contiguous allocation, so fragmentation
    // can defeat it even when total free looks sufficient.
    char tlsErr[128] = {0};
    vidClient->lastError(tlsErr, sizeof(tlsErr));
    USBSerial.printf("Video: connect failed | heap before %u, now %u, largest block %u | TLS: %s\n",
                     heapBefore, ESP.getFreeHeap(),
                     heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                     tlsErr[0] ? tlsErr : "(none reported)");
    return false;
  }
  return true;
}

//***************************************************************************************************
// The single teardown. The TLS client is shared with the image fetcher, so
// abandoning a half-read response would leave the next still fetch reading video
// bytes. A feed has five ways out - the 60 s expiry, a fetch error, a decode
// error, the back button, and screen unload - and all of them reach this through
// videoStreamStop().
static void closeConnection() {
  reqInFlight = false;
  frameReady = false;
  hdrDone = false;
  hdrLen = 0;
  contentLen = -1;
  jpegReceived = 0;
  if (vidClient) {
    vidClient->stop();
    vidClient = nullptr;
  }
}

//***************************************************************************************************
// Case-insensitive header lookup. Returns -1 if absent.
static int headerInt(const char* name) {
  const size_t nameLen = strlen(name);
  for (size_t i = 0; i + nameLen < hdrLen; i++) {
    if (strncasecmp(hdrBuf + i, name, nameLen) == 0) {
      const char* q = hdrBuf + i + nameLen;
      while (*q == ' ' || *q == ':') q++;
      return atoi(q);
    }
  }
  return -1;
}

//***************************************************************************************************
// Write the GET and return immediately. pollResponse() collects the answer later,
// which is what lets the network wait overlap decode and blit.
static bool sendRequest() {
  if (!ensureConnected()) return false;

  hdrLen = 0;
  hdrDone = false;
  contentLen = -1;
  jpegReceived = 0;

  // The token is in the request line, so `req` must never be logged - the same
  // rule image_fetcher.cpp follows.
  char req[640];
  const int n = snprintf(req, sizeof(req),
                         "GET %slive?height=%u&quality=%u&token=%s HTTP/1.1\r\n"
                         "Host: %s:%u\r\n"
                         "Connection: keep-alive\r\n"
                         "Accept: image/jpeg\r\n\r\n",
                         epPath, static_cast<unsigned>(REQ_HEIGHT),
                         static_cast<unsigned>(REQ_QUALITY), API_TOKEN,
                         epHost, static_cast<unsigned>(epPort));
  if (n <= 0 || n >= static_cast<int>(sizeof(req))) {
    USBSerial.println("Video: request did not fit its buffer");
    return false;
  }

  reqSentMs = millis();
  reqSentUs = micros();
  if (vidClient->write(reinterpret_cast<const uint8_t*>(req),
                       static_cast<size_t>(n)) != static_cast<size_t>(n)) {
    USBSerial.println("Video: request write failed");
    return false;
  }

  reqInFlight = true;
  return true;
}

//***************************************************************************************************
// Drain whatever has arrived. Never blocks.
// 1 = a complete frame is in jpegBuf, 0 = still receiving, -1 = error.
static int pollResponse() {
  if (!reqInFlight) return 0;
  if (!vidClient) return -1;

  if (millis() - reqSentMs > HTTP_TIMEOUT_MS) {
    USBSerial.println("Video: response timeout");
    return -1;
  }

  // Buffered bytes still count after a close, so check both before giving up.
  if (!vidClient->connected() && vidClient->available() == 0) {
    USBSerial.println("Video: connection closed mid-response");
    return -1;
  }

  if (!hdrDone) {
    while (vidClient->available() && hdrLen < sizeof(hdrBuf) - 1) {
      hdrBuf[hdrLen++] = static_cast<char>(vidClient->read());
      if (hdrLen >= 4 && hdrBuf[hdrLen - 4] == '\r' && hdrBuf[hdrLen - 3] == '\n' &&
          hdrBuf[hdrLen - 2] == '\r' && hdrBuf[hdrLen - 1] == '\n') {
        hdrBuf[hdrLen] = '\0';
        hdrDone = true;
        break;
      }
    }

    if (!hdrDone) {
      if (hdrLen >= sizeof(hdrBuf) - 1) {
        USBSerial.println("Video: header overflow");
        return -1;
      }
      return 0;                            // headers still arriving
    }

    hdrDoneUs = micros();

    if (!strstr(hdrBuf, " 200 ")) {
      USBSerial.println("Video: non-200 response");
      return -1;
    }

    contentLen = headerInt("Content-Length");
    if (contentLen <= 0 || contentLen > static_cast<int>(MAX_FRAME_BYTES)) {
      USBSerial.printf("Video: bad Content-Length: %d\n", contentLen);
      return -1;
    }
  }

  while (vidClient->available() && jpegReceived < static_cast<size_t>(contentLen)) {
    const size_t want = static_cast<size_t>(contentLen) - jpegReceived;
    const size_t avail = static_cast<size_t>(vidClient->available());
    const int got = vidClient->read(jpegBuf + jpegReceived, (avail < want) ? avail : want);
    if (got <= 0) break;
    jpegReceived += static_cast<size_t>(got);
  }

  if (jpegReceived >= static_cast<size_t>(contentLen)) {
    bodyDoneUs = micros();
    jpegLen = jpegReceived;
    reqInFlight = false;
    return 1;
  }

  return 0;                                // body still arriving
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
  // Throughput is bytes over the xfer time alone, so it measures the link while
  // it is actually moving data instead of averaging in the wait for the server.
  const float xferSecs = sumXfer / 1000000.0f;
  USBSerial.printf("Video: http = ttfb %.0f + xfer %.0f ms | frame %.1f KB | %.0f KB/s while transferring\n",
                   sumTtfb / 1000.0f / frames, sumXfer / 1000.0f / frames,
                   sumBytes / 1024.0f / frames,
                   (xferSecs > 0.0f) ? (sumBytes / 1024.0f / xferSecs) : 0.0f);
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

  sumHttp = sumTtfb = sumXfer = sumBytes = 0;
  sumDecode = sumBlit = sumFrame = 0;
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

  if (!parseEndpoint()) return false;

  vidClient = imageFetcherSecureClient();
  if (!vidClient) {
    USBSerial.println("Video: no shared TLS client available");
    return false;
  }

  active = true;

  // Prime the pipeline. Every later request is issued by videoStreamLoop() the
  // moment the previous frame lands, so this is the only one sent from here.
  if (!sendRequest()) {
    USBSerial.println("Video: first request failed");
    videoStreamStop();
    return false;
  }

  return true;
}

//***************************************************************************************************
void videoStreamStop() {
  if (!active) return;

  printSummary();
  active = false;

  // Release the connection so the shared TLS client is idle and clean for the
  // still-image path. Every exit route reaches this one call.
  closeConnection();

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

  // Collect whatever has arrived for the frame already asked for. Returns at
  // once if it is incomplete, so the UI keeps running while the network works.
  // The mid-frame drain below can complete the next frame while this one is still
  // being displayed, in which case there is nothing left to poll for.
  if (!frameReady) {
    const int r = pollResponse();
    if (r < 0) {
      USBSerial.println("Video: fetch failed, stopping");
      videoStreamStop();
      returnHome();
      return;
    }
    if (r == 0) {
      // Nothing more to do this pass. runBackgroundTick() in the main loop
      // already ran lv_timer_handler() and the IMU read before getting here, and
      // returning straight away is what lets both run at full rate during the
      // wait - which is why the IMU recovers to ~50 Hz during a feed.
      return;
    }
  }
  frameReady = false;

  // The frame is in jpegBuf and its network cost is already known.
  const uint32_t ttfbUs = hdrDoneUs - reqSentUs;
  const uint32_t xferUs = bodyDoneUs - hdrDoneUs;
  const uint32_t httpUs = bodyDoneUs - reqSentUs;

  const bool more = (millis() - startMs) < VIDEO_DURATION_MS;

  // The prefetch itself, and the whole of Phase 2: ask for the next frame before
  // spending ~182 ms decoding and blitting this one, so the wait for it overlaps
  // that work instead of following it. Because http exceeds decode+blit, the
  // rendering disappears into the wait entirely.
  //
  // Note ttfb now measures from this send to the headers arriving, which spans
  // the decode and blit below. A ttfb close to decode+blit is the proof that the
  // overlap is real; it is not a regression.
  if (more && !sendRequest()) {
    USBSerial.println("Video: prefetch failed, stopping");
    videoStreamStop();
    returnHome();
    return;
  }

  if (!HEAP_CHECK("fetch")) { videoStreamStop(); returnHome(); return; }

  // Let LVGL run between the heavy stages. The touch controller is only sampled
  // inside lv_timer_handler(), and a tap may change screens, which fires
  // screenVideo_event_handler and stops the feed - hence the active checks.
  lv_timer_handler();
  if (!active) return;

  uint32_t decodeUs = 0, blitUs = 0;

  if (!decodeFrame(&decodeUs)) {
    USBSerial.println("Video: decode failed, stopping");
    videoStreamStop();
    returnHome();
    return;
  }

  if (!HEAP_CHECK("decode")) { videoStreamStop(); returnHome(); return; }

  // Mid-frame drain. Nothing reads the socket during decode and blit, so lwIP's
  // receive window fills within a few KB and the server stalls until we come
  // back. That is why prefetch hid the round trip but left the entire transfer on
  // the critical path: ttfb came back as exactly decode + blit + overhead while
  // xfer did not move at all. Emptying the window here lets the server keep
  // sending through the ~110 ms blit below.
  //
  // The home panel tried this and measured no gain - see run 6 in
  // doc/live_video_feed.md - but its transfer was 24 ms of a 115 ms frame. Here
  // it is 160 ms of 342, so the same change has very different stakes.
  {
    const int mid = pollResponse();
    if (mid < 0) {
      USBSerial.println("Video: fetch failed mid-frame, stopping");
      videoStreamStop();
      returnHome();
      return;
    }
    if (mid == 1) frameReady = true;      // next frame already complete
  }

  lv_timer_handler();
  if (!active) return;

  displayFrame(&blitUs);

  if (!HEAP_CHECK("blit")) { videoStreamStop(); returnHome(); return; }

  const uint32_t now = micros();
  sumFrame += (frames == 0) ? (now - startUs) : (now - lastFrameUs);
  lastFrameUs = now;
  sumHttp += httpUs;
  sumTtfb += ttfbUs;
  sumXfer += xferUs;
  sumBytes += jpegLen;
  sumDecode += decodeUs;
  sumBlit += blitUs;
  frames++;

  if (!more) {
    videoStreamStop();
    returnHome();
  }
}
