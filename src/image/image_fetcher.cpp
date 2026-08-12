#include "image_fetcher.h"

#include <HTTPClient.h>
#include <TJpg_Decoder.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include "HWCDC.h"
#if defined(__has_include) && __has_include("secrets_private.h")
#include "secrets_private.h"
#else
#include "secrets.h"
#endif
#include "ui.h"
extern lv_obj_t* ui_previous_screen;

namespace {

enum ImageRequestState {
  HTTP_IDLE,
  HTTP_REQUESTING,
  HTTP_RECEIVING,
  HTTP_DECODING,
  HTTP_COMPLETE,
  HTTP_ERROR
};

// --- HTTP/S configuration ---
// These four timeouts must stay consistent with SCREEN2_LOADING_TIMEOUT below, because
// GET() blocks the whole sketch: while it runs, neither HTTP_TIMEOUT_MS nor the Screen-2
// deadline can be evaluated and LVGL is frozen. Worst-case blocking inside a single GET()
// is CONNECT + HANDSHAKE + SOCKET (5 + 5 + 8 = 18 s), which must remain below the 20 s
// Screen-2 deadline so the abort path can actually run.
constexpr unsigned long HTTP_TIMEOUT_MS = 15000;
constexpr unsigned long HTTP_SOCKET_TIMEOUT_MS = 8000;  // Socket-level timeout for GET() blocking
constexpr unsigned long HTTP_CONNECT_TIMEOUT_MS = 5000;  // TCP connect
constexpr unsigned int HTTP_HANDSHAKE_TIMEOUT_S = 5;     // TLS handshake (setter takes seconds)
// 128 KB. PSRAM is 8 MB, so the old 60 KB cap bought nothing and silently rejected any
// larger photo the image server happened to produce.
constexpr size_t MAX_JPEG_SIZE = 128000;

// --- Screen 2 timeout management ---
// Armed at the button press, so this single budget has to cover DNS + TCP connect + TLS
// handshake + server think time + body download + JPEG decode. It must exceed
// HTTP_TIMEOUT_MS (15 s) plus decode time, otherwise a transfer that the HTTP layer still
// considers healthy gets killed by the UI layer — that was the original 8 s bug, where a
// slow HTTPS-over-cellular connect consumed the entire budget before the body arrived.
constexpr unsigned long SCREEN2_LOADING_TIMEOUT = 20000;  // 20 seconds
constexpr unsigned long SCREEN2_DISPLAY_TIMEOUT = 60000;  // 1 minute

// --- Notification echo suppression ---
// The image server publishes "latest" on the MQTT image topic once it has served /esp32/new,
// which is how the *other* companion unit learns to refresh. The unit that pressed the button
// receives that notification too, for the image it just downloaded. MQTT is starved for the
// whole fetch (the blocking GET() and the receive loop never call mqttClient.loop()), so the
// echo is delivered one loop iteration after the photo is painted: the user sees the new image
// for a single frame, then the green loading screen returns for a second, redundant download of
// byte-identical data. Ignore a notification that lands within this window of a successful load.
// Button presses are never suppressed — only requestLatestImage(fromNotification = true).
constexpr unsigned long NOTIFICATION_ECHO_WINDOW_MS = 10000;  // 10 seconds

// State
ImageRequestState httpState = HTTP_IDLE;
HTTPClient httpClient;
WiFiClientSecure httpsClient;

uint16_t* image_buffer_psram = nullptr;
lv_img_dsc_t img_dsc{};
uint8_t* jpeg_buffer_psram = nullptr;
size_t jpeg_buffer_size = 0;
size_t jpeg_bytes_received = 0;
unsigned long httpRequestStartTime = 0;
bool requestInProgress = false;

unsigned long screenTransitionTime = 0;
bool screen2TimeoutActive = false;
bool imageDisplayTimeoutActive = false;
unsigned long imageDisplayStartTime = 0;

// Timestamp of the last image that reached the screen. 0 means "none since boot", which
// matters because without the sentinel every millis() early in the boot would fall inside
// NOTIFICATION_ECHO_WINDOW_MS and swallow a genuine notification. Deliberately never
// cleared: the echo can arrive after the user has navigated away from Screen 2, and
// honouring it there is exactly the yank-back-to-loading-screen we are suppressing.
unsigned long lastImageLoadedTime = 0;

ImageFetcherConfig cfg{};

// Asynchronous request tracking
const char* pendingEndpoint = nullptr;

}  // namespace

// Forward declarations
static void cleanupImageRequest();
static void returnToPreviousScreen(const char* reason);
static void prepareForRequest();
static bool requestImage(const char* endpoint_type);
static void processHTTPResponse();
static bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap);

// External dependencies
extern HWCDC USBSerial;
extern void printMemoryStats(const char* location);

//***************************************************************************************************
void imageFetcherInit(const ImageFetcherConfig& config) {
  cfg = config;
  httpState = HTTP_IDLE;
  requestInProgress = false;
  screen2TimeoutActive = false;
  imageDisplayTimeoutActive = false;
  lastImageLoadedTime = 0;
  pendingEndpoint = nullptr;

  // The GFX pipeline expects RGB565 (big endian); set decoder to match
  TJpgDec.setSwapBytes(false);

  // Ensure descriptor is zeroed
  memset(&img_dsc, 0, sizeof(img_dsc));
}

//***************************************************************************************************
static void cleanupImageRequest() {
  USBSerial.println("Cleaning up image fetcher state...");

  // 1. Stop HTTP
  httpClient.end();
  httpsClient.stop();

  // 2. Hide image if it exists to avoid LVGL accessing freed memory
  if (cfg.imgScreen2Background) {
    lv_obj_set_style_opa(cfg.imgScreen2Background, LV_OPA_TRANSP, LV_PART_MAIN);
  }

  // 3. Free buffers
  if (jpeg_buffer_psram) {
    free(jpeg_buffer_psram);
    jpeg_buffer_psram = nullptr;
  }
  if (image_buffer_psram) {
    free(image_buffer_psram);
    image_buffer_psram = nullptr;
  }

  // 4. Reset descriptors and states
  memset(&img_dsc, 0, sizeof(img_dsc));
  httpState = HTTP_IDLE;
  // Note: we don't reset requestInProgress here to allow the async transition to work.
  // Note: screen2TimeoutActive / imageDisplayTimeoutActive are intentionally NOT cleared
  // here — clearing them would disarm the Screen-2 auto-return fallback after an HTTP
  // error and strand the user on the green "Getting image" screen. Those flags are
  // managed by the screen-2 timeout handler and the screen UNLOAD event.
}

//***************************************************************************************************
// Leave the loading screen immediately instead of letting the user stare at the green
// "Getting image" screen until SCREEN2_LOADING_TIMEOUT expires. Callers must have already
// released any buffers; this only clears the timeout flags and navigates.
static void returnToPreviousScreen(const char* reason) {
  USBSerial.printf("Image request ended (%s) - returning to previous screen\n", reason);

  requestInProgress = false;
  screen2TimeoutActive = false;
  imageDisplayTimeoutActive = false;

  // If the user already navigated away (or we were never on Screen 2), don't yank them
  // back to ui_previous_screen.
  if (cfg.screen2 && lv_scr_act() != cfg.screen2) return;

  if (ui_previous_screen) {
    lv_disp_load_scr(ui_previous_screen);
  } else if (cfg.screen1) {
    lv_disp_load_scr(cfg.screen1);
  }
}

//***************************************************************************************************
bool imageFetcherIsBusy() {
  return pendingEndpoint != nullptr || requestInProgress || httpState == HTTP_REQUESTING ||
         httpState == HTTP_RECEIVING || httpState == HTTP_DECODING;
}

//***************************************************************************************************
static void prepareForRequest() {
  USBSerial.println("Preparing UI for new image request...");
  
  cleanupImageRequest();

  // Save the current screen if it's not the image screen itself (Screen 2)
  lv_obj_t* current = lv_scr_act();
  if (current != cfg.screen2 && cfg.screen2) {
      ui_previous_screen = current;
      lv_disp_load_scr(cfg.screen2);
  } else if (current == cfg.screen2 && ui_previous_screen == NULL && cfg.screen1) {
       // Ideally we shouldn't be here without a previous screen, but safety fallback:
       ui_previous_screen = cfg.screen1;
  }


  // Force rotation to default (90 degrees) for the UI/Loading state
  lv_disp_t* disp = lv_disp_get_default();
  if (disp) {
    lv_disp_set_rotation(disp, LV_DISP_ROT_90);
  }

  // Reset loading timeout state
  screenTransitionTime = millis();
  screen2TimeoutActive = true;
  imageDisplayTimeoutActive = false;
  requestInProgress = true;

  // Force an immediate UI refresh so the rotation and "Loading" state are visible 
  // BEFORE we potentially block on the network request in the next loop.
  lv_refr_now(NULL);
}

//***************************************************************************************************
void imageFetcherLoop() {
  // Handle asynchronous request triggering
  if (pendingEndpoint != nullptr) {
    const char* endpoint = pendingEndpoint;
    pendingEndpoint = nullptr; // Clear it immediately
    if (!requestImage(endpoint)) {
      USBSerial.println("Async request failed to initiate.");
      // Most failure paths in requestImage() set HTTP_ERROR and would be picked up by the
      // fail-fast handling in processHTTPResponse(), but the "WiFi not connected" path
      // leaves the state IDLE. Handle the return here so every failure to even start a
      // request leaves the loading screen immediately instead of waiting out the full
      // SCREEN2_LOADING_TIMEOUT.
      httpState = HTTP_IDLE;
      returnToPreviousScreen("request failed to start");
    }
    return;
  }

  processHTTPResponse();

  // Handle Screen 2 timeouts
  if (cfg.screen2 && lv_scr_act() == cfg.screen2) {
    // Loading timeout.
    // HTTP_DECODING is deliberately exempt: reaching that state means the complete JPEG is
    // already in PSRAM and all that remains is a bounded synchronous decode (~0.5 s), which
    // processHTTPResponse() runs on the very next iteration. Cancelling here discarded a
    // download that had fully succeeded — the finish-line race that lost images even when
    // the network had done its job. Decode always resolves to HTTP_COMPLETE (which clears
    // screen2TimeoutActive) or HTTP_ERROR (which fails fast), so this cannot hang.
    if (screen2TimeoutActive && httpState != HTTP_DECODING &&
        millis() - screenTransitionTime > SCREEN2_LOADING_TIMEOUT) {
      USBSerial.println("Screen 2 timeout - image loading took too long, returning to Screen 1");

      if (httpState != HTTP_IDLE && httpState != HTTP_COMPLETE) {
        cleanupImageRequest();
        httpState = HTTP_ERROR;
      }

      // Flags are cleared inside returnToPreviousScreen().
      returnToPreviousScreen("loading timeout");
    }

    // Display timeout (after successful load)
    if (imageDisplayTimeoutActive) {
      unsigned long elapsed = millis() - imageDisplayStartTime;
      if (elapsed > SCREEN2_DISPLAY_TIMEOUT) {
        USBSerial.println("Screen 2 display timeout - 1 minute elapsed, returning to Screen 1");
        returnToPreviousScreen("display timeout");
      }
    }
  }
}

//***************************************************************************************************
static bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  if (!image_buffer_psram || y >= cfg.screenHeight || x >= cfg.screenWidth) return 0;

  for (uint16_t row = 0; row < h; row++) {
    if ((y + row) >= cfg.screenHeight) break;
    for (uint16_t col = 0; col < w; col++) {
      if ((x + col) >= cfg.screenWidth) break;
      uint32_t dstIndex = static_cast<uint32_t>(y + row) * cfg.screenWidth + (x + col);
      image_buffer_psram[dstIndex] = bitmap[static_cast<uint32_t>(row) * w + col];
    }
  }
  return 1;
}

//***************************************************************************************************
static bool requestImage(const char* endpoint_type) {
  USBSerial.printf("=== requestImage('%s') START ===\n", endpoint_type);

  if (WiFi.status() != WL_CONNECTED) {
    USBSerial.println("WiFi not connected, cannot make HTTP request.");
    return false;
  }

  String url;
  bool isSecureConnection = (WiFi.SSID() == ssid2);

  if (isSecureConnection) {
    url = String(IMAGE_SERVER_REMOTE) + String(endpoint_type) + "?token=" + String(API_TOKEN);
    USBSerial.println("Initiating HTTPS GET: " + url);
    httpsClient.setCACert(remote_server_ca_cert);
    bool beginResult = httpClient.begin(httpsClient, url);
    if (!beginResult) {
      USBSerial.println("FATAL: httpClient.begin() failed for HTTPS!");
      httpState = HTTP_ERROR;
      return false;
    }
  } else {
    url = String(IMAGE_SERVER_BASE) + String(endpoint_type) + "?token=" + String(API_TOKEN);
    USBSerial.println("Initiating HTTP GET: " + url);
    bool beginResult = httpClient.begin(url);
    if (!beginResult) {
      USBSerial.println("FATAL: httpClient.begin() failed for HTTP!");
      httpState = HTTP_ERROR;
      return false;
    }
  }

  httpClient.setTimeout(HTTP_SOCKET_TIMEOUT_MS);
  httpClient.setConnectTimeout(HTTP_CONNECT_TIMEOUT_MS);
  // Bound the TLS handshake so a stalled HTTPS endpoint can't block GET() for the
  // 120 s default. Setter takes seconds.
  httpsClient.setHandshakeTimeout(HTTP_HANDSHAKE_TIMEOUT_S);

  httpState = HTTP_REQUESTING;
  httpRequestStartTime = millis();  // Set BEFORE blocking call for timeout tracking
  USBSerial.println("Sending HTTP GET...");
  int httpCode = httpClient.GET();
  // Connect + TLS handshake + server think time, all of it blocking. This is the phase
  // that used to swallow the whole loading budget on the iPhone hotspot, so log it.
  unsigned long connectMs = millis() - httpRequestStartTime;

  if (httpCode != HTTP_CODE_OK) {
    USBSerial.printf("FATAL: HTTP GET failed with code: %d (after %lu ms)\n", httpCode, connectMs);
    httpClient.end();
    httpState = HTTP_ERROR;
    return false;
  }

  int contentLength = httpClient.getSize();
  USBSerial.printf("Response received in %lu ms, Content-Length: %d\n", connectMs, contentLength);

  if (contentLength <= 0) {
    // getSize() returns -1 when the server omits Content-Length (chunked transfer).
    // The receive loop is sized from this value, so we cannot proceed.
    USBSerial.printf("FATAL: no usable Content-Length (%d) - chunked responses unsupported\n",
                     contentLength);
    httpClient.end();
    httpState = HTTP_ERROR;
    return false;
  }

  if (contentLength > static_cast<int>(MAX_JPEG_SIZE)) {
    USBSerial.printf("FATAL: image too large: %d bytes (limit %u)\n", contentLength,
                     static_cast<unsigned>(MAX_JPEG_SIZE));
    httpClient.end();
    httpState = HTTP_ERROR;
    return false;
  }

  jpeg_buffer_psram = static_cast<uint8_t*>(ps_malloc(contentLength));
  if (!jpeg_buffer_psram) {
    USBSerial.println("FATAL: Failed to allocate PSRAM for JPEG buffer");
    httpClient.end();
    httpState = HTTP_ERROR;
    return false;
  }

  jpeg_buffer_size = contentLength;
  jpeg_bytes_received = 0;
  httpState = HTTP_RECEIVING;

  USBSerial.println("Starting to receive image data...");
  return true;
}

//***************************************************************************************************
static void processHTTPResponse() {
  static bool timeoutMessageShown = false;

  if (httpState == HTTP_IDLE || httpState == HTTP_COMPLETE) {
    timeoutMessageShown = false;
    return;
  }

  // HTTP_DECODING is exempt for the same reason as the Screen-2 deadline: the transfer is
  // already finished, so a network timeout no longer applies. Without this, a download that
  // completed at 14.9 s would be freed here before the decode branch below ever ran.
  if (httpState != HTTP_DECODING && millis() - httpRequestStartTime > HTTP_TIMEOUT_MS) {
    if (!timeoutMessageShown) {
      USBSerial.println("HTTP request timed out!");
      timeoutMessageShown = true;
    }
    cleanupImageRequest();
    // Leave httpState = HTTP_IDLE (set by cleanupImageRequest). Forcing it back to
    // HTTP_ERROR here would re-enter this block on every loop iteration because
    // httpRequestStartTime is never advanced — that produced an endless cleanup spam.
    //
    // Because this path stays IDLE rather than ERROR, the fail-fast handling in the
    // HTTP_ERROR branch below never sees a stalled transfer. Return the user here
    // instead, otherwise the transfer gives up at 15 s but the green screen lingers
    // until the 20 s Screen-2 deadline.
    returnToPreviousScreen("HTTP timeout");
    return;
  }

  if (httpState == HTTP_RECEIVING) {
    WiFiClient* stream = httpClient.getStreamPtr();

    while (stream->available() && jpeg_bytes_received < jpeg_buffer_size) {
      size_t bytesToRead =
          min(static_cast<size_t>(stream->available()), jpeg_buffer_size - jpeg_bytes_received);
      size_t bytesRead = stream->readBytes(jpeg_buffer_psram + jpeg_bytes_received, bytesToRead);
      jpeg_bytes_received += bytesRead;

      if (jpeg_bytes_received % 4096 == 0) {
        lv_timer_handler();
        delay(1);
      }
    }

    if (jpeg_bytes_received >= jpeg_buffer_size) {
      USBSerial.printf("Image download complete (%u bytes, %lu ms since button press). "
                       "Starting decode...\n",
                       static_cast<unsigned>(jpeg_bytes_received), millis() - screenTransitionTime);
      httpClient.end();
      httpState = HTTP_DECODING;
    }
    return;
  }

  if (httpState == HTTP_DECODING) {
    if (image_buffer_psram) {
      free(image_buffer_psram);
      image_buffer_psram = nullptr;
    }

    size_t imageBufferSize =
        static_cast<size_t>(cfg.screenWidth) * cfg.screenHeight * sizeof(uint16_t);

    image_buffer_psram = static_cast<uint16_t*>(ps_malloc(imageBufferSize));

    if (!image_buffer_psram) {
      USBSerial.println("FATAL: PSRAM allocation failed for decoded image buffer");
      cleanupImageRequest();
      httpState = HTTP_ERROR;
      return;
    }

    TJpgDec.setJpgScale(1);
    TJpgDec.setCallback(tft_output);

    uint8_t result = TJpgDec.drawJpg(0, 0, jpeg_buffer_psram, jpeg_buffer_size);

    free(jpeg_buffer_psram);
    jpeg_buffer_psram = nullptr;

    if (result != 0) {
      USBSerial.println("TJpgDec error code: " + String(result));
      if (image_buffer_psram) { free(image_buffer_psram); image_buffer_psram = nullptr; }
      httpState = HTTP_ERROR;
      return;
    }

    USBSerial.println("JPEG decoded successfully into PSRAM.");

    lv_disp_t* disp = lv_disp_get_default();
    lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);

    img_dsc.header.always_zero = 0;
    img_dsc.header.w = cfg.screenWidth;
    img_dsc.header.h = cfg.screenHeight;
    img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
    img_dsc.data_size = cfg.screenWidth * cfg.screenHeight * LV_COLOR_DEPTH / 8;
    img_dsc.data = reinterpret_cast<const uint8_t*>(image_buffer_psram);

    if (cfg.imgScreen2Background) {
      lv_img_set_src(cfg.imgScreen2Background, &img_dsc);
      lv_obj_set_style_opa(cfg.imgScreen2Background, LV_OPA_COVER, LV_PART_MAIN);
    }
    USBSerial.printf("LVGL image source updated. Total %lu ms from button press (budget %lu ms).\n",
                     millis() - screenTransitionTime, SCREEN2_LOADING_TIMEOUT);

    httpState = HTTP_COMPLETE;
    requestInProgress = false;
    screen2TimeoutActive = false;

    imageDisplayTimeoutActive = true;
    imageDisplayStartTime = millis();
    lastImageLoadedTime = imageDisplayStartTime;
    return;
  }

  if (httpState == HTTP_ERROR) {
    httpState = HTTP_IDLE;
    requestInProgress = false;
    // Every path that sets HTTP_ERROR is definitive — bad HTTP status, unusable
    // Content-Length, PSRAM exhaustion, decoder failure. Nothing further is coming, so
    // don't make the user wait out the remaining Screen-2 deadline on a green screen.
    returnToPreviousScreen("HTTP error");
    return;
  }
}

//***************************************************************************************************
bool requestLatestImage(bool fromNotification) {
  // See NOTIFICATION_ECHO_WINDOW_MS. Only the MQTT path passes fromNotification = true; a
  // button press always requests, however recently the last image landed.
  if (fromNotification && lastImageLoadedTime != 0 &&
      millis() - lastImageLoadedTime < NOTIFICATION_ECHO_WINDOW_MS) {
    USBSerial.printf(
        "Ignoring 'latest' notification %lu ms after last image load (echo window %lu ms)\n",
        millis() - lastImageLoadedTime, NOTIFICATION_ECHO_WINDOW_MS);
    return false;
  }

  lv_obj_t* current_screen = lv_scr_act();
  if (current_screen != cfg.screen1 && current_screen != cfg.screen2 &&
      current_screen != cfg.screen3 && current_screen != cfg.inclinometerScreen) {
    USBSerial.println("On unsupported screen, ignoring image request");
    return false;
  }

  USBSerial.println("Initiating async latest image request...");
  prepareForRequest();
  pendingEndpoint = "latest";
  return true;
}

//***************************************************************************************************
void buttonLatest_event_handler(lv_event_t* e) {
  if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
    USBSerial.println("Latest button clicked");
    requestLatestImage();
  }
}

//***************************************************************************************************
void buttonNew_event_handler(lv_event_t* e) {
  if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
    USBSerial.println("New button clicked, initiating async request...");
    prepareForRequest();
    pendingEndpoint = "new";
  }
}

//***************************************************************************************************
void buttonBack_event_handler(lv_event_t* e) {
  if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
    USBSerial.println("Back button clicked, initiating async request...");
    prepareForRequest();
    pendingEndpoint = "back";
  }
}

//***************************************************************************************************
void screen2_event_handler(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_disp_t* disp = lv_disp_get_default();

  if (code == LV_EVENT_SCREEN_LOADED) {
    USBSerial.println("Screen 2 Loaded.");
    if (httpState != HTTP_COMPLETE) {
      if (cfg.imgScreen2Background) {
        lv_obj_set_style_opa(cfg.imgScreen2Background, LV_OPA_TRANSP, LV_PART_MAIN);
      }
    } else {
      if (cfg.imgScreen2Background) {
        lv_obj_set_style_opa(cfg.imgScreen2Background, LV_OPA_COVER, LV_PART_MAIN);
      }
      lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);
    }
    // Only reset timeout if no request is in progress
    // (prevents extending timeout when screen loads after blocking HTTP call)
    if (!requestInProgress) {
      screenTransitionTime = millis();
      screen2TimeoutActive = true;
    }
    if (httpState != HTTP_COMPLETE) {
      imageDisplayTimeoutActive = false;
    }
  } else if (code == LV_EVENT_SCREEN_UNLOAD_START) {
    USBSerial.println("Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.");
    screen2TimeoutActive = false;
    imageDisplayTimeoutActive = false;
    if (cfg.imgScreen2Background) {
      lv_obj_set_style_opa(cfg.imgScreen2Background, LV_OPA_TRANSP, LV_PART_MAIN);
    }
    if (image_buffer_psram != nullptr) {
      free(image_buffer_psram);
      image_buffer_psram = nullptr;
    }
    if (jpeg_buffer_psram != nullptr) {
      free(jpeg_buffer_psram);
      jpeg_buffer_psram = nullptr;
    }
    memset(&img_dsc, 0, sizeof(lv_img_dsc_t));
    httpState = HTTP_IDLE;
    requestInProgress = false;
    lv_disp_set_rotation(disp, LV_DISP_ROT_90);
  }
}
