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
constexpr unsigned long HTTP_TIMEOUT_MS = 15000;
constexpr size_t MAX_JPEG_SIZE = 60000;  // 60 KB

// --- Screen 2 timeout management ---
constexpr unsigned long SCREEN2_LOADING_TIMEOUT = 8000;   // 8 seconds
constexpr unsigned long SCREEN2_DISPLAY_TIMEOUT = 60000;  // 1 minute

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

ImageFetcherConfig cfg{};

}  // namespace

// Forward declarations
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

  // The GFX pipeline expects RGB565 (big endian); set decoder to match
  TJpgDec.setSwapBytes(false);

  // Ensure descriptor is zeroed
  memset(&img_dsc, 0, sizeof(img_dsc));
}

//***************************************************************************************************
void imageFetcherLoop() {
  processHTTPResponse();

  // Handle Screen 2 timeouts
  if (cfg.screen2 && lv_scr_act() == cfg.screen2) {
    // Loading timeout
    if (screen2TimeoutActive && millis() - screenTransitionTime > SCREEN2_LOADING_TIMEOUT) {
      USBSerial.println("Screen 2 timeout - image loading took too long, returning to Screen 1");

      if (httpState != HTTP_IDLE && httpState != HTTP_COMPLETE) {
        httpClient.end();
        if (jpeg_buffer_psram) {
          free(jpeg_buffer_psram);
          jpeg_buffer_psram = nullptr;
        }
        httpState = HTTP_ERROR;  // Will be reset to IDLE when leaving Screen 2
      }

      requestInProgress = false;
      screen2TimeoutActive = false;
      imageDisplayTimeoutActive = false;

      if (cfg.screen1) {
        lv_disp_load_scr(cfg.screen1);
      }
    }

    // Display timeout (after successful load)
    if (imageDisplayTimeoutActive) {
      unsigned long elapsed = millis() - imageDisplayStartTime;
      if (elapsed > SCREEN2_DISPLAY_TIMEOUT) {
        USBSerial.println("Screen 2 display timeout - 1 minute elapsed, returning to Screen 1");

        requestInProgress = false;
        screen2TimeoutActive = false;
        imageDisplayTimeoutActive = false;

        if (cfg.screen1) {
          lv_disp_load_scr(cfg.screen1);
        }
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
  USBSerial.println("=== requestImage() ENTRY ===");

  delay(10);

  if (httpState != HTTP_IDLE) {
    USBSerial.println("HTTP request already in progress, ignoring new request.");
    return false;
  }

  if (WiFi.status() != WL_CONNECTED) {
    USBSerial.println("WiFi not connected, cannot make HTTP request.");
    return false;
  }

  USBSerial.printf("WiFi connected to: %s, RSSI: %d dBm\n", WiFi.SSID().c_str(), WiFi.RSSI());

  lv_refr_now(NULL);

  String url;
  bool isSecureConnection = (WiFi.SSID() == ssid2);

  if (isSecureConnection) {
    USBSerial.println("Cleaning up previous HTTPS client state...");
    httpsClient.stop();
    delay(10);

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

  httpClient.setTimeout(HTTP_TIMEOUT_MS);
  httpClient.setConnectTimeout(HTTP_TIMEOUT_MS);

  httpState = HTTP_REQUESTING;
  USBSerial.println("Sending HTTP GET...");
  int httpCode = httpClient.GET();

  if (httpCode != HTTP_CODE_OK) {
    USBSerial.printf("FATAL: HTTP GET failed with code: %d\n", httpCode);
    httpClient.end();
    httpState = HTTP_ERROR;
    return false;
  }

  int contentLength = httpClient.getSize();
  USBSerial.print("Response received, Content-Length: ");
  USBSerial.println(contentLength);

  if (contentLength <= 0 || contentLength > static_cast<int>(MAX_JPEG_SIZE)) {
    USBSerial.println("Invalid or too large content length");
    httpClient.end();
    httpState = HTTP_ERROR;
    return false;
  }

  if (jpeg_buffer_psram) {
    USBSerial.println("WARNING: jpeg_buffer_psram was not null, freeing old buffer...");
    free(jpeg_buffer_psram);
    jpeg_buffer_psram = nullptr;
    delay(10);
  }

  if (image_buffer_psram) {
    USBSerial.println("WARNING: image_buffer_psram exists, freeing...");
    free(image_buffer_psram);
    image_buffer_psram = nullptr;
    delay(10);
  }

  jpeg_buffer_psram = static_cast<uint8_t*>(ps_malloc(contentLength));
  if (!jpeg_buffer_psram) {
    USBSerial.println("FATAL: Failed to allocate PSRAM for JPEG buffer");
    USBSerial.printf("Requested size: %d bytes\n", contentLength);
    httpClient.end();
    httpState = HTTP_ERROR;
    return false;
  }

  USBSerial.printf("Successfully allocated JPEG buffer: %d bytes at 0x%08X\n", contentLength,
                   reinterpret_cast<uint32_t>(jpeg_buffer_psram));
  printMemoryStats("After JPEG allocation");

  jpeg_buffer_size = contentLength;
  jpeg_bytes_received = 0;
  httpRequestStartTime = millis();
  httpState = HTTP_RECEIVING;

  USBSerial.println("Starting to receive image data...");
  USBSerial.println("=== requestImage() EXIT SUCCESS ===");
  return true;
}

//***************************************************************************************************
static void processHTTPResponse() {
  static bool timeoutMessageShown = false;

  if (httpState == HTTP_IDLE || httpState == HTTP_COMPLETE) {
    timeoutMessageShown = false;
    return;
  }

  if (millis() - httpRequestStartTime > HTTP_TIMEOUT_MS) {
    if (!timeoutMessageShown) {
      USBSerial.println("HTTP request timed out!");
      timeoutMessageShown = true;
    }
    httpClient.end();
    if (jpeg_buffer_psram) {
      free(jpeg_buffer_psram);
      jpeg_buffer_psram = nullptr;
    }
    httpState = HTTP_ERROR;
    return;
  }

  if (httpState == HTTP_RECEIVING) {
    WiFiClient* stream = httpClient.getStreamPtr();

    while (stream->available() && jpeg_bytes_received < jpeg_buffer_size) {
      size_t bytesToRead =
          min(static_cast<size_t>(stream->available()), jpeg_buffer_size - jpeg_bytes_received);
      size_t bytesRead =
          stream->readBytes(jpeg_buffer_psram + jpeg_bytes_received, bytesToRead);
      jpeg_bytes_received += bytesRead;

      if (jpeg_bytes_received % 4096 == 0) {
        lv_timer_handler();
        delay(1);
      }
    }

    if (jpeg_bytes_received >= jpeg_buffer_size) {
      USBSerial.println("Image download complete. Starting decode...");
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

    size_t imageBufferSize = static_cast<size_t>(cfg.screenWidth) * cfg.screenHeight *
                             sizeof(uint16_t);

    image_buffer_psram = static_cast<uint16_t*>(ps_malloc(imageBufferSize));

    if (!image_buffer_psram) {
      USBSerial.println("FATAL: PSRAM allocation failed for decoded image buffer");
      USBSerial.printf("Requested size: %d bytes\n", imageBufferSize);

      free(jpeg_buffer_psram);
      jpeg_buffer_psram = nullptr;
      httpClient.end();
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
      free(image_buffer_psram);
      image_buffer_psram = nullptr;
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
    USBSerial.println("LVGL image source updated.");

    httpState = HTTP_COMPLETE;
    requestInProgress = false;
    screen2TimeoutActive = false;

    imageDisplayTimeoutActive = true;
    imageDisplayStartTime = millis();
    USBSerial.println("Image loaded successfully - starting 1-minute display timeout");
    return;
  }

  if (httpState == HTTP_ERROR) {
    httpState = HTTP_IDLE;
    requestInProgress = false;
    return;
  }
}

//***************************************************************************************************
bool requestLatestImage() {
  if (requestInProgress || httpState != HTTP_IDLE) {
    USBSerial.println("Request already in progress, ignoring request");
    return false;
  }

  lv_obj_t* current_screen = lv_scr_act();
  if (current_screen != cfg.screen1 && current_screen != cfg.screen3 &&
      current_screen != cfg.inclinometerScreen) {
    USBSerial.println("Not on Screen1 or Screen3 or inclinometer, ignoring image request");
    return false;
  }

  USBSerial.println("Initiating latest image request");

  requestInProgress = true;
  if (cfg.screen2) {
    lv_disp_load_scr(cfg.screen2);
  }

  if (requestImage("latest")) {
    return true;
  } else {
    USBSerial.println("Failed to initiate image request");
    requestInProgress = false;
    return false;
  }
}

//***************************************************************************************************
void buttonLatest_event_handler(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_CLICKED) {
    USBSerial.println("Latest button clicked");
    requestLatestImage();
  }
}

//***************************************************************************************************
void buttonNew_event_handler(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_CLICKED) {
    USBSerial.println("New button clicked");

    if (requestInProgress || httpState != HTTP_IDLE) {
      USBSerial.println("Request already in progress, ignoring button press");
      return;
    }

    requestInProgress = true;
    if (cfg.screen2) {
      lv_disp_load_scr(cfg.screen2);
    }

    if (requestImage("new")) {
      USBSerial.println("New image request initiated, transitioning to Screen 2");
    } else {
      USBSerial.println("Failed to initiate new image request");
      requestInProgress = false;
    }
  }
}

//***************************************************************************************************
void buttonBack_event_handler(lv_event_t* e) {
  lv_event_code_t code = lv_event_get_code(e);

  if (code == LV_EVENT_CLICKED) {
    USBSerial.println("Back button clicked");

    if (requestInProgress || httpState != HTTP_IDLE) {
      USBSerial.println("Request already in progress, ignoring button press");
      return;
    }

    requestInProgress = true;
    if (cfg.screen2) {
      lv_disp_load_scr(cfg.screen2);
    }

    if (requestImage("back")) {
      USBSerial.println("Back image request initiated, transitioning to Screen 2");
    } else {
      USBSerial.println("Failed to initiate back image request");
      requestInProgress = false;
    }
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
      USBSerial.println(
          "Race condition detected: Download finished before screen load. Forcing display.");
      if (cfg.imgScreen2Background) {
        lv_obj_set_style_opa(cfg.imgScreen2Background, LV_OPA_COVER, LV_PART_MAIN);
      }
      lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);
    }

    screenTransitionTime = millis();
    screen2TimeoutActive = true;

    if (httpState != HTTP_COMPLETE) {
      imageDisplayTimeoutActive = false;
    }

    USBSerial.println("Screen 2 timeout started (8 seconds)");

    if (httpState == HTTP_REQUESTING || httpState == HTTP_RECEIVING) {
      USBSerial.println("Request already in progress from button handler, waiting for completion");
    } else if (httpState == HTTP_IDLE && !requestInProgress) {
      USBSerial.println("WARNING: Screen 2 loaded but no request was initiated");
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

