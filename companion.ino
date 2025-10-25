#include "secrets.h"
#include <Wire.h>
#include "SensorQMI8658.hpp"
#include <Arduino.h>
#include "pin_config.h"
#include <lvgl.h>
#include "Arduino_GFX_Library.h"
#include "ui.h"
#include "Arduino_DriveBus_Library.h"
#include <ESP_IOExpander_Library.h>
#include "HWCDC.h"
#include "XPowersLib.h"
#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

// --- HTTP IMAGE INTEGRATION
#include <HTTPClient.h>
#include <TJpg_Decoder.h>

// QMI8658 Register Addresses
#define QMI8658_CTRL2       0x03
#define QMI8658_CTRL7       0x08
#define QMI8658_CTRL9       0x0A
#define QMI8658_CAL1_L      0x0B
#define QMI8658_CAL1_H      0x0C
#define QMI8658_STATUS1     0x2F

// Forward-declare the USBSerial object so our class can see it.
extern HWCDC USBSerial;

// Helper class to access and add low-level register functions
class QMI8658_WoM_Handler : public SensorQMI8658 {
private:
  // We add our own I2C communication functions because we cannot
  // access the private members of the base class.
  void writeRegister(uint8_t reg, uint8_t value) {
    // FINAL FIX: We use the known I2C address constant directly, since
    // we cannot get the address from the object itself.
    Wire.beginTransmission(QMI8658_L_SLAVE_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
  }

  uint8_t readRegister(uint8_t reg) {
    // Use the known I2C address constant here as well.
    Wire.beginTransmission(QMI8658_L_SLAVE_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission(false); // Send repeated start
    Wire.requestFrom((uint8_t)QMI8658_L_SLAVE_ADDRESS, (uint8_t)1);
    if (Wire.available()) {
      return Wire.read();
    }
    return 0; // Return 0 on error
  }

public:
  void configureWoM() {
    USBSerial.println("Configuring Wake on Motion (WoM)...");
    // These calls refer to the functions we just added above.
    writeRegister(QMI8658_CTRL7, 0x00);
    delay(50);
    uint8_t ctrl2_value = 0b00011101; // 4g range, 21Hz ODR
    writeRegister(QMI8658_CTRL2, ctrl2_value);
    delay(10);
    writeRegister(QMI8658_CAL1_L, 50); // ~50mg threshold. More sensitive: 20, less: 100
    delay(10);
    uint8_t cal1_h_value = 0b01000000; // Use INT2, initial value 0
    writeRegister(QMI8658_CAL1_H, cal1_h_value);
    delay(10);
    writeRegister(QMI8658_CTRL9, 0x08); // Execute WoM command
    delay(50);
    writeRegister(QMI8658_CTRL7, 0x01); // Enable accelerometer
    delay(50);
    USBSerial.println("WoM configured and active. Waiting for motion...");
  }

  uint8_t readStatus1() {
    return readRegister(QMI8658_STATUS1);
  }

  void disableIMU() {
    writeRegister(QMI8658_CTRL7, 0x00);
  }
};

// Create the global object immediately after defining the class
QMI8658_WoM_Handler qmi;

// =================  CONFIGURATION =================
// Change this value to 1 or 2 to set the connection priority.
// 1: Tries ssid1 first, then falls back to ssid2.
// 2: Tries ssid2 first, then falls back to ssid1.
#define WIFI_PRIORITY 1

// --- WiFi Retry Configuration ---
#define WIFI_RETRY_DELAY_MS 45000  // 45 seconds between retry attempts when on USB power
// =================================================

// --- HTTP/S IMAGE INTEGRATION --- Configuration
// Image server URLs are now defined in secrets.h
#define HTTP_TIMEOUT_MS 10000  // 6000
#define MAX_JPEG_SIZE 60000  // 60KB should be plenty for 368x448 JPEG
// MQTT Topics
#define MQTT_IMAGE_TOPIC "esp32image"
// =================================================
const char HILO_POWER[] = "ha/hilo_meter_power";
const char HILO_ENERGY[] = "hilo_energie";
const char MOTION_TOPIC[] = "companion/motion";

// Generic pointers that will be assigned based on WIFI_PRIORITY
const char* primarySsid;
const char* primaryPassword;
int primaryNetworkNum;

const char* secondarySsid;
const char* secondaryPassword;
int secondaryNetworkNum;

// Certificate for secure MQTT is in secrets.h

// Client instances for MQTT and HTTP/S
WiFiClient espClient;
WiFiClientSecure secureClient; // For secure MQTT
PubSubClient mqttClient; // We'll assign the appropriate client in connectToWiFi

// --- Screen 2 Timeout Management ---
unsigned long screenTransitionTime = 0;
const unsigned long SCREEN2_LOADING_TIMEOUT = 8000;  // 8 seconds timeout for image loading
const unsigned long SCREEN2_DISPLAY_TIMEOUT = 60000;  // 1 minute (60 seconds) timeout for viewing image
bool screen2TimeoutActive = false;
bool imageDisplayTimeoutActive = false;  // Track if we're in the 5-minute viewing phase
unsigned long imageDisplayStartTime = 0;  // When the image finished loading

// --- MQTT Reconnection Management ---
unsigned long lastMqttAttempt = 0;
const unsigned long MQTT_RECONNECT_INTERVAL = 15000;  // 15 seconds between reconnection attempts
bool mqttConnection = false;
bool mqttSuccess = false;                 //MQTT succeeded once at start to keep reconnecting only if successful

// --- Global Objects ---
HWCDC USBSerial;
XPowersAXP2101 pmic;
ESP_IOExpander *expander = NULL;

// --- Global State Variables (shared between functions) ---
bool adc_switch = false;
String batteryPercent = "";
float batteryVoltage = 0.0;
bool batteryConnected = false;
bool vbusPresent = false;
const unsigned long INACTIVITY_TIMEOUT = 60000;   // Touch user inactivity for going to sleep
const unsigned long MOTION_TIMEOUT = 30000;      // Time to consider the device stationary after motion stops and send MQTT
unsigned long lastActivityTime = 0;
bool shutdownRequested = false; 
bool g_isCurrentlyMoving = false;

unsigned long lastBatteryUpdate = 0;
unsigned long lastConnectionUpdate = 0;
unsigned long lastMotionTXTime = 0;

// --- USB Power Transition Tracking ---
bool allowSleep = false;
bool usbWasEverPresent = false;           // Track if USB was present during this session
unsigned long usbDisconnectedTime = 0;    // Timestamp when USB was disconnected
const unsigned long USB_GRACE_PERIOD = 30000;        // 30 seconds minimum after USB loss
const unsigned long MAX_RUNTIME_AFTER_USB_LOSS = 180000;  // 3 minutes maximum safety limit

// --- POINTERS for manual initialization to prevent race conditions ---
std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus = nullptr;
std::unique_ptr<Arduino_IIC> FT3168 = nullptr;

#define _EXAMPLE_CHIP_CLASS(name, ...) ESP_IOExpander_##name(__VA_ARGS__)
#define EXAMPLE_CHIP_CLASS(name, ...) _EXAMPLE_CHIP_CLASS(name, ##__VA_ARGS__)

#define LVGL_TICK_PERIOD_MS 2
static const uint16_t screenWidth = 368;
static const uint16_t screenHeight = 448;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * screenHeight / 10];

Arduino_DataBus *bus = new Arduino_ESP32QSPI(
  LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);

Arduino_GFX *gfx = new Arduino_SH8601(bus, -1 /* RST */,
                                      0 /* rotation */, false /* IPS */, LCD_WIDTH, LCD_HEIGHT);


// --- HTTP IMAGE INTEGRATION --- Global variables for image display and HTTP state machine
uint16_t* image_buffer_psram = nullptr;  // Pointer to the buffer in PSRAM holding the decoded image
static lv_img_dsc_t img_dsc; // An LVGL image descriptor to wrap the PSRAM buffer

// HTTP State Machine
enum ImageRequestState {
  HTTP_IDLE,
  HTTP_REQUESTING,
  HTTP_RECEIVING,
  HTTP_DECODING,
  HTTP_COMPLETE,
  HTTP_ERROR
};

ImageRequestState httpState = HTTP_IDLE;
HTTPClient httpClient;
WiFiClientSecure httpsClient; // ADDED: Client for handling HTTPS requests
uint8_t* jpeg_buffer_psram = nullptr;
size_t jpeg_buffer_size = 0;
size_t jpeg_bytes_received = 0;
unsigned long httpRequestStartTime = 0;
bool requestInProgress = false;  // Flag to prevent concurrent requests

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char *buf) {
  Serial.printf(buf);
  Serial.flush();
}
#endif


//***************************************************************************************************
void printMemoryStats(const char* location) {
  USBSerial.printf("[MEM] %s - Free Heap: %d, Free PSRAM: %d, Min Free Heap: %d\n", 
                   location, 
                   ESP.getFreeHeap(), 
                   ESP.getFreePsram(),
                   ESP.getMinFreeHeap());
}


//***************************************************************************************************
// --- HTTP IMAGE INTEGRATION --- TJpg_Decoder callback to write decoded pixels into our PSRAM buffer
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  if (!image_buffer_psram || y >= screenHeight || x >= screenWidth) return 0;

  // Copy the decoded bitmap block to the correct location in the full-screen PSRAM buffer
  for (uint16_t row = 0; row < h; row++) {
    if ((y + row) >= screenHeight) break;
    for (uint16_t col = 0; col < w; col++) {
      if ((x + col) >= screenWidth) break;
      uint32_t dstIndex = (uint32_t)(y + row) * screenWidth + (x + col);
      image_buffer_psram[dstIndex] = bitmap[(uint32_t)row * w + col];
    }
  }
  return 1; // Continue decoding
}


//***************************************************************************************************
// This function combines the original image request logic with the now-working
// secure connection method.
bool requestImage(const char* endpoint_type) {
    USBSerial.println("=== requestImage() ENTRY ===");
    printMemoryStats("Entry state");

    // A small delay can help the system stabilize memory before a large allocation
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

    // --- DYNAMIC URL AND CLIENT CONFIGURATION ---
    String url;
    // Check if we are connected to the remote network (ssid2)
    bool isSecureConnection = (WiFi.SSID() == ssid2);

    if (isSecureConnection) {
        // === HTTPS Connection for remote access (ssid2) ===

        USBSerial.println("Cleaning up previous HTTPS client state...");
        httpsClient.stop();  // Close any existing connection
        delay(10);           // Brief delay for cleanup

        url = String(IMAGE_SERVER_REMOTE) + String(endpoint_type) + "?token=" + String(API_TOKEN);
        USBSerial.println("Initiating HTTPS GET: " + url);

        printMemoryStats("Before HTTPS begin");
        
        // Configure the secure client with the now-verified remote server's CA certificate
        httpsClient.setCACert(remote_server_ca_cert);

        // Begin HTTPS connection using the secure client
        bool beginResult = httpClient.begin(httpsClient, url);
        
        if (!beginResult) {
            USBSerial.println("FATAL: httpClient.begin() failed for HTTPS!");
            printMemoryStats("HTTPS begin failure");
            httpState = HTTP_ERROR;
            return false;
        }
        USBSerial.println("HTTPS begin successful");

    } else {
        // === HTTP Connection for local access (ssid1) ===
        url = String(IMAGE_SERVER_BASE) + String(endpoint_type);
        USBSerial.println("Initiating HTTP GET: " + url);

        // ADD THIS:
        printMemoryStats("Before HTTP begin");
        
        // Begin standard HTTP connection
        bool beginResult = httpClient.begin(url);
        
        // ADD THIS:
        if (!beginResult) {
            USBSerial.println("FATAL: httpClient.begin() failed for HTTP!");
            printMemoryStats("HTTP begin failure");
            httpState = HTTP_ERROR;
            return false;
        }
        USBSerial.println("HTTP begin successful");
    }
    // --- END OF DYNAMIC CONFIGURATION ---


    // Set increased timeouts for cellular network stability
    httpClient.setConnectTimeout(HTTP_TIMEOUT_MS);
    httpClient.setTimeout(HTTP_TIMEOUT_MS);
    
    USBSerial.println("DEBUG: Starting httpClient.GET()...");
    int httpCode = httpClient.GET();
    USBSerial.printf("DEBUG: httpClient.GET() finished with code: %d\n", httpCode);
    
    if (httpCode != HTTP_CODE_OK) {
        USBSerial.printf("FATAL: HTTP GET failed with code: %d\n", httpCode);  // ENHANCED
        printMemoryStats("HTTP GET failure");                                   // NEW
        httpClient.end();
        httpState = HTTP_ERROR;
        return false;
    }

    // --- CONNECTION SUCCESSFUL ---
    
    // Get the content length to allocate buffer
    int contentLength = httpClient.getSize();
    USBSerial.print("Response received, Content-Length: ");
    USBSerial.println(contentLength);

    if (contentLength <= 0 || contentLength > MAX_JPEG_SIZE) {
        USBSerial.println("Invalid or too large content length");
        httpClient.end();
        httpState = HTTP_ERROR;
        return false;
    }

    // Allocate PSRAM buffer for the JPEG data
    if (jpeg_buffer_psram) {
        USBSerial.println("WARNING: jpeg_buffer_psram was not null, freeing old buffer...");
        free(jpeg_buffer_psram);
        jpeg_buffer_psram = nullptr;
        delay(10);  // Give time for memory to be released
    }
    
    // Also check and clean image buffer
    if (image_buffer_psram) {
        USBSerial.println("WARNING: image_buffer_psram exists, freeing...");
        free(image_buffer_psram);
        image_buffer_psram = nullptr;
        delay(10);
    }
    
    printMemoryStats("Before JPEG allocation");
    
    jpeg_buffer_psram = (uint8_t*)ps_malloc(contentLength);
    if (!jpeg_buffer_psram) {
        USBSerial.println("FATAL: Failed to allocate PSRAM for JPEG buffer");
        USBSerial.printf("Requested size: %d bytes\n", contentLength);
        printMemoryStats("JPEG allocation failure");
        httpClient.end();
        httpState = HTTP_ERROR;
        return false;
    }
    
    USBSerial.printf("Successfully allocated JPEG buffer: %d bytes at 0x%08X\n", 
                     contentLength, (uint32_t)jpeg_buffer_psram);
    printMemoryStats("After JPEG allocation");

    jpeg_buffer_size = contentLength;
    jpeg_bytes_received = 0;
    httpRequestStartTime = millis();
    httpState = HTTP_RECEIVING;
    
    USBSerial.println("Starting to receive image data...");
    printMemoryStats("Before returning true");
    USBSerial.println("=== requestImage() EXIT SUCCESS ===");    
    return true;
}


//***************************************************************************************************
// --- HTTP IMAGE INTEGRATION --- Process HTTP response in chunks (non-blocking)
void processHTTPResponse() {
  static bool timeoutMessageShown = false;  // Add this static variable

  if (httpState == HTTP_IDLE || httpState == HTTP_COMPLETE) {
    timeoutMessageShown = false;  // Reset flag when idle or complete
    return; 
  }

  // Check for timeout
  if (millis() - httpRequestStartTime > HTTP_TIMEOUT_MS) {
    if (!timeoutMessageShown) {  // Only print once
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
    
    // Read available data in chunks
    while (stream->available() && jpeg_bytes_received < jpeg_buffer_size) {
      size_t bytesToRead = min((size_t)stream->available(), 
                               jpeg_buffer_size - jpeg_bytes_received);
      size_t bytesRead = stream->readBytes(jpeg_buffer_psram + jpeg_bytes_received, bytesToRead);
      jpeg_bytes_received += bytesRead;
      
      // Yield to other tasks periodically
      if (jpeg_bytes_received % 4096 == 0) {
        lv_timer_handler();
        delay(1);
      }
    }

    // Check if download is complete
    if (jpeg_bytes_received >= jpeg_buffer_size) {
      USBSerial.println("Image download complete. Starting decode...");
      httpClient.end();
      httpState = HTTP_DECODING;
    }
    return;
  }

  if (httpState == HTTP_DECODING) {
    // Free the old decoded image buffer if it exists
    if (image_buffer_psram) {
      free(image_buffer_psram);
      image_buffer_psram = nullptr;
    }

    // Allocate PSRAM buffer for the decoded RGB565 image
    size_t imageBufferSize = screenWidth * screenHeight * sizeof(uint16_t);
    
    printMemoryStats("Before image buffer allocation");
    
    image_buffer_psram = (uint16_t*)ps_malloc(imageBufferSize);
    
    if (!image_buffer_psram) {
      USBSerial.println("FATAL: PSRAM allocation failed for decoded image buffer");
      USBSerial.printf("Requested size: %d bytes\n", imageBufferSize);
      printMemoryStats("Image buffer allocation failure");
      
      free(jpeg_buffer_psram);
      jpeg_buffer_psram = nullptr;
      httpClient.end();
      httpState = HTTP_ERROR;
      return;
    }
    
    USBSerial.printf("Successfully allocated image_buffer_psram: %d bytes at 0x%08X\n", 
                     imageBufferSize, (uint32_t)image_buffer_psram);
    printMemoryStats("After image buffer allocation");

    // Set up the decoder
    TJpgDec.setJpgScale(1);
    TJpgDec.setCallback(tft_output);
    
    // Decode the JPEG. The tft_output callback will fill our image_buffer_psram.
    uint8_t result = TJpgDec.drawJpg(0, 0, jpeg_buffer_psram, jpeg_buffer_size);
    
    // Free the JPEG buffer, we don't need it anymore
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

    // NOW rotate the display to portrait mode just before showing the image
    lv_disp_t * disp = lv_disp_get_default();
    lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);

    // Update the LVGL image descriptor
    img_dsc.header.always_zero = 0;
    img_dsc.header.w = screenWidth;
    img_dsc.header.h = screenHeight;
    img_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
    img_dsc.data_size = screenWidth * screenHeight * LV_COLOR_DEPTH / 8;
    img_dsc.data = (const uint8_t*)image_buffer_psram;

    lv_img_set_src(ui_imgScreen2Background, &img_dsc);
    USBSerial.println("LVGL image source updated.");

    // Make the image visible now that it's loaded
    lv_obj_set_style_opa(ui_imgScreen2Background, LV_OPA_COVER, LV_PART_MAIN);

    httpState = HTTP_COMPLETE;
    requestInProgress = false;  // ADD THIS LINE - Clear flag when request completes
    screen2TimeoutActive = false;

    // Start the 5-minute display timeout
    imageDisplayTimeoutActive = true;
    imageDisplayStartTime = millis();
    USBSerial.println("Image loaded successfully - starting 1-minute display timeout");
    return;
  }

  if (httpState == HTTP_ERROR) {
    httpState = HTTP_IDLE;
    requestInProgress = false;  // ADD THIS LINE - Clear flag on error too
    return;
  }
}


//***************************************************************************************************
/**
 * @brief Shared function to request the latest image.
 * 
 * This function handles the common logic for requesting an image, whether triggered
 * by a button press or MQTT message. It checks preconditions and initiates the request.
 * 
 * @return true if request was successfully initiated, false otherwise
 */
bool requestLatestImage() {
    // Check if already busy with a request
    if (requestInProgress || httpState != HTTP_IDLE) {
        USBSerial.println("Request already in progress, ignoring request");
        return false;
    }
    
    // Only respond if we're on Screen1 (prevent unexpected transitions)
    if (lv_scr_act() != ui_Screen1) {
        USBSerial.println("Not on Screen1, ignoring image request");
        return false;
    }
    
    USBSerial.println("Initiating latest image request");
    
    requestInProgress = true;
    lv_disp_load_scr(ui_Screen2);
    
    if (requestImage("latest")) {
        return true;
    } else {
        USBSerial.println("Failed to initiate image request");
        requestInProgress = false;
        return false;
    }
}


//***************************************************************************************************
// Button event handler for "Latest" image request
void buttonLatest_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    
    if (code == LV_EVENT_CLICKED) {
        USBSerial.println("Latest button clicked");
        requestLatestImage();
    }
}


//***************************************************************************************************
// Button event handler for "New" image request
void buttonNew_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    
    if (code == LV_EVENT_CLICKED) {
        USBSerial.println("New button clicked");
        
        if (requestInProgress || httpState != HTTP_IDLE) {
            USBSerial.println("Request already in progress, ignoring button press");
            return;
        }
        
        requestInProgress = true;
        lv_disp_load_scr(ui_Screen2);
        
        if (requestImage("new")) {
            USBSerial.println("New image request initiated, transitioning to Screen 2");
        } else {
            USBSerial.println("Failed to initiate new image request");
            requestInProgress = false;
        }
    }
}

//***************************************************************************************************
// Button event handler for "Back" image request
void buttonBack_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    
    if (code == LV_EVENT_CLICKED) {
        USBSerial.println("Back button clicked");
        
        if (requestInProgress || httpState != HTTP_IDLE) {
            USBSerial.println("Request already in progress, ignoring button press");
            return;
        }
        
        requestInProgress = true;
        lv_disp_load_scr(ui_Screen2);        
        
        if (requestImage("back")) {
            USBSerial.println("Back image request initiated, transitioning to Screen 2");
        } else {
            USBSerial.println("Failed to initiate back image request");
            requestInProgress = false;
        }
    }
}


//***************************************************************************************************
// --- HTTP IMAGE INTEGRATION --- Event handler for Screen 2
// This function is called whenever an event happens on Screen 2.
void screen2_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_disp_t * disp = lv_disp_get_default(); 

    if (code == LV_EVENT_SCREEN_LOADED) {
        USBSerial.println("Screen 2 Loaded.");

        // Make the image widget completely transparent until image is ready
        lv_obj_set_style_opa(ui_imgScreen2Background, LV_OPA_TRANSP, LV_PART_MAIN);

        // Start the timeout timer
        screenTransitionTime = millis();
        screen2TimeoutActive = true;

        // Only reset display timeout if we're not already in the middle of displaying an image
        if (httpState != HTTP_COMPLETE) {
            imageDisplayTimeoutActive = false;  // Not viewing yet, still loading
        }

        USBSerial.println("Screen 2 timeout started (8 seconds)");

        // Check if request was already initiated by button handler
        if (httpState == HTTP_REQUESTING || httpState == HTTP_RECEIVING) {
            USBSerial.println("Request already in progress from button handler, waiting for completion");
        } else if (httpState == HTTP_IDLE && !requestInProgress) {
            // This shouldn't normally happen - we should always arrive here with a request in progress
            USBSerial.println("WARNING: Screen 2 loaded but no request was initiated");
            // Could optionally go back to Screen 1 or show an error
        }
        // If httpState is something else (DECODING, COMPLETE, ERROR), processHTTPResponse will handle it
        
    } else if (code == LV_EVENT_SCREEN_UNLOAD_START) {
        USBSerial.println("Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.");
        
        // Disable both timeouts when leaving Screen 2
        screen2TimeoutActive = false;
        imageDisplayTimeoutActive = false;
        
        // Make the image transparent when leaving the screen
        lv_obj_set_style_opa(ui_imgScreen2Background, LV_OPA_TRANSP, LV_PART_MAIN);
        
        // Free memory
        if (image_buffer_psram != nullptr) {
            free(image_buffer_psram);
            image_buffer_psram = nullptr;
        }
        if (jpeg_buffer_psram != nullptr) {
            free(jpeg_buffer_psram);
            jpeg_buffer_psram = nullptr;
        }
        
        // Clear the image descriptor structure to prevent stale pointers
        memset(&img_dsc, 0, sizeof(lv_img_dsc_t));
        
        // Reset HTTP state and request flag
        httpState = HTTP_IDLE;
        requestInProgress = false;  // IMPORTANT: Clear the flag when leaving Screen 2

        // Set display back to the default landscape mode for other screens
        lv_disp_set_rotation(disp, LV_DISP_ROT_90);
    }
}


//***************************************************************************************************
/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(disp);
}


//***************************************************************************************************
void increase_lvgl_tick(void *arg) {
  /* Tell LVGL how many milliseconds has elapsed */
  lv_tick_inc(LVGL_TICK_PERIOD_MS);
}


//***************************************************************************************************
/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  int32_t touchX = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
  int32_t touchY = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

  if (FT3168->IIC_Interrupt_Flag == true) {
    FT3168->IIC_Interrupt_Flag = false;
    data->state = LV_INDEV_STATE_PR;

    /*Set the coordinates*/
    data->point.x = touchX;
    data->point.y = touchY;   

  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}


//***************************************************************************************************
void updatePowerDisplay(String powerStr) {
    String displayText = LV_SYMBOL_HOME " " + powerStr + " W";
    lv_label_set_text(ui_labelPowerValue, displayText.c_str());
}


//***************************************************************************************************
void updateEnergyDisplay(String energyStr) {
    String displayText = energyStr;
    lv_label_set_text(ui_labelEnergyValue, displayText.c_str());
}


//***************************************************************************************************
void callbackMqtt(char* topic, byte* payload, unsigned int length) {
    String topicString = topic;
    
    // Safely extract payload
    String payloadString = "";
    for (unsigned int i = 0; i < length; i++) {
        payloadString += (char)payload[i];
    }
    payloadString.trim();
    
    // Image request handling
    if (topicString == MQTT_IMAGE_TOPIC) {
        if (payloadString == "latest") {
            requestLatestImage();
        }
    }
    // Power handling
    else if (topicString == HILO_POWER) {
        USBSerial.print("Power: ");
        USBSerial.println(payloadString);
        updatePowerDisplay(payloadString);
    }
    // Energy handling
    else if (topicString == HILO_ENERGY) {
        USBSerial.print("Energy: ");
        USBSerial.println(payloadString);
        updateEnergyDisplay(payloadString);
    }
}


//***************************************************************************************************
//MQTT connection with proper reconnection logic
void checkMQTT(bool bypassRateLimit = false) {  // ADD parameter with default value
  yield(); 
  if (!mqttClient.connected()) {
    // Check if enough time has passed since last attempt
    unsigned long currentTime = millis();
    if (!bypassRateLimit && currentTime - lastMqttAttempt < MQTT_RECONNECT_INTERVAL) {
      // Not enough time has passed, skip this attempt
      return;
    }
    
    lastMqttAttempt = currentTime;
    
    USBSerial.print("Attempting MQTT connection...");
    
    // Explicitly disconnect to clean up any stale connection state
    mqttClient.disconnect();
    delay(100);  // Brief delay to allow cleanup
    
    // For secure connections, we might need to reset the client state
    // The client is already set up in connectToWiFi(), so we just attempt connection
    
    if (mqttClient.connect(CLIENT_ID, USERNAME, KEY)) {
      mqttConnection = true;
      mqttSuccess = true;
      USBSerial.println(" connected");                                    
      USBSerial.print("MQTT connected, rc=");                                                     
      USBSerial.println(mqttClient.state());    

      if (mqttClient.subscribe(MQTT_IMAGE_TOPIC, 1)) {  // QoS 1
          USBSerial.print("Subscribed to topic: ");
          USBSerial.println(MQTT_IMAGE_TOPIC);
      } else {
          USBSerial.println("Failed to subscribe to image topic");
      } 
      mqttClient.subscribe(HILO_POWER, 1);
      mqttClient.subscribe(HILO_ENERGY, 1);                               
    } else {
      mqttConnection = false;
      USBSerial.print("failed, rc=");
      USBSerial.print(mqttClient.state());
      USBSerial.print(" - will retry in ");
      USBSerial.print(MQTT_RECONNECT_INTERVAL / 1000);
      USBSerial.println(" seconds");
    } 
  }
}


//***************************************************************************************************
bool connectToWiFi(int connection) {      // connection is either 1 for wifi1 or 2 for wifi2
  bool connected = false;

  USBSerial.println("Connecting to WiFi");

  for (int i = 0; i < 30 && !connected; i++) {   // Loop for a maximum of 6 seconds, 12
      USBSerial.print(".");
      updateMotionState();      // Poll the motion sensor
      updateMotionStatusUI();   // Update the icon's visibility
      lv_timer_handler();       // IMPORTANT: Tell LVGL to process tasks and redraw
      
      delay(500);
      
      // Check WiFi status and print detailed status messages
      wl_status_t wifiStatus = WiFi.status();
      
      switch (wifiStatus) {
        case WL_IDLE_STATUS:
          USBSerial.println("WiFi is idle, waiting for connection...");
          break;
        case WL_NO_SSID_AVAIL:
          USBSerial.println("Specified SSID not available.");
          break;
        case WL_CONNECT_FAILED:
          USBSerial.println("Connection failed. Check credentials or signal strength.");
          break;
        case WL_CONNECTION_LOST:
          USBSerial.println("Connection lost, retrying...");
          break;
        case WL_DISCONNECTED:
          USBSerial.println("WiFi disconnected, attempting to reconnect...");
          break;
        case WL_CONNECTED:
          connected = true;
          break;
        default:
          USBSerial.println("Unknown WiFi status.");
          break;
      }
  }

  if (connected) {
    USBSerial.println("");
    USBSerial.println("WiFi connected.");
    USBSerial.print("SSID: ");
    USBSerial.println(WiFi.SSID());    
    USBSerial.println("IP: " + WiFi.localIP().toString());

    // Disable Wi-Fi Power Save for stability
    WiFi.setSleep(false);
    USBSerial.println("INFO: Wi-Fi Power Save disabled for stability.");

    // Determine secure vs non-secure based on PORT NUMBER, not connection number
    if (connection == 1) {
      // Check if SERVER1 uses secure port
      if (SERVERPORT1 == 9735 || SERVERPORT1 == 8883) {
        // Secure MQTT
        secureClient.setCACert(ca_cert);
        mqttClient.setClient(secureClient);
        mqttClient.setServer(SERVER1, SERVERPORT1);
        USBSerial.print("Using secure MQTT with TLS on port ");
        USBSerial.println(SERVERPORT1);
      } else {
        // Non-secure MQTT
        mqttClient.setClient(espClient);
        mqttClient.setServer(SERVER1, SERVERPORT1);
        USBSerial.print("Using standard MQTT on port ");
        USBSerial.println(SERVERPORT1);
      }
    } else {
      // connection == 2
      // Check if SERVER2 uses secure port
      if (SERVERPORT2 == 9735 || SERVERPORT2 == 8883) {
        // Secure MQTT
        secureClient.setCACert(ca_cert);
        mqttClient.setClient(secureClient);
        mqttClient.setServer(SERVER2, SERVERPORT2);
        USBSerial.print("Using secure MQTT with TLS on port ");
        USBSerial.println(SERVERPORT2);
      } else {
        // Non-secure MQTT
        mqttClient.setClient(espClient);
        mqttClient.setServer(SERVER2, SERVERPORT2);
        USBSerial.print("Using standard MQTT on port ");
        USBSerial.println(SERVERPORT2);
      }
    }
    
    mqttClient.setCallback(callbackMqtt);  
    checkMQTT();     
  } else {
    USBSerial.println("Failed to connect to WiFi.");
  }

  return connected;
}


//***************************************************************************************************
/**
 * @brief Scans for a specific WiFi SSID in a non-blocking way.
 *
 * This function uses an asynchronous WiFi scan. It starts the scan and then enters
 * a loop that continuously updates the UI and motion state while waiting for the
 * scan to complete. This prevents the UI from freezing for several seconds.
 *
 * @param ssid The name of the WiFi network to search for.
 * @return true if the SSID was found, false otherwise or if the scan times out.
 */
bool scanForSSID(const char* ssid) {
  USBSerial.print("Starting non-blocking scan for SSID: ");
  USBSerial.println(ssid);

  // Start an ASYNCHRONOUS scan. This function returns immediately.
  WiFi.scanNetworks(true);

  unsigned long startTime = millis();
  const unsigned long SCAN_TIMEOUT = 4000; // 10-second timeout for the scan 10000

  // This is our "keep-alive" loop that runs while the scan is in progress.
  // WiFi.scanComplete() returns -1 while scanning.
  while (WiFi.scanComplete() == -1) {
    
    // --- KEEP UI AND SENSORS ALIVE ---
    updateMotionState();
    updateMotionStatusUI();
    lv_timer_handler();

    // Check if the scan has timed out
    if (millis() - startTime > SCAN_TIMEOUT) {
      USBSerial.println("\nWiFi scan timed out!");
      WiFi.scanDelete(); // Clean up the scan results
      return false;
    }

    delay(50); // Wait a short moment to prevent this loop from hogging the CPU
  }

  // The scan is now complete. Let's check the results.
  int numNetworks = WiFi.scanComplete();
  USBSerial.print("\nScan complete. Found ");
  USBSerial.print(numNetworks);
  USBSerial.println(" networks.");

  bool ssidFound = false;
  for (int i = 0; i < numNetworks; i++) {
    if (WiFi.SSID(i) == ssid) {
      ssidFound = true;
      break;
    }
  }

  // Clean up the memory used by the scan results
  WiFi.scanDelete();

  if (ssidFound) {
    USBSerial.println("Specified SSID found.");
  } else {
    USBSerial.println("Specified SSID not found.");
  }
  
  return ssidFound;
}


//***************************************************************************************************
// The optimized function now uses the generic 'secondary' variables
void trySecondSSID(int lastAttemptedConnection) {
    USBSerial.print("Attempting to connect to secondary network: ");
    USBSerial.println(secondarySsid);

    if (scanForSSID(secondarySsid)) {
        USBSerial.println("SSID found. Attempting to connect...");
        
        WiFi.disconnect(true); // Ensure clean state before new attempt
        delay(10);    // was ok at 1000

        WiFi.begin(secondarySsid, secondaryPassword);
        if (connectToWiFi(secondaryNetworkNum)) {
            USBSerial.print("Successfully connected to Wi-Fi network: ");
            USBSerial.println(secondarySsid);
        } else {
            USBSerial.println("Failed to connect to both Wi-Fi networks.");
        }
    } else {
        USBSerial.print("SSID '");
        USBSerial.print(secondarySsid);
        USBSerial.println("' not found in scan.");
        USBSerial.println("Failed to connect to both Wi-Fi networks.");
    }
}


//***************************************************************************************************
/**
 * @brief Attempts to connect to WiFi networks with retry logic based on power source.
 * 
 * When USB power is present (vbusPresent == true):
 *   - Retries WiFi connection up to 5 times with delays between attempts
 *   - Updates UI with retry count and status
 *   - Initiates shutdown after 5 failed attempts
 *   - Allows switching to battery mode during retry (will shutdown on next failure)
 * 
 * When on battery power only (vbusPresent == false):
 *   - Makes 2 connection attempts (primary and secondary networks)
 *   - If both fail, initiates shutdown to conserve battery
 * 
 * @return true if WiFi connection was successful, false otherwise
 */
bool attemptWiFiConnection() {
    const int MAX_RETRIES = 5;  // Maximum number of retry attempts
    int retryCount = 0;
    bool connected = false;
    
    // Initial UI state before starting connection
    lv_label_set_text(ui_labelConnectionStatus, "Connecting...");
    lv_obj_set_style_text_color(ui_labelConnectionStatus, lv_color_hex(0xFFFFFF), LV_PART_MAIN); // White
    // Force a refresh to show "Connecting..." immediately
    for (int i = 0; i < 5; i++) { lv_timer_handler(); delay(5); }
    
    while (!connected && retryCount < MAX_RETRIES) {  // MODIFIED: Added retry limit check
        retryCount++;
        
        // Update UI with retry count if this is a retry
        if (retryCount > 1) {
            char statusBuffer[32];
            snprintf(statusBuffer, sizeof(statusBuffer), "Retry %d/%d...", retryCount - 1, MAX_RETRIES - 1);  // MODIFIED: Show x/5 format
            lv_label_set_text(ui_labelConnectionStatus, statusBuffer);
            lv_obj_set_style_text_color(ui_labelConnectionStatus, lv_color_hex(0xFFB700), LV_PART_MAIN); // Orange
            for (int i = 0; i < 5; i++) { lv_timer_handler(); delay(5); }
            
            USBSerial.print("WiFi Connection Retry #");
            USBSerial.print(retryCount - 1);
            USBSerial.print(" of ");
            USBSerial.println(MAX_RETRIES - 1);
        }
        
        // Try primary network
        USBSerial.print("Attempting to connect to primary network: ");
        USBSerial.println(primarySsid);
        
        if (scanForSSID(primarySsid)) {
            WiFi.begin(primarySsid, primaryPassword);
            if (connectToWiFi(primaryNetworkNum)) {
                USBSerial.println("Connection successful!");
                connected = true;
                break;
            } else {
                USBSerial.println("Primary network found, but connection failed.");
            }
        } else {
            USBSerial.println("Primary network not found in scan.");
        }
        
        // If primary failed, try secondary network
        if (!connected) {
            trySecondSSID(primaryNetworkNum); // Attempt fallback
            
            // Check if secondary connection succeeded
            if (WiFi.status() == WL_CONNECTED) {
                connected = true;
                break;
            }
        }
        
        // --- CRITICAL: Check power source after failed attempt ---
        updatePowerStatus(); // Refresh vbusPresent status
        
        if (!vbusPresent) {
            // Battery power only - shutdown after failed attempts
            USBSerial.println("Battery power only and WiFi connection failed.");
            USBSerial.println("Initiating shutdown to conserve battery...");
            
            // Update UI before shutdown
            lv_label_set_text(ui_labelConnectionStatus, "No WiFi - Shutdown");
            lv_obj_set_style_text_color(ui_labelConnectionStatus, lv_color_hex(0xFF0000), LV_PART_MAIN); // Red
            for (int i = 0; i < 5; i++) { lv_timer_handler(); delay(5); }
            
            delay(2000); // Show message briefly
            goToShutdown(); // This function does not return
            
            // Code should never reach here, but just in case:
            return false;
        }
        
        // --- Check if max retries reached ---
        if (retryCount >= MAX_RETRIES) {  // NEW: Check if we've exhausted all retries
            USBSerial.println("Maximum retry attempts reached. Initiating shutdown...");
            
            // Update UI before shutdown
            lv_label_set_text(ui_labelConnectionStatus, "No WiFi - Shutdown");
            lv_obj_set_style_text_color(ui_labelConnectionStatus, lv_color_hex(0xFF0000), LV_PART_MAIN); // Red
            for (int i = 0; i < 5; i++) { lv_timer_handler(); delay(5); }
            
            delay(2000); // Show message briefly
            goToShutdown(); // This function does not return
            
            // Code should never reach here, but just in case:
            return false;
        }
        
        // --- USB Power Present - Setup for retry ---
        USBSerial.print("USB power present. Waiting ");
        USBSerial.print(WIFI_RETRY_DELAY_MS / 1000);
        USBSerial.println(" seconds before retry...");
        
        // Non-blocking delay that keeps UI responsive
        unsigned long delayStart = millis();
        while (millis() - delayStart < WIFI_RETRY_DELAY_MS) {
            updateMotionState();
            updateMotionStatusUI();
            updateBatteryInfo();
            updateBatteryInfoUI();
            lv_timer_handler();
            
            // Check if USB was disconnected during the delay
            updatePowerStatus();
            if (!vbusPresent) {
                USBSerial.println("USB power removed during retry delay. Next attempt will be final.");
                break; // Exit delay early, will check vbusPresent again at loop start
            }
            
            delay(100); // Small delay to prevent tight loop
        }
        
        // Clean up WiFi state before next attempt
        WiFi.disconnect(true);
        delay(100);
    }
    
    return connected;
}


//***************************************************************************************************
/**
 * @brief Checks motion state and updates a global variable.
 *
 * This function should be called continuously. It polls the IMU, updates the
 * global 'g_isCurrentlyMoving' variable instantly upon motion, and returns
 * true only after the full STATIONARY_TIMEOUT has elapsed.
 * @return true if the device has been stationary for the full timeout, false otherwise.
 */
bool updateMotionState() {
  static const unsigned long MOTION_CHECK_INTERVAL = 100; // Poll every 100ms

  static unsigned long lastMotionCheckTime = 0;
  static unsigned long lastMotionTime = 0; // Initialize at 0, will be set on first loop

  // Initialize lastMotionTime on the first run to avoid immediate shutdown on boot
  if (lastMotionTime == 0) {
      lastMotionTime = millis();
  }
  
  unsigned long currentTime = millis();

  // Poll the IMU at a fixed interval
  if (currentTime - lastMotionCheckTime >= MOTION_CHECK_INTERVAL) {
    lastMotionCheckTime = currentTime;

    // Check the IMU hardware flag for a motion event
    uint8_t status1 = qmi.readStatus1();
    bool motionHardwareFlag = (status1 & 0b00000100);

    if (motionHardwareFlag) {
      // Motion was just detected
      if (!g_isCurrentlyMoving) {
        USBSerial.println("Movement Detected!");
        g_isCurrentlyMoving = true;

        // Publish MQTT motion event 
        if (ENABLE_MOTION_MQTT && mqttClient.connected()) {
          mqttClient.publish(MOTION_TOPIC, "1");
          lastMotionTXTime = millis();
          USBSerial.println("TX motion MQTT: Moving (immediate)");
        }
      }
      // Every time motion is detected, reset the stationary timer
      lastMotionTime = currentTime;
    } else {
      // No motion was detected in this check
      if (g_isCurrentlyMoving && (currentTime - lastMotionTime > MOTION_TIMEOUT)) {
        USBSerial.println("Movement Stopped.");
        g_isCurrentlyMoving = false;
      }
    }
  }

  return false; // Not yet time to shut down
}


//***************************************************************************************************
void Arduino_IIC_Touch_Interrupt(void) {
  if (FT3168) { // Safety check: ensure object exists before using it
    FT3168->IIC_Interrupt_Flag = true;
  }
}


//***************************************************************************************************
void adcOn() {
  pmic.enableTemperatureMeasure();
  pmic.enableBattDetection();
  pmic.enableVbusVoltageMeasure();
  pmic.enableBattVoltageMeasure();
  pmic.enableSystemVoltageMeasure();
  adc_switch = true;
}


//***************************************************************************************************
void adcOff() {
  pmic.disableTemperatureMeasure();
  pmic.disableBattDetection();
  pmic.disableVbusVoltageMeasure();
  pmic.disableBattVoltageMeasure();
  pmic.disableSystemVoltageMeasure();
  adc_switch = false;
}


//***************************************************************************************************
void updatePowerStatus() {
  static bool prevVbusPresent = false;  // Track previous state to detect transitions
  
  vbusPresent = pmic.isVbusIn();
  batteryConnected = pmic.isBatteryConnect();

  // Detect USB connection (wasn't present, now is)
  if (vbusPresent && !prevVbusPresent) {
    usbWasEverPresent = true;
    usbDisconnectedTime = 0;  // Reset disconnect timer
    allowSleep = false;  // Prevent sleeping when USB is present
    USBSerial.println("USB Power Connected - Sleep disabled");
  }
  
  // Detect USB disconnection (was present, now isn't)
  if (!vbusPresent && prevVbusPresent && usbWasEverPresent) {
    usbDisconnectedTime = millis();  // Record when USB was lost
    allowSleep = true;  // ADD THIS LINE - Enable sleep after USB loss
    USBSerial.println("USB Power Disconnected - Starting grace period with motion monitoring");
  }
  
  prevVbusPresent = vbusPresent;  // Save current state for next comparison
}


//***************************************************************************************************
void updateBatteryInfo() {
  updatePowerStatus();

  if (adc_switch) {
    batteryVoltage = pmic.getBattVoltage() / 1000.0; // Convert to volts

    if (batteryVoltage < 2.4) {
      batteryConnected = false;
    }

    if (batteryConnected) {
      int battPercent = pmic.getBatteryPercent();
      batteryPercent = String(battPercent) + "%";
    } else {
      batteryVoltage = 0.0; // Clean up the display value for the "No Battery" case.
      batteryPercent = "N/A";
    }
  } else {
    // If ADC is off, assume no battery connection.
    batteryConnected = false;
    batteryVoltage = 0.0;
    batteryPercent = "N/A";
  }
}


//***************************************************************************************************
/**
 * @brief Updates the LVGL labels with the latest battery information.
 *
 * This function should be called repeatedly. It efficiently checks for changes
 * in the battery state and only updates the UI labels when necessary.
 *
 * Assumes you have created the following labels in SquareLine Studio:
 * - ui_labelBatteryPercent
 * - ui_labelBatteryVoltage
 * - ui_labelBatteryStatus
 */
void updateBatteryInfoUI() {
  // Static variables to store the previous state sent to the UI.
  static String prevUiPercent = "";
  static float prevUiVoltage = -1.0f;
  static String prevUiStatus = "";

  // Get the current charging state.
  bool isCharging = pmic.isCharging();
  String currentStatus = "";

  // Determine the current status string.
  if (batteryConnected) {
    if (isCharging) {
      currentStatus = "Charging";
    } else {
      currentStatus = "On Battery";
    }
  } else {
    if (vbusPresent) {
      currentStatus = "USB Power";
    } else {
      currentStatus = "No Power";
    }
  }

  // Check if any UI-relevant value has changed.
  bool uiShouldUpdate = (batteryPercent != prevUiPercent) ||
                        (abs(batteryVoltage - prevUiVoltage) > 0.01) ||
                        (currentStatus != prevUiStatus);

  // If nothing has changed, exit the function early.
  if (!uiShouldUpdate) {
    return;
  }

  // --- Logic to update the UI labels based on the new state ---

  // Update battery Label with icon
  if (batteryPercent != prevUiPercent) {
      if (batteryConnected) {
          // Determine which battery icon to use based on percentage
          const char* batteryIcon;
          int percent = batteryPercent.substring(0, batteryPercent.length() - 1).toInt(); // Remove '%' and convert
          
          if (percent >= 80) {
              batteryIcon = LV_SYMBOL_BATTERY_FULL;
          } else if (percent >= 60) {
              batteryIcon = LV_SYMBOL_BATTERY_3;
          } else if (percent >= 40) {
              batteryIcon = LV_SYMBOL_BATTERY_2;
          } else if (percent >= 20) {
              batteryIcon = LV_SYMBOL_BATTERY_1;
          } else {
              batteryIcon = LV_SYMBOL_BATTERY_EMPTY;
          }
          
          String displayText = String(batteryIcon);
          lv_label_set_text(ui_batText, displayText.c_str());
          lv_label_set_text(ui_labelBatteryPercent, batteryPercent.c_str());
      } else {
          lv_label_set_text(ui_batText, LV_SYMBOL_BATTERY_EMPTY);
          lv_label_set_text(ui_labelBatteryPercent, ""); // Show placeholder if no battery
      }

      prevUiPercent = batteryPercent;
  }

  // Update Voltage Label
  if (abs(batteryVoltage - prevUiVoltage) > 0.01) {
    char voltageBuffer[8]; // Buffer for "x.xxV"
    if (batteryConnected) {
      snprintf(voltageBuffer, sizeof(voltageBuffer), "%.2fV", batteryVoltage);
      lv_label_set_text(ui_labelBatteryVoltage, voltageBuffer);
    } else {
       lv_label_set_text(ui_labelBatteryVoltage, ""); // Show placeholder
    }
    prevUiVoltage = batteryVoltage;
  }

  // Update Status Label and Color
  if (currentStatus != prevUiStatus) {
    lv_label_set_text(ui_labelBatteryStatus, currentStatus.c_str());
    if (currentStatus == "Charging") {
      lv_obj_set_style_text_color(ui_labelBatteryStatus, lv_color_hex(0x00FF00), LV_PART_MAIN); // Green
    } else if (currentStatus == "USB Power") {
      lv_obj_set_style_text_color(ui_labelBatteryStatus, lv_color_hex(0x00FFFF), LV_PART_MAIN); // Cyan
    } else {
      lv_obj_set_style_text_color(ui_labelBatteryStatus, lv_color_hex(0xFFFFFF), LV_PART_MAIN); // White (Default)
    }
    prevUiStatus = currentStatus;
  }
}


//***************************************************************************************************
/**
 * @brief Updates the connection status label on the UI.
 *
 * Checks the current state of WiFi and MQTT and updates a dedicated label
 * with concise, color-coded status messages. It only redraws the label
 * when the connection state actually changes.
 * Assumes a label named 'ui_labelConnectionStatus' exists.
 */
void updateConnectionStatusUI() {
    static int prev_wifi_status = -1;
    static bool prev_mqtt_status = false;

    // Get the current status
    int current_wifi_status = WiFi.status();
    bool current_mqtt_status = mqttClient.connected();

    // Exit early if nothing has changed
    if (current_wifi_status == prev_wifi_status && current_mqtt_status == prev_mqtt_status) {
        return;
    }

    // --- A change was detected, update the label ---
    if (current_mqtt_status) {
        // Best case: MQTT is online
        lv_label_set_text(ui_labelConnectionStatus, "MQTT Online");
        lv_obj_set_style_text_color(ui_labelConnectionStatus, lv_color_hex(0x00FF00), LV_PART_MAIN); // Green
    } else if (current_wifi_status == WL_CONNECTED) {
        // Good case: WiFi is connected, but MQTT is not (or is trying)
        lv_label_set_text(ui_labelConnectionStatus, "WiFi Connected");
        lv_obj_set_style_text_color(ui_labelConnectionStatus, lv_color_hex(0xFFB700), LV_PART_MAIN); // Orange
    } else {
        // Worst case: No WiFi connection
        lv_label_set_text(ui_labelConnectionStatus, "Offline");
        lv_obj_set_style_text_color(ui_labelConnectionStatus, lv_color_hex(0xFF0000), LV_PART_MAIN); // Red
    }

    // Save the current state for the next check
    prev_wifi_status = current_wifi_status;
    prev_mqtt_status = current_mqtt_status;
}


//***************************************************************************************************
/**
 * @brief Updates the visibility of the motion icon on the UI.
 *
 * Reads the global 'g_isCurrentlyMoving' variable and shows or hides the
 * 'ui_labelMotionIcon' accordingly. Uses a static variable to only
 * change the UI when the motion state changes.
 */
void updateMotionStatusUI() {
    static bool prev_is_moving_state = false;

    // Exit early if the state hasn't changed
    if (g_isCurrentlyMoving == prev_is_moving_state) {
        return;
    }

    if (g_isCurrentlyMoving) {
        // Show the icon if moving
        lv_obj_clear_flag(ui_labelMotionIcon, LV_OBJ_FLAG_HIDDEN);
    } else {
        // Hide the icon if not moving
        lv_obj_add_flag(ui_labelMotionIcon, LV_OBJ_FLAG_HIDDEN);
    }

    // Save the new state for the next check
    prev_is_moving_state = g_isCurrentlyMoving;
}


//***************************************************************************************************
/**
 * @brief LVGL event handler to reset the inactivity timer.
 *
 * This function is called by LVGL whenever the screen is pressed. Its sole
 * purpose is to update the 'lastActivityTime' global variable, which is
 * used to detect user inactivity for triggering deep sleep.
 * @param e Pointer to the LVGL event data.
 */
void activity_event_handler(lv_event_t * e) {
    USBSerial.println("Screen touched, resetting inactivity timer.");
    lastActivityTime = millis();
}


//***************************************************************************************************
void goToDeepSleep() {
  USBSerial.println("Preparing to enter Deep Sleep...");

  // --- Display "Sleeping..." message on the UI ---
  lv_label_set_text(ui_labelConnectionStatus, "Sleeping...");
  lv_obj_set_style_text_color(ui_labelConnectionStatus, lv_color_hex(0xFFB700), LV_PART_MAIN); // Orange

  // Force LVGL to redraw the screen with the new message NOW.
  for (int i = 0; i < 5; i++) {
      lv_timer_handler();
      delay(5);
  }

  // Keep the message on screen for 1 second before sleeping.
  delay(1000);  

  USBSerial.println("Entering Deep Sleep (4 mA mode)... Touch screen to wake.");
  
  adcOff();
  
  FT3168->IIC_Write_Device_State(FT3168->Arduino_IIC_Touch::Device::TOUCH_POWER_MODE,
                                 FT3168->Arduino_IIC_Touch::Device_Mode::TOUCH_POWER_MONITOR);
  
  gfx->displayOff();
  
  pmic.disableALDO1(); 
  pmic.disableALDO2(); 
  pmic.disableBLDO1();

  Wire.end();

  esp_sleep_enable_ext0_wakeup((gpio_num_t)TP_INT, 0);
  delay(100);
  esp_deep_sleep_start();
}


//***************************************************************************************************
void goToShutdown() {
  USBSerial.println("Preparing to shut down...");

  // --- Display "Shutdown..." message on the UI ---
  lv_label_set_text(ui_labelConnectionStatus, "Shutdown...");
  lv_obj_set_style_text_color(ui_labelConnectionStatus, lv_color_hex(0xFF0000), LV_PART_MAIN); // Red
  
  // Hide the motion icon as it's no longer relevant
  lv_obj_add_flag(ui_labelMotionIcon, LV_OBJ_FLAG_HIDDEN);

  // Force LVGL to redraw the screen immediately
  for (int i = 0; i < 5; i++) {
      lv_timer_handler();
      delay(5);
  }

  // Keep the message on screen for 1 second.
  delay(1000);

  USBSerial.println("Shutting down completely... Press PWR button to start.");

  gfx->displayOff();  

  USBSerial.println("Attempting PMIC shutdown...");
  USBSerial.flush();

  // 2. Power down the WiFi radio.
  USBSerial.println("Turning off WiFi radio...");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(500); 

  // 3. Command all I2C peripherals to enter their lowest power state.
  USBSerial.println("Putting I2C peripherals to sleep...");
  if (FT3168) {
      FT3168->IIC_Write_Device_State(Arduino_IIC_Touch::Device::TOUCH_POWER_MODE,
                                     Arduino_IIC_Touch::Device_Mode::TOUCH_POWER_HIBERNATE);
  }
  qmi.disableIMU();
  delay(50);

  // 4. Power down internal MCU systems.
  adcOff();
  
  // 5. Clear all pending interrupt flags in the PMIC.
  USBSerial.println("Clearing PMIC IRQ status...");
  pmic.clearIrqStatus();
  delay(50);

  // 6. Disable ALL power rails controlled by the PMIC.
  pmic.disableALDO1(); 
  pmic.disableALDO2(); 
  pmic.disableALDO3();
  pmic.disableBLDO1();
 
  delay(100);
  
  // 7. Finally, command the shutdown.
  pmic.shutdown();

  // The code should NEVER reach this point.
  while(1) {
    delay(1000);
    USBSerial.println("Shutdown failed. System is stuck.");
  }
}


//***************************************************************************************************
// INITIALIZATION HELPER FUNCTIONS
/****************************************************************************************************
 * Initialize USB Serial communication
 * @return true if successful
 */
bool initSerial() {
    USBSerial.begin(115200);
    USBSerial.println("\n--- Board is starting up ---");
    return true;
}

/****************************************************************************************************
 * Check wake reason and handle I2C driver reset if needed
 * @return Wake reason code
 */
esp_sleep_wakeup_cause_t handleWakeReason() {
    esp_sleep_wakeup_cause_t wakeReason = esp_sleep_get_wakeup_cause();

    if (wakeReason == ESP_SLEEP_WAKEUP_EXT0) {
        // This is a Deep Sleep Wake. The ESP32's I2C driver is stuck.
        // We MUST reset it to prevent the "i2c driver install error".
        USBSerial.println("Deep Sleep Wake detected. Performing I2C driver reset...");
        Wire.end();
        delay(10);
    } else {
        USBSerial.println("Woke up from Power-On or Full Shutdown.");
    }
    // For any other type of boot (Cold or Shutdown), we DO NOT call Wire.end().

    return wakeReason;
}

/****************************************************************************************************
 * Initialize I2C bus
 * @return true if successful
 */
bool initI2C() {
    USBSerial.println("--- Initializing I2C Bus ---");
    Wire.begin(IIC_SDA, IIC_SCL);
    delay(50);
    USBSerial.println("I2C bus initialized");
    return true;
}

/****************************************************************************************************
 * Initialize PMIC (AXP2101) - Power Management IC
 * Configures charging, voltage rails, and ADC
 * 
 * IMPORTANT: WiFi.mode(WIFI_OFF) must be called in setup() BEFORE calling this function
 * 
 * @return true if successful, false otherwise
 */
bool initPMIC() {
    USBSerial.println("--- Initializing PMIC (AXP2101) ---");
    
    if (!pmic.init()) {
        USBSerial.println("ERROR: PMIC AXP2101 failed to initialize!");
        // Original code CONTINUES here, doesn't return false!
    } else {
        USBSerial.println("PMIC init OK.");

        USBSerial.println("Performing full ADC subsystem reset to ensure clean state...");
        adcOff();
        delay(50); 
        adcOn();
        delay(150);
        
        pmic.enableALDO1(); 
        pmic.enableALDO2(); 
        pmic.enableBLDO1(); 
        pmic.enableALDO3();
    }
    
    USBSerial.println("PMIC initialization complete");
    return true;  // Always returns true, even if init failed
}

/****************************************************************************************************
 * Initialize I/O Expander (TCA9554)
 * Sets up GPIO pins for LCD control
 * @return true if successful, false otherwise
 */
bool initIOExpander() {
    USBSerial.println("--- Initializing I/O Expander ---");
    
    expander = new EXAMPLE_CHIP_CLASS(TCA95xx_8bit,
                                      (i2c_port_t)0, ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000,
                                      IIC_SCL, IIC_SDA);
    
    if (!expander) {
        USBSerial.println("ERROR: Failed to create I/O Expander object");
        return false;
    }
    
    expander->init();
    expander->begin();
    
    // Configure pins for LCD control
    expander->pinMode(0, OUTPUT);
    expander->pinMode(1, OUTPUT);
    expander->pinMode(2, OUTPUT);
    
    // Reset sequence
    expander->digitalWrite(0, LOW);
    expander->digitalWrite(1, LOW);
    expander->digitalWrite(2, LOW);
    delay(20);
    expander->digitalWrite(0, HIGH);
    expander->digitalWrite(1, HIGH);
    expander->digitalWrite(2, HIGH);
    
    USBSerial.println("I/O Expander initialization complete");
    return true;
}

/****************************************************************************************************
 * Initialize Touch Controller (FT3168)
 * Sets up I2C communication with capacitive touch panel
 * @return true if successful, false otherwise
 */
bool initTouch() {
    USBSerial.println("--- Initializing Touch Controller ---");
    
    // Create shared I2C bus for touch
    IIC_Bus = std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);
    FT3168 = std::make_unique<Arduino_FT3x68>(IIC_Bus, FT3168_DEVICE_ADDRESS, 
                                               DRIVEBUS_DEFAULT_VALUE, TP_INT, 
                                               Arduino_IIC_Touch_Interrupt);
    
    // CRITICAL: Match original infinite retry behavior
    while (FT3168->begin() == false) {
        USBSerial.println("ERROR: FT3168 initialization fail");
        delay(2000);
        // Keep trying forever, just like the original!
    }
    
    // Set touch to active power mode
    FT3168->IIC_Write_Device_State(FT3168->Arduino_IIC_Touch::Device::TOUCH_POWER_MODE,
                                   FT3168->Arduino_IIC_Touch::Device_Mode::TOUCH_POWER_ACTIVE);
    
    USBSerial.println("Touch controller initialization complete");
    return true;
}

/****************************************************************************************************
 * Initialize Display Hardware
 * Sets up display controller and basic configuration
 * @param wakeReason Wake reason from handleWakeReason()
 * @return true if successful, false otherwise
 */
bool initDisplay(esp_sleep_wakeup_cause_t wakeReason) {
    USBSerial.println("--- Initializing Display Hardware ---");
    
    if (!gfx->begin()) {
        USBSerial.println("ERROR: Display initialization failed");
        return false;
    }

    if (wakeReason == ESP_SLEEP_WAKEUP_EXT0) {
        USBSerial.println("Woke up from touch (Deep Sleep).");
    }

    gfx->fillScreen(BLACK);
    gfx->Display_Brightness(150);
    
    USBSerial.println("Display hardware initialization complete");
    return true;
}

/****************************************************************************************************
 * Initialize LVGL Graphics Library
 * Sets up display buffers, input devices, timers, and loads UI
 * @return true if successful, false otherwise
 */
bool initLVGL() {
    USBSerial.println("--- Initializing LVGL ---");
    
    // 1. Initialize the LVGL library itself
    lv_init();

    // 2. Initialize the display driver
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * screenHeight / 10);

    // Initialize the display
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.sw_rotate = 1;
    disp_drv.rotated = LV_DISP_ROT_90;
    lv_disp_t * disp = lv_disp_drv_register(&disp_drv);

    // Initialize the input device driver
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    // Create LVGL tick timer
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"
    };

    esp_timer_handle_t lvgl_tick_timer = NULL;
    if (esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer) != ESP_OK) {
        USBSerial.println("ERROR: Failed to create LVGL tick timer");
        return false;
    }
    
    if (esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000) != ESP_OK) {
        USBSerial.println("ERROR: Failed to start LVGL tick timer");
        return false;
    }

    // Load UI from SquareLine Studio
    ui_init();

    USBSerial.println("LVGL initialization complete");
    return true;
}

/****************************************************************************************************
 * Initialize UI Event Handlers and Components
 * Registers button callbacks and configures UI elements
 * @return true if successful
 */
bool initUIHandlers() {
    USBSerial.println("--- Initializing UI Event Handlers ---");
    
    // Register button event handlers
    lv_obj_add_event_cb(ui_ButtonLatest, buttonLatest_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_ButtonNew, buttonNew_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_ButtonBack, buttonBack_event_handler, LV_EVENT_CLICKED, NULL);
    USBSerial.println("  Button event handlers registered");

    // Attach Screen 2 event handler for image loading
    lv_obj_add_event_cb(ui_Screen2, screen2_event_handler, LV_EVENT_ALL, NULL);
    USBSerial.println("  Screen 2 event handler attached");

    // Initialize the Motion Icon Label
    lv_label_set_text(ui_labelMotionIcon, LV_SYMBOL_CHARGE);
    lv_obj_set_style_text_font(ui_labelMotionIcon, &lv_font_montserrat_24, 0);
    lv_obj_add_flag(ui_labelMotionIcon, LV_OBJ_FLAG_HIDDEN);  // Start hidden
    USBSerial.println("  Motion icon configured");

    USBSerial.println("UI event handlers initialization complete");
    return true;
}

/****************************************************************************************************
 * Check and initialize PSRAM
 * @return true if PSRAM available, false otherwise
 */
bool initPSRAM() {
    USBSerial.println("--- Checking PSRAM ---");
    
    if (psramFound()) {
        USBSerial.println("PSRAM found: " + String(ESP.getPsramSize() / 1024 / 1024) + "MB");
        return true;
    } else {
        USBSerial.println("FATAL: PSRAM not found - cannot continue");
        return false;
    }
}

/****************************************************************************************************
 * Initialize JPEG Decoder
 * @return true if successful
 */
bool initJPEGDecoder() {
    USBSerial.println("--- Initializing JPEG Decoder ---");
    
    // The GFX library expects RGB565 format (Big Endian)
    TJpgDec.setSwapBytes(false);
    
    USBSerial.println("JPEG decoder initialization complete");
    return true;
}

/****************************************************************************************************
 * Initialize IMU/Motion Sensor (QMI8658)
 * Configures accelerometer and Wake-on-Motion
 * @return true if successful, false otherwise
 */
bool initIMU() {
    USBSerial.println("--- Initializing IMU (QMI8658) ---");
    
    if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
        USBSerial.println("FATAL: Failed to find QMI8658 - check your wiring!");
        return false;
    }
    
    USBSerial.println("QMI8658 Initialized.");
    qmi.configureWoM();

    // Poll for initial motion state
    USBSerial.println("Getting initial motion state...");
    for (int i = 0; i < 5; i++) {
        updateMotionState(); // This will populate g_isCurrentlyMoving
        delay(20);
    }
    
    USBSerial.printf("Initial motion state: %s\n", 
                     g_isCurrentlyMoving ? "MOVING" : "STATIONARY");
    USBSerial.println("IMU initialization complete");
    return true;
}

/****************************************************************************************************
 * Initialize Battery Monitoring
 * Reads initial battery state and configures sleep policy
 * @return true if successful
 */
bool initBattery() {
    USBSerial.println("--- Initializing Battery Monitoring ---");
    
    // Get initial battery readings
    updateBatteryInfo();
    
    // Display initial state
    USBSerial.printf("Battery: %s%% (%.2fV)\n", 
                     batteryPercent.c_str(), 
                     batteryVoltage);
    USBSerial.printf("USB Power: %s\n", vbusPresent ? "CONNECTED" : "DISCONNECTED");
    USBSerial.printf("Battery: %s\n", batteryConnected ? "PRESENT" : "NOT DETECTED");
    
    // Set sleep policy based on initial power state
    allowSleep = !vbusPresent;
    if (allowSleep) {
        USBSerial.println("Starting on battery - sleep enabled after inactivity");
    } else {
        USBSerial.println("USB power detected - sleep disabled");
    }
    
    USBSerial.println("Battery monitoring initialization complete");
    return true;
}

/****************************************************************************************************
 * Update UI with initial sensor data
 * Must be called after battery and IMU initialization
 * @return true if successful
 */
bool updateInitialUI() {
    USBSerial.println("--- Updating Initial UI ---");
    
    // Update UI with all sensor data
    updateBatteryInfoUI();
    updateMotionStatusUI();
    
    // Force a complete screen refresh before WiFi connection
    USBSerial.println("Forcing full UI refresh before WiFi connection...");
    for (int i = 0; i < 15; i++) {
        lv_timer_handler();
        delay(5);
    }
    
    USBSerial.println("Initial UI update complete");
    return true;
}

/****************************************************************************************************
 * Configure WiFi Network Priority
 * Sets primary and secondary network based on WIFI_PRIORITY
 */
void configureWiFiPriority() {
    USBSerial.println("--- Configuring WiFi Priority ---");
    
    #if WIFI_PRIORITY == 1
        primarySsid = ssid1;
        primaryPassword = password1;
        primaryNetworkNum = 1;
        
        secondarySsid = ssid2;
        secondaryPassword = password2;
        secondaryNetworkNum = 2;
    #elif WIFI_PRIORITY == 2
        primarySsid = ssid2;
        primaryPassword = password2;
        primaryNetworkNum = 2;

        secondarySsid = ssid1;
        secondaryPassword = password1;
        secondaryNetworkNum = 1;
    #else
        #error "Invalid WIFI_PRIORITY defined. Please choose 1 or 2."
    #endif
    
    USBSerial.printf("Primary network: %d, Secondary network: %d\n", 
                     primaryNetworkNum, secondaryNetworkNum);
}

/****************************************************************************************************
 * Initialize WiFi Connection
 * Attempts connection with fallback to secondary network
 * 
 * IMPORTANT: The following must be done in setup() BEFORE calling this function:
 * 1. WiFi.mode(WIFI_OFF) - to disable WiFi radio during hardware init
 * 2. configureWiFiPriority() - to set primary/secondary network variables
 * 
 * @return true if successful, false otherwise
 */
bool initWiFi() {
    USBSerial.println("--- Initializing WiFi ---");
    
    // Attempt connection (network priority was already configured in setup)
    if (!attemptWiFiConnection()) {
        USBSerial.println("WiFi connection failed (unexpected state).");
        return false;
    }
    
    USBSerial.println("WiFi connection established successfully.");
    
    // Allow network stack to stabilize
    USBSerial.println("Allowing network stack to stabilize...");
    for (int i = 0; i < 10; i++) {
        updateMotionState();
        updateMotionStatusUI();
        lv_timer_handler();
        delay(200);
    }
    
    USBSerial.println("WiFi initialization complete");
    return true;
}

/****************************************************************************************************
 * Initialize MQTT Connection
 * Attempts initial connection with retry logic
 * @return true if successful, false if failed (non-fatal)
 */
bool initMQTT() {
    USBSerial.println("--- Initializing MQTT ---");
    
    if (WiFi.status() != WL_CONNECTED) {
        USBSerial.println("WARNING: Cannot initialize MQTT - WiFi not connected");
        return false;
    }
    
    USBSerial.println("Attempting initial MQTT connection...");
    
    for (int i = 0; i < 3; i++) {
        checkMQTT(true);  // Bypass rate limiting during setup
        
        if (mqttClient.connected()) {
            USBSerial.println("Initial MQTT connection successful!");
            return true;
        }
        
        USBSerial.print("MQTT attempt ");
        USBSerial.print(i + 1);
        USBSerial.println(" failed, retrying...");
        
        if (i < 2) {  // Don't delay after last attempt
            // 3 second delay between attempts
            for (int j = 0; j < 15; j++) {
                updateMotionState();
                updateMotionStatusUI();
                lv_timer_handler();
                delay(200);
            }
        }
    }
    
    USBSerial.println("Initial MQTT connection failed - will retry in loop");
    return false;  // Non-fatal - will retry in loop
}

/****************************************************************************************************
 * Finalize Setup
 * Updates UI with final status and prepares for main loop
 */
void finalizeSetup() {
    USBSerial.println("--- Finalizing Setup ---");
    
    // Update connection status UI
    updateConnectionStatusUI();
    
    // Final UI refresh
    for (int i = 0; i < 5; i++) { 
        lv_timer_handler(); 
        delay(5); 
    }
    
    // Set initial activity timestamp
    lastActivityTime = millis();
    
    USBSerial.println("--- Setup complete, entering loop ---\n");
}


//***************************************************************************************************
// MAIN SETUP FUNCTION (CORRECTED ORDER)
//***************************************************************************************************
void setup() {
    // === EARLY INITIALIZATION (must happen first) ===
    
    // 1. Initialize serial
    USBSerial.begin(115200);
    
    // 2. Handle wake reason and I2C reset if needed
    esp_sleep_wakeup_cause_t wakeReason = esp_sleep_get_wakeup_cause();

    if (wakeReason == ESP_SLEEP_WAKEUP_EXT0) {
        // This is a Deep Sleep Wake. The ESP32's I2C driver is stuck.
        // We MUST reset it to prevent the "i2c driver install error".
        USBSerial.println("Deep Sleep Wake detected. Performing I2C driver reset...");
        Wire.end();
        delay(10);
    }
    // For any other type of boot (Cold or Shutdown), we DO NOT call Wire.end().
    
    // 3. Initialize I2C bus
    Wire.begin(IIC_SDA, IIC_SCL);
    delay(50);
    
    USBSerial.println("\n--- Board is starting up ---");
    
    // 4. CRITICAL: Disable WiFi BEFORE hardware initialization
    //    This prevents interference with I2C bus and PMIC
    WiFi.mode(WIFI_OFF);
    
    // === HARDWARE INITIALIZATION ===
    
    // Initialize PMIC - critical for power management
    initPMIC();  // Don't check return value, just like original
    
    // Initialize I/O Expander - needed for display control
    if (!initIOExpander()) {
        USBSerial.println("FATAL: I/O Expander initialization failed");
        while(1) { delay(1000); }
    }
    
    // Initialize Touch Controller
    initTouch();  // Don't check return value - infinite retry inside
    
    // Initialize Display Hardware
    if (!initDisplay(wakeReason)) {
        USBSerial.println("FATAL: Display initialization failed");
        while(1) { delay(1000); }
    }
    
    // Initialize LVGL
    if (!initLVGL()) {
        USBSerial.println("FATAL: LVGL initialization failed");
        while(1) { delay(1000); }
    }
    
    // Initialize UI Event Handlers
    if (!initUIHandlers()) {
        USBSerial.println("WARNING: UI handlers initialization failed");
    }
    
    // Check PSRAM availability
    if (!initPSRAM()) {
        USBSerial.println("FATAL: PSRAM not available - cannot continue");
        while(1) { delay(1000); }
    }
    
    // Initialize JPEG Decoder
    initJPEGDecoder();
    
    // Initialize IMU/Motion Sensor
    if (!initIMU()) {
        USBSerial.println("FATAL: IMU initialization failed");
        while(1) { delay(1000); }
    }
    
    // Initialize Battery Monitoring
    if (!initBattery()) {
        USBSerial.println("WARNING: Battery monitoring initialization failed");
    }
    
    // Update UI with initial sensor data
    updateInitialUI();
    
    // === NETWORK INITIALIZATION ===
    
    // Configure WiFi priority (must be done before initWiFi)
    configureWiFiPriority();
    
    // Initialize WiFi (failure handling in attemptWiFiConnection)
    initWiFi();
    
    // Initialize MQTT (will retry in loop if needed)
    initMQTT();
    
    // === FINALIZATION ===
    
    // Finalize setup
    finalizeSetup();
}


//***************************************************************************************************
void loop() {
  static bool shutdownInitiated = false;

  // --- GATEKEEPER: Check for shutdown FIRST ---
  if (shutdownInitiated) {
    delay(1);
    return; // Exit the loop immediately.
  }

  // If we are not shutting down, proceed with normal operations.
  lv_timer_handler();

  // --- Task 1: Update motion state ---
  updateMotionState(); // Just update the motion flag, no shutdown decision
  updateMotionStatusUI(); // Update the UI icon

  // --- Task 2: Handle MQTT communications if connected ---
  if (WiFi.status() == WL_CONNECTED) {
    mqttClient.loop();  // Always call loop() to maintain connection
    if(mqttSuccess) {
      checkMQTT();  // Only attempt reconnection if initial connection succeeded
    }
  }

  // --- Task 3: Process HTTP response if in progress
  processHTTPResponse();

  // --- Task 4 (Timed): Update Battery Info and UI ---
  if (millis() - lastBatteryUpdate >= 200) {
    lastBatteryUpdate = millis();
    updateBatteryInfo();   // 1. Get the latest hardware data.
    updateMotionStatusUI(); // 2. Update the motion icon.
    updateBatteryInfoUI(); // 3. Update the LVGL labels.
  }

  // --- Task 5 (Timed): Update Connection Status UI ---
  if (millis() - lastConnectionUpdate >= 1000) { // Check every second
    lastConnectionUpdate = millis();
    updateConnectionStatusUI();
  }

  // --- TASK 6: Check Screen 2 Timeout ---
  if (lv_scr_act() == ui_Screen2) {
    // Check loading timeout (8 seconds)
    if (screen2TimeoutActive && millis() - screenTransitionTime > SCREEN2_LOADING_TIMEOUT) {
      USBSerial.println("Screen 2 timeout - image loading took too long, returning to Screen 1");
      
      // Clean up any ongoing HTTP request
      if (httpState != HTTP_IDLE && httpState != HTTP_COMPLETE) {
        httpClient.end();
        if (jpeg_buffer_psram) {
          free(jpeg_buffer_psram);
          jpeg_buffer_psram = nullptr;
        }
        httpState = HTTP_ERROR;  // Will be reset to IDLE when leaving Screen 2
      }
      
      // Reset flags
      requestInProgress = false;
      screen2TimeoutActive = false;
      imageDisplayTimeoutActive = false;
      
      // Return to Screen 1
      lv_disp_load_scr(ui_Screen1);
    }

    // Check display timeout (1 minute after image loads)
    if (imageDisplayTimeoutActive) {
      unsigned long elapsed = millis() - imageDisplayStartTime;
      if (elapsed > SCREEN2_DISPLAY_TIMEOUT) {
        USBSerial.println("Screen 2 display timeout - 1 minute elapsed, returning to Screen 1");
        
        // Reset flags
        requestInProgress = false;
        screen2TimeoutActive = false;
        imageDisplayTimeoutActive = false;
        
        // Return to Screen 1
        lv_disp_load_scr(ui_Screen1);
      }
    }
  }

  // --- Task 7: Transmit motion MQTT if connected --- 
  if (ENABLE_MOTION_MQTT && g_isCurrentlyMoving && mqttClient.connected()) {
    if (millis() - lastMotionTXTime > MOTION_TIMEOUT) {
      mqttClient.publish(MOTION_TOPIC, "1");
      lastMotionTXTime = millis();
      USBSerial.println("TX motion MQTT: Moving (periodic)");
    }
  }

  // --- Task 8: Check for user inactivity to trigger deep sleep if allowed to sleep ---
  if (millis() - lastActivityTime > INACTIVITY_TIMEOUT && allowSleep) {
      
      if (usbWasEverPresent && !vbusPresent) {
          // USB was present and now lost - apply grace period logic
          if (!g_isCurrentlyMoving) {
              unsigned long timeSinceUsbLoss = millis() - usbDisconnectedTime;
              if (timeSinceUsbLoss > USB_GRACE_PERIOD) {
                  // Grace period expired, no touch, and stationary - shutdown
                  shutdownInitiated = true;
                  goToShutdown();
              }
              // If grace period not expired yet, do nothing (stay awake)
          }
          // If still moving, do nothing (stay awake)
          
      } else {
          // Normal battery-only operation (USB never connected)
          // Check motion state to decide sleep vs shutdown
          if (!g_isCurrentlyMoving) {
              // No touch for 30s AND stationary → SHUTDOWN to save max power
              shutdownInitiated = true;
              goToShutdown();
          } else {
              // No touch for 30s BUT still moving → SLEEP for quick wake
              goToDeepSleep();
          }
      }
  }

    // Smart delay based on system state
    if (httpState == HTTP_RECEIVING || httpState == HTTP_REQUESTING) {
        delay(1);  // Fast response during downloads
    } else if (lv_scr_act() == ui_Screen2 && imageDisplayTimeoutActive) {
        delay(5);  // Moderate when displaying image
    } else {
        delay(10); // Longer delay when idle saves more power
    }
}