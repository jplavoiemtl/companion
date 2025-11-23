#include "calibration.h"
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

SensorQMI8658 qmi;

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
#define HTTP_TIMEOUT_MS 15000  // 6000
#define MAX_JPEG_SIZE 60000  // 60KB should be plenty for 368x448 JPEG
// MQTT Topics
#define MQTT_IMAGE_TOPIC "esp32image"
// =================================================
const char HILO_POWER[] = "ha/hilo_meter_power";
const char HILO_ENERGY[] = "hilo_energie";
const char MOTION_TOPIC[] = "companion/motion";
const char IMU_TOPIC[] = "companion/imu";
const char IMU_CALIBRATION_TOPIC[] = "companion/calibration";

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

// --- IMU Management and motion detection ---
const unsigned long IMU_PRINT_INTERVAL = 5000;  // Print & send MQTT IMU data every 5s=5000
unsigned long lastImuPrintTime = 0;
struct {
  float x, y, z;
} acc, gyr;

const float ACCEL_MOTION_THRESHOLD = 0.04;  // Adjust sensitivity (m/s²) 0.05 
bool motionDetected = false;
float lastAccelMagnitude = 0;
const float GYRO_MOTION_THRESHOLD = 4.7;  // 4.5 trigger while immobile, may have to increase to avoid false positives
float lastGyroMagnitude = 0;

// Peak magnitude tracking for IMU data (2-second MQTT interval)
struct {
  float x, y, z;
  float magnitude;
} acc_peak = {0.0, 0.0, 0.0, 0.0}, gyr_peak = {0.0, 0.0, 0.0, 0.0};

// Peak tracking for display updates (100ms interval)
struct {
  float x, y, z;
  float magnitude;
} acc_disp_peak = {0.0, 0.0, 0.0, 0.0};

struct {
  float x, y, z;
  float magnitude;
} gyr_disp_peak = {0.0, 0.0, 0.0, 0.0};

// Inertial display coordinates (transformed via rotation matrix)
struct {
  float vert;   // Vertical (display X - negative = down/accel, positive = up/brake)
  float horiz;  // Horizontal (display Y - negative = turn left/dot right, positive = turn right/dot left)
  float up;     // Up reference (display Z - should stay ~1g)
} acc_inertial = {0.0, 0.0, 0.0};

struct {
  float vert;
  float horiz;
  float up;
} gyr_inertial = {0.0, 0.0, 0.0};

bool imuPeakInitialized = false;
// Current change values for motion detection
float currentAccelChange = 0;
float currentGyroChange = 0;
// Peak change values during interval
float peakAccelChange = 0;
float peakGyroChange = 0;

// Gyro bias in SENSOR coordinates (before rotation matrix)
float gyro_bias_sensor_x = 0.0;
float gyro_bias_sensor_y = 0.0;
float gyro_bias_sensor_z = 0.0;

// G-Meter Display Constants
#define WIDTH_DISPLAY 448
#define HEIGHT_DISPLAY 368
#define G_SCALE_DISPLAY 0.4  // Max G-force shown at edge of circle 0.4 G
// Calculate number of tick marks based on G_SCALE_DISPLAY
#define TICK_INTERVAL 0.1
#define NUM_TICKS_PER_AXIS ((int)(G_SCALE_DISPLAY / TICK_INTERVAL))  // 0.4 / 0.1 = 4
#define TOTAL_TICK_MARKS (NUM_TICKS_PER_AXIS * 4)  // 4 directions × ticks per direction = 16

// G-Meter Color Constants - https://www.colorhexa.com/
#define GMETER_CIRCLE_COLOR 0xFF7F00   // Orange circle outline 0xFF8000
#define GMETER_AXIS_COLOR   0x0080FF  
#define GMETER_TICK_COLOR   0x00FF00  
#define GMETER_DOT_COLOR    0xFF0000   // Red main dot
#define GMETER_TRAIL_COLOR  0xFF0000   // Trail dots 0xFF0000 0xB8B800
#define GMETER_CIRCLE_FILL  0x300000   // Dark red fill 0x200000
#define GMETER_INNER_CIRCLE_COLOR 0x00FF00 // Green inner circle outline 0x00FF00
#define GMETER_INNER_CIRCLE_FILL 0x003200   // Green inner circle fill 0x003200

// Global variables for time tracking complementary filter
static unsigned long previous_time = 0;
unsigned long current_time = 0;
float dt = 0.0;
float sampling_frequency = 0.0; // Variable to hold the sampling frequency in Hz

// ========== INCLINOMETER STATE VARIABLES ==========
float pitch_angle = 0.0;              // Current pitch angle in degrees
float roll_angle = 0.0;               // Current roll angle in degrees
unsigned long last_inclinometer_update = 0;  // Timestamp for dt calculation
bool inclinometer_initialized = false; // Flag to track initialization state
// Gyro bias values (to be determined during calibration)
float gyro_bias_vert = 0.0;
float gyro_bias_horiz = 0.0;
float gyro_bias_up = 0.0;
unsigned long last_inclinometer_display_update = 0;
const unsigned long INCLINOMETER_DISPLAY_INTERVAL = 500; // 200ms = 5 Hz, 500

// --- Filter/estimator state (add these near other globals) ---
float pitch_gyro = 0.0f;   // gyro-only integrated pitch (degrees)
float roll_gyro  = 0.0f;   // gyro-only integrated roll  (degrees)

// LPF state for accelerometer components (in inertial car frame)
float acc_lp_vert = 0.0f;
float acc_lp_horiz = 0.0f;
float acc_lp_up = 0.0f;

// Confidence flags exposed from motion detection
float accel_magnitude = 1.0f;   // instantaneous |a|
float gyro_magnitude = 0.0f;    // instantaneous |omega|
const float BIAS_LEARN_BETA = 0.001f;   // bias IIR gain when stationary (0.0005 - 0.005)

float last_effective_tau = 0.0f;
float last_alpha = 0.0f;

// ---------------- TURN-FREEZE STATE ----------------

// Whether we are currently freezing angle updates
bool turn_freeze_active = false;

// When did we enter/exit turn-freeze?
unsigned long turn_freeze_start_ms = 0;
unsigned long turn_freeze_exit_ms = 0;

// Last stable angles before entering freeze
float frozen_pitch = 0.0f;
float frozen_roll  = 0.0f;

// For smooth recovery
bool recovering_from_turn = false;
unsigned long recover_start_ms = 0;
const unsigned long RECOVER_DELAY_MS = 500;   // wait after turn ends
const unsigned long RECOVER_BLEND_MS = 1200;  // blend back duration


// --- Global Objects ---
HWCDC USBSerial;
XPowersAXP2101 pmic;
ESP_IOExpander *expander = NULL;

// --- G-Meter Display Objects ---
lv_obj_t * ui_gMeterContainer = NULL; // Container for G-meter elements avoid leaking memory
lv_obj_t * ui_gMeterCircle = NULL;   // Green outline circle
lv_obj_t * ui_gMeterInnerCircle = NULL;  // White inner circle at half radius
lv_obj_t * ui_gMeterAxisV = NULL;    // Vertical green line
lv_obj_t * ui_gMeterAxisH = NULL;    // Horizontal green line
lv_obj_t * ui_gMeterDot = NULL;      // Red moving dot

lv_obj_t * ui_gMeterTicks[TOTAL_TICK_MARKS] = {NULL};  // Tick marks array

// Trail dots (5 additional dots behind current position)
lv_obj_t * ui_gMeterTrail[5] = {NULL, NULL, NULL, NULL, NULL};

// Trail position history
#define TRAIL_LENGTH 5
int trail_pos_x[TRAIL_LENGTH] = {0, 0, 0, 0, 0};
int trail_pos_y[TRAIL_LENGTH] = {0, 0, 0, 0, 0};

// Trail opacity levels (100% to 10%)
const int trail_opacity[TRAIL_LENGTH] = {80, 60, 40, 25, 10};

// Stationary detection
unsigned long last_position_change_time = 0;
int last_x_display = 0;
int last_y_display = 0;

// --- Global State Variables (shared between functions) ---
bool adc_switch = false;
String batteryPercent = "";
float batteryVoltage = 0.0;
bool batteryConnected = false;
bool vbusPresent = false;
const unsigned long MOTION_CHECK_INTERVAL = 20;  // Motion state update interval (10ms) 10
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
       
        // Configure the secure client with the now-verified remote server's CA certificate
        httpsClient.setCACert(remote_server_ca_cert);

        // Begin HTTPS connection using the secure client
        bool beginResult = httpClient.begin(httpsClient, url);
        
        if (!beginResult) {
            USBSerial.println("FATAL: httpClient.begin() failed for HTTPS!");
            httpState = HTTP_ERROR;
            return false;
        }
        USBSerial.println("HTTPS begin successful");

    } else {
        // === HTTP Connection for local access (ssid1) ===
        url = String(IMAGE_SERVER_BASE) + String(endpoint_type);
        USBSerial.println("Initiating HTTP GET: " + url);
       
        // Begin standard HTTP connection
        bool beginResult = httpClient.begin(url);
        
        // ADD THIS:
        if (!beginResult) {
            USBSerial.println("FATAL: httpClient.begin() failed for HTTP!");
            httpState = HTTP_ERROR;
            return false;
        }
        USBSerial.println("HTTP begin successful");
    }
    // --- END OF DYNAMIC CONFIGURATION ---


    // Set increased timeouts for cellular network stability
    httpClient.setConnectTimeout(HTTP_TIMEOUT_MS);
    httpClient.setTimeout(HTTP_TIMEOUT_MS);
    
    int httpCode = httpClient.GET();
    
    if (httpCode != HTTP_CODE_OK) {
        USBSerial.printf("FATAL: HTTP GET failed with code: %d\n", httpCode);  // ENHANCED                                  // NEW
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
    
    jpeg_buffer_psram = (uint8_t*)ps_malloc(contentLength);
    if (!jpeg_buffer_psram) {
        USBSerial.println("FATAL: Failed to allocate PSRAM for JPEG buffer");
        USBSerial.printf("Requested size: %d bytes\n", contentLength);
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
    
    image_buffer_psram = (uint16_t*)ps_malloc(imageBufferSize);
    
    if (!image_buffer_psram) {
      USBSerial.println("FATAL: PSRAM allocation failed for decoded image buffer");
      USBSerial.printf("Requested size: %d bytes\n", imageBufferSize);
      
      free(jpeg_buffer_psram);
      jpeg_buffer_psram = nullptr;
      httpClient.end();
      httpState = HTTP_ERROR;
      return;
    }

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
    
  // Only respond if we're on Screen1 or Screen3 or inclinometer (prevent unexpected transitions)
    lv_obj_t * current_screen = lv_scr_act();
    if (current_screen != ui_Screen1 && current_screen != ui_Screen3 && current_screen != ui_InclinometerScreen) {
        USBSerial.println("Not on Screen1 or Screen3 or inclinometer, ignoring image request");
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
// Button event handler for G-meter display
void buttonGmeter_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    
    if (code == LV_EVENT_CLICKED) {
        USBSerial.println("G-meter button clicked");

        // Don't reload if already on Screen3
        if (lv_scr_act() == ui_Screen3) {
            USBSerial.println("Already on Screen3, ignoring button click");
            return;
        }        

        // Load Screen3
        lv_disp_load_scr(ui_Screen3);        
        
        // Prevent multiple creation - check if objects already exist
        if (ui_gMeterCircle != NULL) {
            USBSerial.println("G-meter objects already exist, skipping creation");
            return;
        }         
        
        // --- Create the master container object ---
        ui_gMeterContainer = lv_obj_create(ui_Screen3);
        // Make the container transparent and non-interactive, covering the whole screen
        lv_obj_remove_style_all(ui_gMeterContainer);
        lv_obj_set_size(ui_gMeterContainer, lv_pct(100), lv_pct(100));
        lv_obj_clear_flag(ui_gMeterContainer, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

        // Move the container to the background layer, so it doesn't block buttons.
        lv_obj_move_background(ui_gMeterContainer);        

        // Create G-meter display elements
        // Calculate center position and circle dimensions
        int center_x = WIDTH_DISPLAY / 2;   // 224
        int center_y = HEIGHT_DISPLAY / 2;  // 184
        int radius = HEIGHT_DISPLAY / 2;    // 184 (circle diameter = 368)
        int circle_left = (WIDTH_DISPLAY - HEIGHT_DISPLAY) / 2;  // 40
        int circle_right = circle_left + HEIGHT_DISPLAY;         // 408
        
        // 1. Create circle outline
        ui_gMeterCircle = lv_obj_create(ui_gMeterContainer); // Create inside the container
        lv_obj_set_size(ui_gMeterCircle, radius * 2, radius * 2);
        lv_obj_set_pos(ui_gMeterCircle, center_x - radius, center_y - radius);
        lv_obj_set_style_radius(ui_gMeterCircle, LV_RADIUS_CIRCLE, LV_PART_MAIN);
        lv_obj_set_style_bg_color(ui_gMeterCircle, lv_color_hex(GMETER_CIRCLE_FILL), LV_PART_MAIN);  
        lv_obj_set_style_bg_opa(ui_gMeterCircle, LV_OPA_COVER, LV_PART_MAIN);  // Opaque fill
        lv_obj_set_style_border_color(ui_gMeterCircle, lv_color_hex(GMETER_CIRCLE_COLOR), LV_PART_MAIN);  
        lv_obj_set_style_border_width(ui_gMeterCircle, 4, LV_PART_MAIN);
        lv_obj_set_style_border_opa(ui_gMeterCircle, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_clear_flag(ui_gMeterCircle, LV_OBJ_FLAG_SCROLLABLE);

        // 1.5. Create white inner circle at half radius
        int inner_radius = radius / 2;  // Half the main radius (92 pixels)
        ui_gMeterInnerCircle = lv_obj_create(ui_gMeterContainer);
        lv_obj_set_size(ui_gMeterInnerCircle, inner_radius * 2, inner_radius * 2);
        lv_obj_set_pos(ui_gMeterInnerCircle, center_x - inner_radius, center_y - inner_radius);
        lv_obj_set_style_radius(ui_gMeterInnerCircle, LV_RADIUS_CIRCLE, LV_PART_MAIN);
        lv_obj_set_style_bg_color(ui_gMeterInnerCircle, lv_color_hex(GMETER_INNER_CIRCLE_FILL), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(ui_gMeterInnerCircle, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_color(ui_gMeterInnerCircle, lv_color_hex(GMETER_INNER_CIRCLE_COLOR), LV_PART_MAIN);  // White
        lv_obj_set_style_border_width(ui_gMeterInnerCircle, 2, LV_PART_MAIN);
        lv_obj_set_style_border_opa(ui_gMeterInnerCircle, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_clear_flag(ui_gMeterInnerCircle, LV_OBJ_FLAG_SCROLLABLE);        
        
        // 2. Create vertical axis (green line) - full height inside circle
        ui_gMeterAxisV = lv_line_create(ui_gMeterContainer); // Create inside the container
        static lv_point_t line_points_v[] = {{center_x, 0}, {center_x, HEIGHT_DISPLAY}};
        lv_line_set_points(ui_gMeterAxisV, line_points_v, 2);
        lv_obj_set_style_line_color(ui_gMeterAxisV, lv_color_hex(GMETER_AXIS_COLOR), LV_PART_MAIN);  // Green
        lv_obj_set_style_line_width(ui_gMeterAxisV, 2, LV_PART_MAIN);
        
        // 3. Create horizontal axis (green line) - constrained to circle width
        ui_gMeterAxisH = lv_line_create(ui_gMeterContainer); // Create inside the container
        static lv_point_t line_points_h[] = {{circle_left, center_y}, {circle_right, center_y}};
        lv_line_set_points(ui_gMeterAxisH, line_points_h, 2);
        lv_obj_set_style_line_color(ui_gMeterAxisH, lv_color_hex(GMETER_AXIS_COLOR), LV_PART_MAIN);  // Green
        lv_obj_set_style_line_width(ui_gMeterAxisH, 2, LV_PART_MAIN);
        
        // 4. Create tick marks at 0.1g intervals
        float g_per_tick = TICK_INTERVAL;
        float pixels_per_g = radius / G_SCALE_DISPLAY;
        int tick_spacing = (int)(pixels_per_g * g_per_tick);
        int tick_length = 15;
        
        int tick_idx = 0;
        USBSerial.printf("Starting tick creation, array size: 16\n");
        
        // Create tick marks on vertical axis (horizontal lines)
        for (int i = 1; i <= NUM_TICKS_PER_AXIS; i++) {
            int offset = i * tick_spacing;
            
            // Tick above center (negative y, braking)
            if (tick_idx < TOTAL_TICK_MARKS) {  // SAFETY CHECK
                lv_obj_t * tick_up = lv_line_create(ui_gMeterContainer);
                static lv_point_t tick_points_up[NUM_TICKS_PER_AXIS][2];  
                tick_points_up[i-1][0].x = center_x - tick_length / 2;
                tick_points_up[i-1][0].y = center_y - offset;
                tick_points_up[i-1][1].x = center_x + tick_length / 2;
                tick_points_up[i-1][1].y = center_y - offset;
                lv_line_set_points(tick_up, tick_points_up[i-1], 2);
                lv_obj_set_style_line_color(tick_up, lv_color_hex(GMETER_TICK_COLOR), LV_PART_MAIN);
                lv_obj_set_style_line_width(tick_up, 1, LV_PART_MAIN);
                
                ui_gMeterTicks[tick_idx] = tick_up;
                tick_idx++;
            }
            
            // Tick below center (positive y, accelerating)
            if (tick_idx < TOTAL_TICK_MARKS) {  // SAFETY CHECK
                lv_obj_t * tick_down = lv_line_create(ui_gMeterContainer);
                static lv_point_t tick_points_down[NUM_TICKS_PER_AXIS][2];  // CHANGED from [6] to [4]
                tick_points_down[i-1][0].x = center_x - tick_length / 2;
                tick_points_down[i-1][0].y = center_y + offset;
                tick_points_down[i-1][1].x = center_x + tick_length / 2;
                tick_points_down[i-1][1].y = center_y + offset;
                lv_line_set_points(tick_down, tick_points_down[i-1], 2);
                lv_obj_set_style_line_color(tick_down, lv_color_hex(GMETER_TICK_COLOR), LV_PART_MAIN);
                lv_obj_set_style_line_width(tick_down, 1, LV_PART_MAIN);
                
                ui_gMeterTicks[tick_idx] = tick_down;
                tick_idx++;
            }
        }
        
        // Create tick marks on horizontal axis (vertical lines)
        for (int i = 1; i <= NUM_TICKS_PER_AXIS; i++) {
            int offset = i * tick_spacing;
            
            // Tick left of center (negative x, right turn)
            if (tick_idx < TOTAL_TICK_MARKS) {  // SAFETY CHECK
                lv_obj_t * tick_left = lv_line_create(ui_gMeterContainer);
                static lv_point_t tick_points_left[NUM_TICKS_PER_AXIS][2];  // CHANGED from [6] to [4]
                tick_points_left[i-1][0].x = center_x - offset;
                tick_points_left[i-1][0].y = center_y - tick_length / 2;
                tick_points_left[i-1][1].x = center_x - offset;
                tick_points_left[i-1][1].y = center_y + tick_length / 2;
                lv_line_set_points(tick_left, tick_points_left[i-1], 2);
                lv_obj_set_style_line_color(tick_left, lv_color_hex(GMETER_TICK_COLOR), LV_PART_MAIN);
                lv_obj_set_style_line_width(tick_left, 1, LV_PART_MAIN);
                
                ui_gMeterTicks[tick_idx] = tick_left;
                tick_idx++;
            }
            
            // Tick right of center (positive x, left turn)
            if (tick_idx < TOTAL_TICK_MARKS) {  // SAFETY CHECK
                lv_obj_t * tick_right = lv_line_create(ui_gMeterContainer);
                static lv_point_t tick_points_right[NUM_TICKS_PER_AXIS][2];  // CHANGED from [6] to [4]
                tick_points_right[i-1][0].x = center_x + offset;
                tick_points_right[i-1][0].y = center_y - tick_length / 2;
                tick_points_right[i-1][1].x = center_x + offset;
                tick_points_right[i-1][1].y = center_y + tick_length / 2;
                lv_line_set_points(tick_right, tick_points_right[i-1], 2);
                lv_obj_set_style_line_color(tick_right, lv_color_hex(GMETER_TICK_COLOR), LV_PART_MAIN);
                lv_obj_set_style_line_width(tick_right, 1, LV_PART_MAIN);
                
                ui_gMeterTicks[tick_idx] = tick_right;
                tick_idx++;
            }
        }

        // 5. Create red dot (initially at center) - INCREASED SIZE TO 30px
        ui_gMeterDot = lv_obj_create(ui_gMeterContainer); // Create inside the container
        lv_obj_set_size(ui_gMeterDot, 30, 30);  // Increased from 8 to 30
        lv_obj_set_pos(ui_gMeterDot, center_x - 15, center_y - 15);  // Center the 30px dot
        lv_obj_set_style_radius(ui_gMeterDot, LV_RADIUS_CIRCLE, LV_PART_MAIN);
        lv_obj_set_style_bg_color(ui_gMeterDot, lv_color_hex(GMETER_DOT_COLOR), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(ui_gMeterDot, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_width(ui_gMeterDot, 0, LV_PART_MAIN);  // No border
        lv_obj_clear_flag(ui_gMeterDot, LV_OBJ_FLAG_SCROLLABLE);
        
        // This is to ensure the G-meter elements are at the back of the screen stack and 
        // do not obscure other UI elements like buttons.
        lv_obj_move_background(ui_gMeterCircle);
        lv_obj_move_foreground(ui_gMeterInnerCircle);
        lv_obj_move_foreground(ui_gMeterAxisV);
        lv_obj_move_foreground(ui_gMeterAxisH);

        // Move all tick marks to foreground (above inner circle)
        for (int i = 0; i < tick_idx; i++) {  // Use tick_idx instead of hardcoded 16
            if (ui_gMeterTicks[i] != NULL) {  // NULL check
                lv_obj_move_foreground(ui_gMeterTicks[i]);
            } 
        }

        lv_obj_move_foreground(ui_gMeterDot);  // Ensure dot is in foreground

        // Create 5 trail dots
        for (int i = 0; i < TRAIL_LENGTH; i++) {
            ui_gMeterTrail[i] = lv_obj_create(ui_gMeterContainer); // Create inside the container
            lv_obj_set_size(ui_gMeterTrail[i], 20, 20);  // Smaller than main dot
            lv_obj_set_pos(ui_gMeterTrail[i], center_x - 10, center_y - 10);  // Start at center
            lv_obj_set_style_radius(ui_gMeterTrail[i], LV_RADIUS_CIRCLE, LV_PART_MAIN);
            lv_obj_set_style_bg_color(ui_gMeterTrail[i], lv_color_hex(GMETER_TRAIL_COLOR), LV_PART_MAIN);
            lv_obj_set_style_bg_opa(ui_gMeterTrail[i], LV_OPA_TRANSP, LV_PART_MAIN);  // Start invisible
            lv_obj_set_style_border_width(ui_gMeterTrail[i], 0, LV_PART_MAIN);
            lv_obj_clear_flag(ui_gMeterTrail[i], LV_OBJ_FLAG_SCROLLABLE);
            lv_obj_clear_flag(ui_gMeterTrail[i], LV_OBJ_FLAG_CLICKABLE);
            
            lv_obj_move_foreground(ui_gMeterTrail[i]);
            
            // Initialize position history
            trail_pos_x[i] = center_x;
            trail_pos_y[i] = center_y;
        }
        
        if (ui_Gscale != NULL) {
            char gscale_text[16];  // Buffer for formatted text
            snprintf(gscale_text, sizeof(gscale_text), "%.1f G", G_SCALE_DISPLAY);
            lv_label_set_text(ui_Gscale, gscale_text);
        }

        // Initialize stationary tracking
        last_x_display = center_x;
        last_y_display = center_y;
        last_position_change_time = millis();        

        USBSerial.println("G-meter display elements created");
        USBSerial.printf("Free heap: %d bytes, Free PSRAM: %d bytes\n", 
                 ESP.getFreeHeap(), ESP.getFreePsram());
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
// Event handler for Screen 3 (G-meter display)
void screen3_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_SCREEN_UNLOAD_START) {
        USBSerial.println("Screen 3 Unloading: Cleaning up G-meter objects and resetting pointers");
        
        // Delete the single parent container, which will delete all children
        if (ui_gMeterContainer != NULL) {
            lv_obj_del(ui_gMeterContainer);
        }
        
        // Set ALL associated global pointers to NULL to prevent dangling pointer crashes.
        ui_gMeterContainer = NULL;
        ui_gMeterCircle = NULL;
        ui_gMeterInnerCircle = NULL;
        ui_gMeterAxisV = NULL;
        ui_gMeterAxisH = NULL;
        ui_gMeterDot = NULL;

        for (int i = 0; i < TRAIL_LENGTH; i++) {
            ui_gMeterTrail[i] = NULL;
        }
        // Reset tick mark pointers
        for (int i = 0; i < TOTAL_TICK_MARKS; i++) {
            ui_gMeterTicks[i] = NULL;
        }
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
        //USBSerial.print("Power: ");
        //USBSerial.println(payloadString);
        updatePowerDisplay(payloadString);
    }
    // Energy handling
    else if (topicString == HILO_ENERGY) {
        //USBSerial.print("Energy: ");
        //USBSerial.println(payloadString);
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
      
      // Send Calibration Report on Connect
      calibReportStatus();           
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
// TIMER CALLBACK
// ===========================================================
void onGravityTimerExpired(lv_timer_t * timer) {
    // Retrieve the pointer to the 'isStabilizing' flag
    bool* pIsStabilizing = (bool*)timer->user_data;
    
    // Reset the flag
    if (pIsStabilizing) {
        *pIsStabilizing = false;
    }
    
    calibStartGravity();
    USBSerial.println("[UI] Stabilization complete. Sampling started.");
}

// ===========================================================
// BUTTON EVENT HANDLER
// ===========================================================
void gravityCalButton_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);

    // Static variable INSIDE the function.
    // It keeps its value between button clicks.
    static bool isStabilizing = false;

    if (code == LV_EVENT_CLICKED) {
        // 1. DEBOUNCE CHECK
        if (isStabilizing) {
            USBSerial.println("[UI] Ignored double-click.");
            return;
        }

        // 2. STATE CHECK
        // Only block if we are actively doing something.
        // It is OK to click if state is IDLE, DONE, or ERROR.
        CalibState cs = calibGetState();
        if (cs == CALIB_GRAVITY_SAMPLING || 
            cs == CALIB_FORWARD_SAMPLING || 
            cs == CALIB_READY_TO_COMPUTE) {
            USBSerial.println("[UI] Ignored click (Calibration in progress).");
            return;
        }

        // 3. Lock the button
        isStabilizing = true;
        
        USBSerial.println("[UI] Gravity Calib Button Pressed.");
        USBSerial.println("[UI] Waiting 1s for stabilization...");

        // Create timer and PASS THE ADDRESS of our static variable
        lv_timer_t * timer = lv_timer_create(onGravityTimerExpired, 1000, &isStabilizing);
        lv_timer_set_repeat_count(timer, 1);
    }
}


//***************************************************************************************************
// FORWARD TIMER CALLBACK
// ===========================================================
void onForwardTimerExpired(lv_timer_t * timer) {
    // Retrieve the pointer to the 'isStabilizing' flag
    bool* pIsStabilizing = (bool*)timer->user_data;
    
    // Reset the flag
    if (pIsStabilizing) {
        *pIsStabilizing = false;
    }
    
    // Actually start the Forward Calibration logic
    calibStartForward();
    
    USBSerial.println("[UI] Forward Sampling started. Accelerate NOW!");
}

// ===========================================================
// FORWARD BUTTON EVENT HANDLER
// ===========================================================
void forwardCalButton_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);

    // Static variable INSIDE the function (unique to this button)
    static bool isStabilizing = false;

    if (code == LV_EVENT_CLICKED) {
        // 1. DEBOUNCE CHECK
        if (isStabilizing) {
            USBSerial.println("[UI] Ignored double-click.");
            return;
        }

        // 2. STATE CHECK
        CalibState cs = calibGetState();
        if (cs == CALIB_GRAVITY_SAMPLING || 
            cs == CALIB_FORWARD_SAMPLING || 
            cs == CALIB_READY_TO_COMPUTE) {
            USBSerial.println("[UI] Ignored click (Calibration in progress).");
            return;
        }

        // 3. PREREQUISITE CHECK (Crucial for Forward Calib)
        // We cannot drive if we don't know which way is Down.
        if (!calibHasGravity()) {
            USBSerial.println("[UI] Error: Missing Gravity Data. Please do Gravity Calib first.");
            // Optional: Update a UI Label here to warn the user
            // lv_label_set_text(ui_LabelStatus, "Error: Do Gravity First!");
            return;
        }

        // 4. Lock the button
        isStabilizing = true;
        
        USBSerial.println("[UI] Forward Calib Button Pressed.");
        USBSerial.println("[UI] Waiting 1s before sampling starts...");

        // Create timer and PASS THE ADDRESS of our static variable
        // We wait 1 second to give the user time to put their hand back on the wheel
        lv_timer_t * timer = lv_timer_create(onForwardTimerExpired, 1000, &isStabilizing);
        lv_timer_set_repeat_count(timer, 1);
    }
}


//***************************************************************************************************
void updateCalibration() {
  static CalibState prevCalibState = CALIB_IDLE;
  CalibState currentState = calibGetState();

  // Detect state changes
  if (currentState != prevCalibState) {
      
      // ---------------------------------------------------------
      // CASE 1: Gravity Calibration Finished (Stationary Step)
      // ---------------------------------------------------------
      // The library automatically goes from GRAVITY_SAMPLING -> IDLE on success.
      if (prevCalibState == CALIB_GRAVITY_SAMPLING && currentState == CALIB_IDLE) {
          if (calibHasGravity()) {
              // Save Gravity Vector and Scale Factor immediately
              calibSaveGravityToNvs(); 
              USBSerial.println("[Main] Gravity/Scale Saved to NVS!");
              // TODO: Update UI: "Step 1 (Gravity) Complete!"
          }
      }

      // ---------------------------------------------------------
      // CASE 2: Forward Calibration Finished (Driving Step)
      // ---------------------------------------------------------
      // The library goes from FORWARD_SAMPLING -> READY_TO_COMPUTE on success.
      else if (currentState == CALIB_READY_TO_COMPUTE) {
          // Sampling is done. Now compute the full rotation matrix.
          if (calibComputeRotation()) {
              // Save Rotation Matrix immediately
              calibSaveRotationToNvs(); 
              USBSerial.println("[Main] Rotation Matrix Computed & Saved!");
              // TODO: Update UI: "Calibration Success!"
          } else {
              USBSerial.println("[Main] Computation Logic Failed.");
              // TODO: Update UI: "Computation Failed"
          }
      } 
      
      // ---------------------------------------------------------
      // CASE 3: Error Occurred
      // ---------------------------------------------------------
      else if (currentState == CALIB_ERROR) {
            USBSerial.println("[Main] Calibration Error occurred.");
            // TODO: Update UI: "Error - Try Again"
      }

      // Update tracking variable
      prevCalibState = currentState;
  }
  
  // ---------------------------------------------------------
  // UI Progress Update (Run continuously while sampling)
  // ---------------------------------------------------------
  if (currentState == CALIB_GRAVITY_SAMPLING || currentState == CALIB_FORWARD_SAMPLING) {
      uint32_t remaining = calibGetRemainingMs();
      // TODO: Update UI progress bar or countdown text using 'remaining'
  }
}


//***************************************************************************************************
void updateGMeterDisplay(float vert, float horiz) {
  // Calculate center and radius
  int center_x = WIDTH_DISPLAY / 2;   // 224
  int center_y = HEIGHT_DISPLAY / 2;  // 184
  int radius = HEIGHT_DISPLAY / 2;    // 184
  
  // Calculate raw display coordinates (may be outside circle)
  // Invert horiz and vert signs for inertial display
  int x_raw = center_x - (int)((WIDTH_DISPLAY * -horiz) / (2.0 * G_SCALE_DISPLAY));
  int y_raw = center_y - (int)((HEIGHT_DISPLAY * -vert) / (2.0 * G_SCALE_DISPLAY));
  
  // Calculate distance from center
  float dx = x_raw - center_x;
  float dy = y_raw - center_y;
  float distance = sqrt(dx * dx + dy * dy);
  
  // Constrain to circle boundary if outside
  int x_display, y_display;
  if (distance > radius) {
    // Scale back to circle edge, preserving direction
    float scale = radius / distance;
    x_display = center_x + (int)(dx * scale);
    y_display = center_y + (int)(dy * scale);
  } else {
    // Inside circle, use raw position
    x_display = x_raw;
    y_display = y_raw;
  }
  
  // Update G-meter dot position (only if on Screen3 and dot exists)
  if (lv_scr_act() == ui_Screen3 && ui_gMeterDot != NULL) {
    lv_obj_set_pos(ui_gMeterDot, x_display - 15, y_display - 15);  // Center 30px dot
    
    // Detect if position changed significantly (> 2 pixels)
    bool position_changed = (abs(x_display - last_x_display) > 1) || 
                           (abs(y_display - last_y_display) > 1);
    
    if (position_changed) {
      // Position changed - update trail
      last_position_change_time = millis();
      
      // Shift trail position history (newest to oldest)
      for (int i = TRAIL_LENGTH - 1; i > 0; i--) {
        trail_pos_x[i] = trail_pos_x[i - 1];
        trail_pos_y[i] = trail_pos_y[i - 1];
      }
      trail_pos_x[0] = last_x_display;
      trail_pos_y[0] = last_y_display;
      
      // Update trail dot positions and set full opacity
      for (int i = 0; i < TRAIL_LENGTH; i++) {
        if (ui_gMeterTrail[i] != NULL) {
          lv_obj_set_pos(ui_gMeterTrail[i], trail_pos_x[i] - 10, trail_pos_y[i] - 10);  // Center 20px dot
          lv_obj_set_style_bg_opa(ui_gMeterTrail[i], trail_opacity[i] * 255 / 100, LV_PART_MAIN);
        }
      }
      
      // Store current position for next comparison
      last_x_display = x_display;
      last_y_display = y_display;
      
    } else {
      // Position hasn't changed - check if we should fade trail
      unsigned long stationary_time = millis() - last_position_change_time;
      
      if (stationary_time > 500) {  // Start fading after 500ms of being stationary
        // Calculate fade factor (0.0 to 1.0)
        float fade_duration = 700.0;  // Fade over 700ms
        float fade_progress = min(1.0f, (stationary_time - 500) / fade_duration);
        
        // Fade out trail dots
        for (int i = 0; i < TRAIL_LENGTH; i++) {
          if (ui_gMeterTrail[i] != NULL) {
            int current_opacity = trail_opacity[i] * (1.0 - fade_progress);
            lv_obj_set_style_bg_opa(ui_gMeterTrail[i], current_opacity * 255 / 100, LV_PART_MAIN);
          }
        }
      }
    }

    // ================================================================
    //   PEAK-HOLD G-FORCE (adjustable interval)
    // ================================================================
    const unsigned long GMETER_PEAK_INTERVAL_MS = 2000;   // 1 second

    // Static state (retains value between calls)
    static unsigned long last_peak_update_time = 0;
    static float peak_g = 0.0f;

    // Compute horizontal-plane G magnitude
    float g_current = sqrtf(vert * vert + horiz * horiz);

    // Track peak value inside interval
    if (g_current > peak_g)
        peak_g = g_current;

    // Update every GMETER_PEAK_INTERVAL_MS
    unsigned long now = millis();
    if (now - last_peak_update_time >= GMETER_PEAK_INTERVAL_MS) {
        last_peak_update_time = now;

        // Update label
        char g_text[16];
        snprintf(g_text, sizeof(g_text), "%.02f", peak_g);
        lv_label_set_text(ui_screen3braking, g_text);

        // Reset for next window
        peak_g = 0.0f;
    }

  }
}


//***************************************************************************************************
void applyInertialTransform(float sensor[3], float display[3]) {
  // Transform sensor coordinates to car inertial display coordinates
  // sensor[3]: raw accelerometer [x, y, z]
  // display[3]: output [x_vertical, y_horizontal, up_reference]
  
  for (int i = 0; i < 3; i++) {
    display[i] = 0;
    for (int j = 0; j < 3; j++) {
      display[i] += ROTATION_MATRIX[i][j] * sensor[j];
    }
  }
}


//***************************************************************************************************
void updateMotionState() {
  static unsigned long lastMotionCheckTime = 0;
  static unsigned long lastMotionTime = 0;
  static int startupIgnoreCount = 20; // Ignore first 20 intervals (2 seconds) for sensor stabilization

  if (lastMotionTime == 0) {
      lastMotionTime = millis();
  }
  
  unsigned long currentTime = millis();

  if (currentTime - lastMotionCheckTime >= MOTION_CHECK_INTERVAL) {
    lastMotionCheckTime = currentTime;

    // Update the time tracking variables for advanced complementary filter
    current_time = micros();
    dt = (current_time - previous_time) / 1000000.0;  // Calculate dt in seconds
    previous_time = current_time;  // Update previous time

    if (dt > 0) {
      sampling_frequency = 1.0 / dt;  // Sampling frequency = 1 / dt
    }  

    // Calculate accelerometer magnitude and change
    float accelMagnitude = sqrt(acc_disp_peak.x * acc_disp_peak.x + 
                                acc_disp_peak.y * acc_disp_peak.y + 
                                acc_disp_peak.z * acc_disp_peak.z);
    float accelChange = abs(accelMagnitude - lastAccelMagnitude);
    lastAccelMagnitude = accelMagnitude;
    
    // Calculate gyroscope magnitude and change
    float gyroMagnitude = sqrt(gyr_disp_peak.x * gyr_disp_peak.x + 
                               gyr_disp_peak.y * gyr_disp_peak.y + 
                               gyr_disp_peak.z * gyr_disp_peak.z);
    float gyroChange = abs(gyroMagnitude - lastGyroMagnitude);
    lastGyroMagnitude = gyroMagnitude;

    // Store current changes for MQTT transmission
    currentAccelChange = accelChange;
    currentGyroChange = gyroChange;

    // Skip motion detection during startup stabilization period
    if (startupIgnoreCount > 0) {
      startupIgnoreCount--;
      USBSerial.printf("Startup stabilization: %d intervals remaining (Accel: %.2f, Gyro: %.2f)\n", 
                       startupIgnoreCount, accelChange, gyroChange);
      
      // Continue to G-meter display update and peak reset (skip motion detection only)
    } else {
      // Track peak changes during interval
      if (accelChange > peakAccelChange) peakAccelChange = accelChange;
      if (gyroChange > peakGyroChange) peakGyroChange = gyroChange;

      // Motion detected if EITHER sensor exceeds its threshold
      bool motionDetectedNow = (accelChange > ACCEL_MOTION_THRESHOLD) || (gyroChange > GYRO_MOTION_THRESHOLD);

      if (motionDetectedNow) {
        if (!g_isCurrentlyMoving) {
          USBSerial.printf("Movement Detected! (Accel: %.2f, Gyro: %.2f)\n", accelChange, gyroChange);
          g_isCurrentlyMoving = true;

          if (ENABLE_MOTION_MQTT && mqttClient.connected()) {
            mqttClient.publish(MOTION_TOPIC, "1");
            lastMotionTXTime = millis();
            USBSerial.println("TX motion MQTT: Moving (immediate)");
          }
        }
        lastMotionTime = currentTime;
      } else {
        if (g_isCurrentlyMoving && (currentTime - lastMotionTime > MOTION_TIMEOUT)) {
          USBSerial.println("Movement Stopped.");
          g_isCurrentlyMoving = false;
        }
      }
      // >>> INSERT NEW CODE HERE (gyro bias learning when stationary)
      if (!g_isCurrentlyMoving && (gyro_magnitude < 2.0f)) {
        // Slowly adapt gyro bias in sensor coordinates
        gyro_bias_sensor_x = gyro_bias_sensor_x * (1.0f - BIAS_LEARN_BETA) + gyr.x * BIAS_LEARN_BETA;
        gyro_bias_sensor_y = gyro_bias_sensor_y * (1.0f - BIAS_LEARN_BETA) + gyr.y * BIAS_LEARN_BETA;
        gyro_bias_sensor_z = gyro_bias_sensor_z * (1.0f - BIAS_LEARN_BETA) + gyr.z * BIAS_LEARN_BETA;
      }
    }

    // === G-METER INERTIAL DISPLAY layout ===
    // These are acceleration values.  For inertial values we will invert the signs for the display.
    // 
    //                          + vert. UP (braking)
    //                                  |
    // + horiz. LEFT (right turn)  ---- O ---- - horiz. RIGHT (left turn) 
    //                                  |
    //                        - vert. DOWN (accelerating)
    //
    // Transform accelerometer peak to inertial display coordinates

    // Apply the scale factor (Divide raw / scale)
    float currentScaleFactor = calibGetScaleFactor(); 
    float sensor_accel[3] = {
        acc_disp_peak.x / currentScaleFactor, 
        acc_disp_peak.y / currentScaleFactor, 
        acc_disp_peak.z / currentScaleFactor
    };
    float display_accel[3];
    
    applyInertialTransform(sensor_accel, display_accel);
    
    // Store transformed values in global structure for MQTT and display
    acc_inertial.vert = display_accel[0];
    acc_inertial.horiz = display_accel[1];
    acc_inertial.up = display_accel[2];
    
    // Transform gyroscope to car reference frame
    // CRITICAL: Subtract bias BEFORE transformation!
    float sensor_gyro[3] = {
      gyr.x - gyro_bias_sensor_x,
      gyr.y - gyro_bias_sensor_y,
      gyr.z - gyro_bias_sensor_z
    };
    float display_gyro[3];
    applyInertialTransform(sensor_gyro, display_gyro);

    gyr_inertial.vert = display_gyro[0];
    gyr_inertial.horiz = display_gyro[1];
    gyr_inertial.up = display_gyro[2];

    // Calculate gyroscope magnitude and change (use bias-corrected inertial gyro)
    float gyroInstant = sqrt(gyr_inertial.vert  * gyr_inertial.vert +
                            gyr_inertial.horiz * gyr_inertial.horiz +
                            gyr_inertial.up    * gyr_inertial.up);

    // Optional: low-pass filter it a bit for stability
    const float GYRO_MAG_ALPHA = 0.9f;  // keep 90% old, 10% new
    gyro_magnitude = GYRO_MAG_ALPHA * gyro_magnitude + (1.0f - GYRO_MAG_ALPHA) * gyroInstant;    
    
    // Update inclinometer with transformed gyro values
    updateInclinometer(display_gyro[0], display_gyro[1], display_gyro[2]);

    // Update G-meter display
    updateGMeterDisplay(acc_inertial.vert, acc_inertial.horiz);   
    
    // Reset display peak to zero for next 100ms interval
    acc_disp_peak.x = 0;
    acc_disp_peak.y = 0;
    acc_disp_peak.z = 0;
    acc_disp_peak.magnitude = 0;    

    // Reset gyroscope display peak
    gyr_disp_peak.x = 0;
    gyr_disp_peak.y = 0;
    gyr_disp_peak.z = 0;
    gyr_disp_peak.magnitude = 0;    
  }
}


//***************************************************************************************************
// ========== INCLINOMETER FUNCTIONS ==========
void updateInclinometerDisplay() {
  // Only update if the inclinometer screen is active
  if (lv_scr_act() != ui_InclinometerScreen) return;
  
  unsigned long start_time = millis();

  // Only update if inclinometer is initialized
  if (!inclinometer_initialized) return;
  
  // Static variables to remember last displayed values
  static char last_pitch_str[20] = "";
  static char last_roll_str[20] = "";
  static float last_pitch_rotation = 0.0;
  static float last_roll_rotation = 0.0;
  
  // Format pitch string (no leading zero)
  char pitch_str[20];
  int pitch_int = abs((int)pitch_angle);
  int pitch_dec = (int)(abs(pitch_angle) * 10) % 10;
  char pitch_sign = (pitch_angle >= 0) ? '+' : '-';
  sprintf(pitch_str, "%c%d.%d°", pitch_sign, pitch_int, pitch_dec);  // Changed %02d to %d
  
  // Only update label if text changed
  if (strcmp(pitch_str, last_pitch_str) != 0) {
    lv_label_set_text(ui_PitchLabel, pitch_str);
    strcpy(last_pitch_str, pitch_str);
  }
  
  // Only update image rotation if changed by more than 1 degree
  if (abs(pitch_angle - last_pitch_rotation) > 1.0) {
    lv_img_set_angle(ui_PitchCarImage, (int16_t)(pitch_angle * 10));
    last_pitch_rotation = pitch_angle;
  }
  
  // Format roll string (no leading zero)
  char roll_str[20];
  int roll_int = abs((int)roll_angle);
  int roll_dec = (int)(abs(roll_angle) * 10) % 10;
  char roll_sign = (roll_angle >= 0) ? '+' : '-';
  sprintf(roll_str, "%c%d.%d°", roll_sign, roll_int, roll_dec);  // Changed %02d to %d
  
  // Only update label if text changed
  if (strcmp(roll_str, last_roll_str) != 0) {
    lv_label_set_text(ui_RollLabel, roll_str);
    strcpy(last_roll_str, roll_str);
  }
  
  // Only update image rotation if changed by more than 1 degree
  if (abs(roll_angle - last_roll_rotation) > 1.0) {
    lv_img_set_angle(ui_RollCarImage, (int16_t)(roll_angle * 10));
    last_roll_rotation = roll_angle;
  }
  unsigned long end_time = millis();
  unsigned long duration = end_time - start_time;
  if (duration > 10) {  // Only print if took more than 10ms
    Serial.print("Display update took: ");
    Serial.print(duration);
    Serial.println(" ms");
  }
}


//***************************************************************************************************
void calculateGyroBias() {
  // Calculate gyro bias in SENSOR coordinates while stationary
  Serial.println("=== Calibrating Gyro Bias (please keep stationary) ===");
  
  const int GYRO_BIAS_SAMPLES = 200;
  float sum_x = 0.0;
  float sum_y = 0.0;
  float sum_z = 0.0;
  int valid_samples = 0;
  
  for (int i = 0; i < GYRO_BIAS_SAMPLES; i++) {
    unsigned long start_wait = millis();
    bool data_ready = false;
    
    while (!data_ready && (millis() - start_wait < 50)) {
      data_ready = qmi.getDataReady();
      if (!data_ready) {
        delay(1);
      }
    }
    
    if (data_ready) {
      // Read raw gyro - DON'T transform yet!
      qmi.getGyroscope(gyr.x, gyr.y, gyr.z);
      
      sum_x += gyr.x;
      sum_y += gyr.y;
      sum_z += gyr.z;
      valid_samples++;
    }
  }
  
  if (valid_samples > 0) {
    // Store bias in SENSOR coordinates
    gyro_bias_sensor_x = sum_x / valid_samples;
    gyro_bias_sensor_y = sum_y / valid_samples;
    gyro_bias_sensor_z = sum_z / valid_samples;
    
    Serial.print("  Gyro bias calculated from ");
    Serial.print(valid_samples);
    Serial.println(" samples (sensor coordinates):");
    Serial.print("    x=");
    Serial.print(gyro_bias_sensor_x, 3);
    Serial.print(" °/s, y=");
    Serial.print(gyro_bias_sensor_y, 3);
    Serial.print(" °/s, z=");
    Serial.print(gyro_bias_sensor_z, 3);
    Serial.println(" °/s");
  } else {
    Serial.println("  WARNING: Failed to calculate gyro bias!");
  }
}


//***************************************************************************************************
void initializeInclinometer() {
  // Initialize inclinometer - calculate initial angles from first gravity reading
  // This should be called after first valid acc_inertial data is available
  
  // More strict validation - ensure we're close to 1G total and mostly in Z
  float total_g = sqrt(acc_inertial.vert * acc_inertial.vert + 
                       acc_inertial.horiz * acc_inertial.horiz + 
                       acc_inertial.up * acc_inertial.up);
  
  if (abs(acc_inertial.up) > 0.8 && total_g > 0.9 && total_g < 1.1) {
    // First, calibrate gyro bias while stationary
    calculateGyroBias();
    
    // Then calculate initial angles
    pitch_angle = atan2(acc_inertial.vert, acc_inertial.up) * 180.0 / PI;
    roll_angle = atan2(acc_inertial.horiz, acc_inertial.up) * 180.0 / PI;
    
    // Sync gyro-only angles with the accel-derived initial orientation
    pitch_gyro = pitch_angle;
    roll_gyro = roll_angle;

    // Initialize accel LPF state
    acc_lp_vert = acc_inertial.vert;
    acc_lp_horiz = acc_inertial.horiz;
    acc_lp_up = acc_inertial.up;

    last_inclinometer_update = micros();
    inclinometer_initialized = true;
    
    Serial.println("=== Inclinometer Initialized ===");
    Serial.print("  acc_inertial: vert=");
    Serial.print(acc_inertial.vert, 3);
    Serial.print(" horiz=");
    Serial.print(acc_inertial.horiz, 3);
    Serial.print(" up=");
    Serial.println(acc_inertial.up, 3);
    Serial.print("  Initial Pitch: ");
    Serial.print(pitch_angle, 2);
    Serial.println("°");
    Serial.print("  Initial Roll: ");
    Serial.print(roll_angle, 2);
    Serial.println("°");
  }
}


//***************************************************************************************************
void integrateGyroAngles(float dt, float gyro_vert, float gyro_horiz, float gyro_up) {
  // Here gyro_* are already in the car/inertial frame (gyr_inertial) and bias-subtracted
  // They are in degrees/second (QMI8658 typical)
  if (dt <= 0.0f) return;

  // Integrate gyro rates to update gyro-only angles
  // NOTE: mapping depends on your coordinate convention; this follows your prior usage:
  pitch_gyro += gyro_horiz * dt;
  roll_gyro  += gyro_vert  * dt;

  // Keep gyro-only angles in -180..180 range
  if (pitch_gyro > 180.0f) pitch_gyro -= 360.0f;
  if (pitch_gyro < -180.0f) pitch_gyro += 360.0f;
  if (roll_gyro > 180.0f) roll_gyro -= 360.0f;
  if (roll_gyro < -180.0f) roll_gyro += 360.0f;
}


//***************************************************************************************************
void calculateAccelAngles(float &pitch_accel, float &roll_accel) {
  // Shortcut references
  float vert  = acc_inertial.vert;   // forward/back
  float horiz = acc_inertial.horiz;  // left/right
  float up    = acc_inertial.up;     // up

  // 1) Magnitude of acceleration vector in car frame
  accel_magnitude = sqrtf(vert * vert + horiz * horiz + up * up);

  // 2) Compute accel-based pitch/roll (gravity vector) if geometry is ok
  if (fabsf(up) > 0.1f) {  // Avoid near 90° singularity
    pitch_accel = atan2f(vert,  up) * 180.0f / PI;   // nose up/down
    roll_accel  = atan2f(horiz, up) * 180.0f / PI;   // left/right tilt
  } else {
    // Near vertical: fall back to current fused angles
    pitch_accel = pitch_angle;
    roll_accel  = roll_angle;
  }

  // 3) Gravity magnitude error
  float mag_err = fabsf(accel_magnitude - 1.0f);  // how far from 1 g

  // 4) Gravity *direction* mismatch in angle space
  // Compare accel tilt vs. gyro-predicted tilt
  float pitch_err = fabsf(pitch_accel - pitch_gyro);
  float roll_err  = fabsf(roll_accel  - roll_gyro);
}


//***************************************************************************************************
// =============================================================
// Compute fused angles (gyro + accel) with dynamic tau
// WITHOUT modifying global pitch_angle / roll_angle.
// Outputs fused angles through references.
// =============================================================
void computeFusedAngles(float dt, float fwd_g,
                        float &out_pitch, float &out_roll) {
    // ---- Step 1: Gyro integration (prediction) ----
    integrateGyroAngles(dt,
                        gyr_inertial.vert,
                        gyr_inertial.horiz,
                        gyr_inertial.up);

    // ---- Step 2: Accelerometer tilt measurement ----
    float pitch_accel, roll_accel;
    calculateAccelAngles(pitch_accel, roll_accel);

    // ---- Step 3: Dynamic tau based on straight-line acceleration ----
    float tau;
    if (fwd_g > 0.30f)      tau = 12.0f;
    else if (fwd_g > 0.10f) tau = 6.0f;
    else                    tau = 1.5f;

    float alpha = tau / (tau + dt);
    alpha = constrain(alpha, 0.90f, 0.999f);

    // ---- Step 4: Complementary fusion ----
    out_pitch = alpha * pitch_angle + (1.0f - alpha) * pitch_accel;
    out_roll  = alpha * roll_angle  + (1.0f - alpha) * roll_accel;

    // ---- Step 5: Normalize ----
    if (out_pitch > 180.0f) out_pitch -= 360.0f;
    if (out_pitch < -180.0f) out_pitch += 360.0f;
    if (out_roll > 180.0f)   out_roll -= 360.0f;
    if (out_roll < -180.0f)  out_roll += 360.0f;

    // ---- Debug values for MQTT ----
    last_effective_tau = tau;
    last_alpha         = alpha;
}


//***************************************************************************************************
void updateInclinometerWithFreeze(float dt) {
    float lat_g = fabsf(acc_inertial.horiz);
    float fwd_g = fabsf(acc_inertial.vert);
    float yaw_rate = fabsf(gyr_inertial.up);
    unsigned long now_ms = millis();

    // ---- Thresholds ----
    const float TURN_FREEZE_THRESHOLD     = 0.10f;
    const float TURN_FREEZE_EXIT_THRESH   = 0.07f;
    const float YAW_TURN_THRESHOLD        = 8.0f;   // tuned threshold
    const uint32_t RECOVER_DELAY_MS       = 150;
    const uint32_t RECOVER_BLEND_MS       = 700;

    // ---- Real turn detection ----
    bool is_real_turn = (lat_g > TURN_FREEZE_THRESHOLD) &&
                        (yaw_rate > YAW_TURN_THRESHOLD);

    // =============================================================
    // 1. TURN-FREEZE ENTRY
    // =============================================================
    if (!turn_freeze_active && is_real_turn) {
        turn_freeze_active   = true;
        recovering_from_turn = false;

        frozen_pitch = pitch_angle;
        frozen_roll  = roll_angle;

        turn_freeze_start_ms = now_ms;
        return;
    }

    // =============================================================
    // 2. TURN-FREEZE ACTIVE — freeze angles
    // =============================================================
    if (turn_freeze_active) {
        pitch_angle = frozen_pitch;
        roll_angle  = frozen_roll;

        if (lat_g < TURN_FREEZE_EXIT_THRESH) {
            turn_freeze_active   = false;
            recovering_from_turn = true;

            turn_freeze_exit_ms = now_ms;
            recover_start_ms    = now_ms;
        }

        return;
    }

    // =============================================================
    // 3. RECOVERY PHASE — blend frozen → fused
    // =============================================================
    if (recovering_from_turn) {
        unsigned long since_exit = now_ms - turn_freeze_exit_ms;

        // ---- Delay: allow accel to settle ----
        if (since_exit < RECOVER_DELAY_MS) {
            pitch_angle = frozen_pitch;
            roll_angle  = frozen_roll;
            return;
        }

        // ---- Compute blending factor ----
        unsigned long blend_ms =
            now_ms - (turn_freeze_exit_ms + RECOVER_DELAY_MS);

        float t = (float)blend_ms / (float)RECOVER_BLEND_MS;
        if (t > 1.0f) t = 1.0f;

        // ---- Compute fused pitch/roll using new helper ----
        float fused_pitch, fused_roll;
        computeFusedAngles(dt, fwd_g, fused_pitch, fused_roll);

        // ---- Blend frozen angle → fused angle ----
        pitch_angle = frozen_pitch * (1.0f - t) + fused_pitch * t;
        roll_angle  = frozen_roll  * (1.0f - t) + fused_roll  * t;

        if (t >= 1.0f)
            recovering_from_turn = false;

        return;
    }

    // =============================================================
    // 4. NORMAL OPERATION
    // =============================================================
    float fused_pitch, fused_roll;
    computeFusedAngles(dt, fwd_g, fused_pitch, fused_roll);

    pitch_angle = fused_pitch;
    roll_angle  = fused_roll;
}



//***************************************************************************************************
void updateInclinometer(float gyro_vert, float gyro_horiz, float gyro_up) {
  if (!inclinometer_initialized) {
    // Auto-initialize on first call with valid data
    if (abs(acc_inertial.up) > 0.5) {  // Wait for reasonable gravity reading
      initializeInclinometer();
    }
    return;
  }
  
  unsigned long current_time_incl = micros();
  float dt_incl = (current_time_incl - last_inclinometer_update) / 1000000.0;  // Convert to seconds
  
  // If dt is unreasonably large (>100ms), just skip this update entirely
  // This handles major gaps like screen switches or system pauses
  if (dt_incl > 0.1) {
    last_inclinometer_update = current_time_incl;
    return;  // Skip update, keep previous angles
  }
  
  if (dt_incl < 0.0001) return;  // Skip if called too quickly
  
  // Clamp dt to reasonable maximum for integration
  if (dt_incl > 0.05) dt_incl = 0.05;  // Cap at 50ms for gyro integration
  
  updateInclinometerWithFreeze(dt_incl);

  last_inclinometer_update = current_time_incl;
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
        lv_obj_clear_flag(ui_labelMotionIcon2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(ui_labelMotionIcon3, LV_OBJ_FLAG_HIDDEN);
    } else {
        // Hide the icon if not moving
        lv_obj_add_flag(ui_labelMotionIcon, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_labelMotionIcon2, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_labelMotionIcon3, LV_OBJ_FLAG_HIDDEN);
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
  lv_obj_add_flag(ui_labelMotionIcon2, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(ui_labelMotionIcon3, LV_OBJ_FLAG_HIDDEN);

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

  qmi.disableAccelerometer();
  qmi.disableGyroscope();
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
void reinitializeMotionBaseline() {
  // Take multiple readings and average them for a stable baseline
  float accelSum = 0;
  float gyroSum = 0;
  int validReadings = 0;
  
  for (int i = 0; i < 20; i++) {
    if (qmi.getDataReady()) {
      if (qmi.getAccelerometer(acc.x, acc.y, acc.z) && qmi.getGyroscope(gyr.x, gyr.y, gyr.z)) {
        float accelMag = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
        float gyroMag = sqrt(gyr.x * gyr.x + gyr.y * gyr.y + gyr.z * gyr.z);
        accelSum += accelMag;
        gyroSum += gyroMag;
        validReadings++;
      }
    }
    delay(20);
  }
  
  if (validReadings > 0) {
    lastAccelMagnitude = accelSum / validReadings;
    lastGyroMagnitude = gyroSum / validReadings;
    USBSerial.printf("Motion baseline reset: Accel=%.2f m/s², Gyro=%.2f °/s (averaged from %d readings)\n", 
                     lastAccelMagnitude, lastGyroMagnitude, validReadings);
  } 

  // Reset peak changes
  peakAccelChange = 0;
  peakGyroChange = 0;  

}


//***************************************************************************************************
void readImuData() {
  // Print and publish to MQTT if connected and not receiving HTTP data
  if (imuPeakInitialized) {     

    char payload[700];  // Slightly enlarged buffer

    // Turn state as human-readable string
    const char* mode_str =
        turn_freeze_active ? "freeze" :
        recovering_from_turn ? "recover" :
        "normal";

    // Derived signals
    float lat_g   = fabsf(acc_inertial.horiz);
    float fwd_g   = fabsf(acc_inertial.vert);
    float yaw_rate = gyr_inertial.up;

    snprintf(payload, sizeof(payload), 
        "{"
          "\"accel_peak\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"mag\":%.2f},"
          "\"gyro_peak\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"mag\":%.2f},"

          "\"accel_change\":%.2f,"
          "\"gyro_change\":%.2f,"

          "\"accel_inertial\":{\"vert\":%.2f,\"horiz\":%.2f,\"up\":%.2f},"
          "\"gyro_inertial\":{\"vert\":%.2f,\"horiz\":%.2f,\"up\":%.2f},"

          "\"pitch\":%.2f,"
          "\"roll\":%.2f,"

          "\"accel_mag\":%.2f,"
          "\"gyro_mag\":%.2f,"

          "\"tau\":%.2f,"
          "\"alpha\":%.3f,"

          "\"yaw_rate\":%.2f,"
          "\"lat_g\":%.2f,"
          "\"fwd_g\":%.2f,"

          "\"mode\":\"%s\","

          "\"freeze\":%d,"
          "\"recover\":%d"
        "}",

        acc_peak.x, acc_peak.y, acc_peak.z, acc_peak.magnitude,
        gyr_peak.x, gyr_peak.y, gyr_peak.z, gyr_peak.magnitude,

        peakAccelChange, peakGyroChange,

        acc_inertial.vert, acc_inertial.horiz, acc_inertial.up,
        gyr_inertial.vert, gyr_inertial.horiz, gyr_inertial.up,

        pitch_angle, roll_angle,

        accel_magnitude, gyro_magnitude,

        last_effective_tau, last_alpha,

        yaw_rate,
        lat_g,
        fwd_g,

        mode_str,

        turn_freeze_active ? 1 : 0,
        recovering_from_turn ? 1 : 0
    );

    // Publish to IMU topic
    if (ENABLE_MOTION_MQTT && mqttClient.connected()) {
      mqttClient.publish(IMU_TOPIC, payload);
    }
    
    USBSerial.println(payload);

    // Reset peak changes for next cycle
    peakAccelChange = 0;
    peakGyroChange = 0;    

    // Reset IMU peaks after publishing  
    acc_peak.x = 0;
    acc_peak.y = 0;
    acc_peak.z = 0;      
    acc_peak.magnitude = 0;

    // Reset gyroscope peak:
    gyr_peak.x = 0;
    gyr_peak.y = 0;
    gyr_peak.z = 0;
    gyr_peak.magnitude = 0;    

    Serial.print("Sampling freq: ");
    Serial.print(sampling_frequency);
    Serial.println(" Hz"); 
    Serial.println();
  }
}


//***************************************************************************************************
// Runs every loop to update IMU data and track peaks
void updateImuData() {
  if (qmi.getDataReady()) {
    qmi.getAccelerometer(acc.x, acc.y, acc.z);
    qmi.getGyroscope(gyr.x, gyr.y, gyr.z);
    
    // Feed raw data to calibration logic
    // It only does math if a calibration step is actually active.
    float raw_accel[3] = {acc.x, acc.y, acc.z};
    calibUpdate(raw_accel);

    // Calculate current magnitudes
    float accelMagnitude = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
    float gyroMagnitude = sqrt(gyr.x * gyr.x + gyr.y * gyr.y + gyr.z * gyr.z);
    
    // Initialize peaks on first reading
    if (!imuPeakInitialized) {
      acc_peak.x = acc.x;
      acc_peak.y = acc.y;
      acc_peak.z = acc.z;
      acc_peak.magnitude = accelMagnitude;
      
      gyr_peak.x = gyr.x;
      gyr_peak.y = gyr.y;
      gyr_peak.z = gyr.z;
      gyr_peak.magnitude = gyroMagnitude;

      acc_disp_peak.x = acc.x;
      acc_disp_peak.y = acc.y;
      acc_disp_peak.z = acc.z;
      acc_disp_peak.magnitude = accelMagnitude;    
      
      gyr_disp_peak.x = gyr.x;
      gyr_disp_peak.y = gyr.y;
      gyr_disp_peak.z = gyr.z;
      gyr_disp_peak.magnitude = gyroMagnitude;      
      
      imuPeakInitialized = true;
    } else {
      // Update accelerometer peak if current magnitude is higher
      if (accelMagnitude > acc_peak.magnitude) {
        acc_peak.x = acc.x;
        acc_peak.y = acc.y;
        acc_peak.z = acc.z;
        acc_peak.magnitude = accelMagnitude;
      }
      
      // Update gyroscope peak if current magnitude is higher
      if (gyroMagnitude > gyr_peak.magnitude) {
        gyr_peak.x = gyr.x;
        gyr_peak.y = gyr.y;
        gyr_peak.z = gyr.z;
        gyr_peak.magnitude = gyroMagnitude;
      }

      // Update display accelerometer peak if current magnitude is higher
      if (accelMagnitude > acc_disp_peak.magnitude) {
        acc_disp_peak.x = acc.x;
        acc_disp_peak.y = acc.y;
        acc_disp_peak.z = acc.z;
        acc_disp_peak.magnitude = accelMagnitude;
      }  

      // Update gyroscope display peak:
      if (gyroMagnitude > gyr_disp_peak.magnitude) {
        gyr_disp_peak.x = gyr.x;
        gyr_disp_peak.y = gyr.y;
        gyr_disp_peak.z = gyr.z;
        gyr_disp_peak.magnitude = gyroMagnitude;
      }
    }
  }
}

// =============================================================
// MQTT BRIDGE (Connects Library to Main Sketch)
// =============================================================
void myCalibMqttSender(const char* topic, const char* payload) {
  // Only send if we have an active connection
  if (mqttClient.connected()) {
    mqttClient.publish(topic, payload);
    USBSerial.print("[MQTT] Calibration sent: ");
    USBSerial.println(payload);
  } else {
    USBSerial.println("[MQTT] Skipped calibration send (Not connected).");
  }
}


/****************************************************************************************************
 * INITIALIZATION HELPER FUNCTIONS
 *
 * Initialize PMIC (AXP2101) - Power Management IC
 * Configures charging, voltage rails, and ADC
 */
void initPMIC() {
    if (!pmic.init()) {
        USBSerial.println("ERROR: PMIC AXP2101 failed to initialize!");
    }
    
    USBSerial.println("PMIC init OK.");

    // This is required to clear stale hardware flags inside the PMIC after a
    // warm boot from full shutdown. It is harmless on other boot types.
    USBSerial.println("Performing full ADC subsystem reset to ensure clean state...");
    adcOff();
    delay(50); 
    adcOn();
    delay(150); // A longer delay to allow the ADC system to fully stabilize.
    
    // Enable voltage rails
    pmic.enableALDO1(); 
    pmic.enableALDO2(); 
    pmic.enableBLDO1(); 
    pmic.enableALDO3();
    
    USBSerial.println("PMIC initialization complete");
}

/****************************************************************************************************
 * Initialize I/O Expander (TCA9554)
 * Sets up GPIO pins for LCD control
 */
void initIOExpander() {
    // This is creating an error with the pin parameters from the I/O Expander constructor
    // But after extensive testing, it works correctly at runtime.
    expander = new EXAMPLE_CHIP_CLASS(TCA95xx_8bit,
                                      (i2c_port_t)0, ESP_IO_EXPANDER_I2C_TCA9554_ADDRESS_000,
                                      IIC_SCL, IIC_SDA);
    
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
}

/****************************************************************************************************
 * Initialize Touch Controller (FT3168)
 * Sets up I2C communication with capacitive touch panel
 */
void initTouch() {
    IIC_Bus = std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);
    FT3168 = std::make_unique<Arduino_FT3x68>(IIC_Bus, FT3168_DEVICE_ADDRESS, DRIVEBUS_DEFAULT_VALUE, TP_INT, Arduino_IIC_Touch_Interrupt);
    while (FT3168->begin() == false) {
      USBSerial.println("ERROR: FT3168 initialization fail");
      delay(2000);
    }
    
    FT3168->IIC_Write_Device_State(FT3168->Arduino_IIC_Touch::Device::TOUCH_POWER_MODE,
                                  FT3168->Arduino_IIC_Touch::Device_Mode::TOUCH_POWER_ACTIVE);
}

/****************************************************************************************************
 * Initialize Display Hardware
 * Sets up display controller and basic configuration
 */
void initDisplay() {
    gfx->begin();

    gfx->fillScreen(BLACK);
    gfx->Display_Brightness(150);    
}

/****************************************************************************************************
 * Initialize LVGL Graphics Library
 * Sets up display buffers, input devices, timers, and loads UI
 */
void initLVGL() {
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
    }
    
    if (esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000) != ESP_OK) {
        USBSerial.println("ERROR: Failed to start LVGL tick timer");
    }

    // Load UI from SquareLine Studio
    ui_init();

    USBSerial.println("LVGL initialization complete");
}

/****************************************************************************************************
 * Initialize UI Event Handlers and Components
 * Registers button callbacks and configures UI elements
 */
void initUIHandlers() {
    // Register button event handlers
    lv_obj_add_event_cb(ui_ButtonLatest, buttonLatest_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_ButtonNew, buttonNew_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_ButtonBack, buttonBack_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_ButtonGmeter, buttonGmeter_event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_gravityCalButton, gravityCalButton_event_handler, LV_EVENT_CLICKED, NULL);   
    lv_obj_add_event_cb(ui_forwardCalButton, forwardCalButton_event_handler, LV_EVENT_CLICKED, NULL); 
    USBSerial.println("  Button event handlers registered");

    // Attach Screen 2 event handler for image loading
    lv_obj_add_event_cb(ui_Screen2, screen2_event_handler, LV_EVENT_ALL, NULL);
    USBSerial.println("  Screen 2 event handler attached");

    lv_obj_add_event_cb(ui_Screen3, screen3_event_handler, LV_EVENT_ALL, NULL);
    USBSerial.println("  Screen 3 event handler attached");    

    // Initialize the Motion Icon Label
    lv_label_set_text(ui_labelMotionIcon, LV_SYMBOL_CHARGE);
    lv_obj_set_style_text_font(ui_labelMotionIcon, &lv_font_montserrat_24, 0);
    lv_obj_add_flag(ui_labelMotionIcon, LV_OBJ_FLAG_HIDDEN);  // Start hidden

    lv_label_set_text(ui_labelMotionIcon2, LV_SYMBOL_CHARGE);
    lv_obj_set_style_text_font(ui_labelMotionIcon2, &lv_font_montserrat_24, 0);
    lv_obj_add_flag(ui_labelMotionIcon2, LV_OBJ_FLAG_HIDDEN);  // Start hidden  
    
    lv_label_set_text(ui_labelMotionIcon3, LV_SYMBOL_CHARGE);
    lv_obj_set_style_text_font(ui_labelMotionIcon3, &lv_font_montserrat_24, 0);
    lv_obj_add_flag(ui_labelMotionIcon3, LV_OBJ_FLAG_HIDDEN);  // Start hidden     
    
    USBSerial.println("  Motion icon configured");
}

/****************************************************************************************************
 * Check and initialize PSRAM
 * @return true if PSRAM available, false otherwise
 */
bool initPSRAM() {
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
 */
void initJPEGDecoder() {
    // The GFX library expects RGB565 format (Big Endian)
    TJpgDec.setSwapBytes(false);
}

/****************************************************************************************************
 * Initialize IMU/Motion Sensor (QMI8658)
 * Configures accelerometer and Wake-on-Motion
 */
void initIMU() {
    if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL)) {
        USBSerial.println("FATAL: Failed to find QMI8658 - check your wiring!");
        while(1) { delay(1000); }
    }
    
    USBSerial.println("QMI8658 Initialized.");

    // Poll for initial motion state
    USBSerial.println("Getting initial motion state...");
    for (int i = 0; i < 5; i++) {
        updateMotionState(); // This will populate g_isCurrentlyMoving
        delay(20);
    }

    qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_1000Hz, SensorQMI8658::LPF_MODE_0);
    qmi.enableAccelerometer();

    qmi.configGyroscope(
      SensorQMI8658::GYR_RANGE_512DPS,   // GYR_RANGE_16DPS / GYR_RANGE_32DPS / GYR_RANGE_64DPS / GYR_RANGE_128DPS / GYR_RANGE_256DPS / GYR_RANGE_512DPS / GYR_RANGE_1024DPS
      SensorQMI8658::GYR_ODR_1793_6Hz,   // GYR_ODR_7174_4Hz / GYR_ODR_3587_2Hz / GYR_ODR_1793_6Hz / GYR_ODR_896_8Hz / GYR_ODR_448_4Hz / GYR_ODR_224_2Hz / GYR_ODR_112_1Hz / GYR_ODR_56_05Hz / GYR_ODR_28_025H
      SensorQMI8658::LPF_MODE_0);        // LPF_MODE_0 (2.66% of ODR) / LPF_MODE_1 (3.63% of ODR) / LPF_MODE_2 (5.39% of ODR) / LPF_MODE_3 (13.37% of ODR): JPL: 239.8HZ
    qmi.enableGyroscope();  

    USBSerial.println("Accelerometer and Gyroscope configured for continuous reading");

    // Give IMU time to stabilize
    delay(100);

    // Initialize motion detection baseline
    reinitializeMotionBaseline();

    calibInit();        // Initialize calibration state

    // 1. Register the MQTT Callback
    calibSetMqttCallback(myCalibMqttSender);
    
    // 2. Set the Car Unit flag based on compiler macro
    #ifdef CAR
      calibSetCarUnit(true);
    #else
      calibSetCarUnit(false);
    #endif    

    calibLoadFromNvs(); // Try to load Scale, Gravity, and Rotation from NVS    
}

/****************************************************************************************************
 * Initialize Battery Monitoring
 * Reads initial battery state and configures sleep policy
 */
void initBattery() {
    // Get initial battery readings
    updateBatteryInfo();
    
    // Set sleep policy based on initial power state
    allowSleep = !vbusPresent;
    if (allowSleep) {
        USBSerial.println("Starting on battery - sleep enabled after inactivity");
    } else {
        USBSerial.println("USB power detected - sleep disabled");
    }
    
}

/****************************************************************************************************
 * Update UI with initial sensor data
 * Must be called after battery and IMU initialization
 */
void updateInitialUI() {
    // Update UI with all sensor data
    updateBatteryInfoUI();
    updateMotionStatusUI();
    
    // Force a complete screen refresh before WiFi connection
    USBSerial.println("Forcing full UI refresh before WiFi connection...");
    for (int i = 0; i < 15; i++) {
        lv_timer_handler();
        delay(5);
    }
}

/****************************************************************************************************
 * Configure WiFi Network Priority
 * Sets primary and secondary network based on WIFI_PRIORITY
 */
void configureWiFiPriority() {  
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
}

/****************************************************************************************************
 * Initialize WiFi Connection
 * Attempts connection with fallback to secondary network
 * 
 * IMPORTANT: The following must be done in setup() BEFORE calling this function:
 * 1. WiFi.mode(WIFI_OFF) - to disable WiFi radio during hardware init
 * 2. configureWiFiPriority() - to set primary/secondary network variables
 */
void initWiFi() {
    USBSerial.println("--- Initializing WiFi ---");
    
    // Attempt connection (network priority was already configured in setup)
    if (!attemptWiFiConnection()) {
        USBSerial.println("WiFi connection failed (unexpected state).");
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
}

/****************************************************************************************************
 * Initialize MQTT Connection
 * Attempts initial connection with retry logic
 */
void initMQTT() {
    USBSerial.println("--- Initializing MQTT ---");
    
    if (WiFi.status() != WL_CONNECTED) {
        USBSerial.println("WARNING: Cannot initialize MQTT - WiFi not connected");
        return;  // Exit early - no point trying MQTT without WiFi
    }
    
    USBSerial.println("Attempting initial MQTT connection...");

    mqttClient.setBufferSize(512);  // Add this before MQTT connection attempts
    
    for (int i = 0; i < 3; i++) {
        checkMQTT(true);  // Bypass rate limiting during setup
        
        if (mqttClient.connected()) {
            USBSerial.println("Initial MQTT connection successful!");
            return;  // Exit function immediately on success
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
}

/****************************************************************************************************
 * Finalize Setup
 * Updates UI with final status and prepares for main loop
 */
void finalizeSetup() {
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
//***************************************************************************************************
void setup() {
  USBSerial.begin(115200);
  // delay(3000); // Allow time for Serial to initialize and see debug messages

  esp_sleep_wakeup_cause_t wakeReason = esp_sleep_get_wakeup_cause();

  if (wakeReason == ESP_SLEEP_WAKEUP_EXT0) {
    // This is a Deep Sleep Wake. The ESP32's I2C driver is stuck.
    // We MUST reset it to prevent the "i2c driver install error".
    USBSerial.println("Deep Sleep Wake detected. Performing I2C driver reset...");
    Wire.end();
    delay(10);
  }
  // For any other type of boot (Cold or Shutdown), we DO NOT call Wire.end().

  Wire.begin(IIC_SDA, IIC_SCL);
  delay(50); 
  
  // Debug delay to allow printing to serial monitor.  Comment out for production.
  delay(900); // Allow time for hardware to stabilize

  USBSerial.println("\n--- Board is starting up ---");
  
  WiFi.mode(WIFI_OFF);

  initPMIC(); // Initialize PMIC - critical for power management
  
  initIOExpander(); // Initialize I/O Expander - needed for display control

  initTouch();  // Initialize Touch Controller

  initDisplay();  // Initialize Display Hardware

  initLVGL(); // Initialize LVGL

  initUIHandlers(); // Initialize UI Event Handlers

  // Check PSRAM availability
  if (!initPSRAM()) {
      USBSerial.println("FATAL: PSRAM not available - cannot continue");
      while(1) { delay(1000); }
  }  

  initJPEGDecoder();  // Initialize JPEG Decoder

  initIMU();  // Initialize IMU/Motion Sensor  

  initBattery();  // Initialize Battery Monitoring

  updateInitialUI();  // Update UI with initial sensor data

  configureWiFiPriority();  // Configure WiFi priority (must be done before initWiFi)

  initWiFi(); // Initialize WiFi (failure handling in attemptWiFiConnection)
  
  initMQTT(); // Initialize MQTT (will retry in loop if needed)

  reinitializeMotionBaseline();  

  finalizeSetup();
}


//***************************************************************************************************
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

  // --- Task 1: Update motion state and calibration ---
  updateImuData();  // Read IMU data once per loop
  updateMotionState(); // Just update the motion flag, no shutdown decision
  updateCalibration();  // Update calibration logic

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

  // --- Task 7: Update inclinometer display at 2 Hz
  unsigned long current_millis = millis();
  if (current_millis - last_inclinometer_display_update >= INCLINOMETER_DISPLAY_INTERVAL) {
    updateInclinometerDisplay();
    last_inclinometer_display_update = current_millis;
  }

  // --- Task 8: Transmit motion MQTT if connected --- 
  if (ENABLE_MOTION_MQTT && g_isCurrentlyMoving && mqttClient.connected()) {
    if (millis() - lastMotionTXTime > MOTION_TIMEOUT) {
      mqttClient.publish(MOTION_TOPIC, "1");
      lastMotionTXTime = millis();
      USBSerial.println("TX motion MQTT: Moving (periodic)");
    }
  }

  // --- Task 9: Send IMU data via MQTT
  unsigned long currentMillis = millis();
  if (currentMillis - lastImuPrintTime >= IMU_PRINT_INTERVAL) {
    lastImuPrintTime = currentMillis;
    readImuData();
  }

  #ifndef TEST_POWER
  // In normal operation mode - check for inactivity to trigger sleep/shutdown 

  // --- Task 10: Check for user inactivity to trigger deep sleep if allowed to sleep ---
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

  #else
  // In TEST_POWER mode - with USB power and no battery to test power states
  // --- Task 10: Check for user inactivity to trigger deep sleep or shutdown ---
  if (millis() - lastActivityTime > INACTIVITY_TIMEOUT) {
      if (!g_isCurrentlyMoving) {
          // No touch for 30s AND stationary → SHUTDOWN to save max power
          shutdownInitiated = true;
          goToShutdown();   // 1 mA power consumption wake up by button
      } else {
          // No touch for 30s BUT still moving → SLEEP for quick wake
          goToDeepSleep();  // 5 mA power consumption wake up by touch
      }      
  }

  #endif

  delay(1); // A small delay to yield time to other tasks.
}