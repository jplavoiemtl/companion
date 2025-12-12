#include "calibration.h"
#if defined(__has_include) && __has_include("secrets_private.h")
#include "secrets_private.h"
#else
#include "secrets.h"
#endif
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
#include "src/image/image_fetcher.h"
#include "src/imu/imu_module.h"


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

// --- MQTT Reconnection Management ---
unsigned long lastMqttAttempt = 0;
const unsigned long MQTT_RECONNECT_INTERVAL = 15000;  // 15 seconds between reconnection attempts
bool mqttConnection = false;
bool mqttSuccess = false;                 //MQTT succeeded once at start to keep reconnecting only if successful


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
unsigned long last_inclinometer_display_update = 0;
const unsigned long INCLINOMETER_DISPLAY_INTERVAL = 500; // 200ms = 5 Hz, 500


// --- Global Objects ---
HWCDC USBSerial;
XPowersAXP2101 pmic;
ESP_IOExpander *expander = NULL;
SemaphoreHandle_t i2c_mutex = NULL;

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
  
  // 1. DEFAULT STATE: No touch
  data->state = LV_INDEV_STATE_REL;

  // Safety Check: If the hardware pin is stuck LOW (Active), force the flag to TRUE.
  // This un-sticks the touch controller if a previous I2C read failed/timed-out.
  if (digitalRead(TP_INT) == LOW) {
    FT3168->IIC_Interrupt_Flag = true;
  }
  
  // 2. CHECK INTERRUPT FIRST - Don't use I2C bus unless hardware signaled a touch
  if (FT3168->IIC_Interrupt_Flag == true) {
    
    // START MUTEX PROTECTION
    if (i2c_mutex && xSemaphoreTakeRecursive(i2c_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {

      // Ask the chip: "How many fingers are actually on the screen?"
      int32_t touchPoints = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER);

      // If the interrupt fired (noise) but the chip sees 0 fingers...
      if (touchPoints == 0) {
          // ...It was a ghost! Clear the flag and ignore it.
          FT3168->IIC_Interrupt_Flag = false;
          xSemaphoreGiveRecursive(i2c_mutex);
          data->state = LV_INDEV_STATE_REL;
          return; 
      }

      // 3. READ COORDINATES ONLY NOW - Prevents I2C collisions with IMU/PMIC
      int32_t touchX = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
      int32_t touchY = FT3168->IIC_Read_Device_Value(FT3168->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);
      
      // 4. CLEAR FLAG AFTER READING - Prevents missing events during I2C reads
      FT3168->IIC_Interrupt_Flag = false;

      xSemaphoreGiveRecursive(i2c_mutex);
        
      // 5. Filter B: Reject out-of-bounds - garbage data often exceeds screen dimensions
      if (touchX < 0 || touchX >= screenWidth || touchY < 0 || touchY >= screenHeight) {
        return;
      }
      
      // 6. VALID TOUCH - All filters passed
      data->state = LV_INDEV_STATE_PR;
      data->point.x = touchX;
      data->point.y = touchY;

    } // END MUTEX PROTECTION

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
// GRAVITY TIMER CALLBACK
// ===========================================================
void onGravityTimerExpired(lv_timer_t * timer) {
    bool* pIsStabilizing = (bool*)timer->user_data;
    if (pIsStabilizing) *pIsStabilizing = false;
    
    // Start Logic
    calibStartGravity();
    
    // UI Update: Active Sampling
    lv_label_set_text(ui_calibStatusLabel, "Sampling... Do not move");
    lv_obj_set_style_text_color(ui_calibStatusLabel, lv_color_hex(0x00BFFF), LV_PART_MAIN); // Deep Sky Blue
    lv_bar_set_value(ui_calibProgressBar, 0, LV_ANIM_OFF);
    
    USBSerial.println("[UI] Stabilization complete. Sampling started.");
}


//***************************************************************************************************
// GRAVITY BUTTON EVENT HANDLER
// ===========================================================
void gravityCalButton_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    static bool isStabilizing = false;

    if (code == LV_EVENT_CLICKED) {
        if (isStabilizing) return;

        CalibState cs = calibGetState();
        if (cs == CALIB_GRAVITY_SAMPLING || 
            cs == CALIB_FORWARD_SAMPLING || 
            cs == CALIB_READY_TO_COMPUTE) {
            return; 
        }

        isStabilizing = true;
        
        // UI Update: Immediate Feedback
        lv_label_set_text(ui_calibStatusLabel, "Stabilizing... Keep Still");
        lv_obj_set_style_text_color(ui_calibStatusLabel, lv_color_hex(0xFFB700), LV_PART_MAIN); // Orange
        
        USBSerial.println("[UI] Gravity Button: Waiting 1s...");

        lv_timer_t * timer = lv_timer_create(onGravityTimerExpired, 1000, &isStabilizing);
        lv_timer_set_repeat_count(timer, 1);
    }
}


//***************************************************************************************************
// FORWARD TIMER CALLBACK
// ===========================================================
void onForwardTimerExpired(lv_timer_t * timer) {
    bool* pIsStabilizing = (bool*)timer->user_data;
    if (pIsStabilizing) *pIsStabilizing = false;
    
    // Start Logic
    calibStartForward();
    
    // UI Update: Active Sampling
    lv_label_set_text(ui_calibStatusLabel, "ACCELERATE NOW!");
    lv_obj_set_style_text_color(ui_calibStatusLabel, lv_color_hex(0x00BFFF), LV_PART_MAIN); // Deep Sky Blue
    lv_bar_set_value(ui_calibProgressBar, 0, LV_ANIM_OFF);
    
    USBSerial.println("[UI] Forward Sampling started.");
}


// ===========================================================
// FORWARD BUTTON EVENT HANDLER
// ===========================================================
void forwardCalButton_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    static bool isStabilizing = false;

    if (code == LV_EVENT_CLICKED) {
        if (isStabilizing) return;

        CalibState cs = calibGetState();
        if (cs == CALIB_GRAVITY_SAMPLING || 
            cs == CALIB_FORWARD_SAMPLING || 
            cs == CALIB_READY_TO_COMPUTE) {
            return;
        }

        // PREREQUISITE CHECK
        if (!calibHasGravity()) {
            // UI Update: Error Feedback
            lv_label_set_text(ui_calibStatusLabel, "Error: Do Gravity Step First!");
            lv_obj_set_style_text_color(ui_calibStatusLabel, lv_color_hex(0xFF0000), LV_PART_MAIN); // Red
            USBSerial.println("[UI] Error: Missing Gravity Data.");
            return;
        }

        isStabilizing = true;
        
        // UI Update: Immediate Feedback
        lv_label_set_text(ui_calibStatusLabel, "Get Ready... Accel in 1s");
        lv_obj_set_style_text_color(ui_calibStatusLabel, lv_color_hex(0xFFB700), LV_PART_MAIN); // Orange
        
        USBSerial.println("[UI] Forward Button: Waiting 1s...");

        lv_timer_t * timer = lv_timer_create(onForwardTimerExpired, 1000, &isStabilizing);
        lv_timer_set_repeat_count(timer, 1);
    }
}


//***************************************************************************************************
void updateCalibration() {
  static CalibState prevCalibState = CALIB_IDLE;
  CalibState currentState = calibGetState();

  // ---------------------------------------------------------
  // 1. HANDLE STATE TRANSITIONS (Success/Failure)
  // ---------------------------------------------------------
  if (currentState != prevCalibState) {
      
      // --- CASE 1: Gravity Calibration Finished ---
      if (prevCalibState == CALIB_GRAVITY_SAMPLING && currentState == CALIB_IDLE) {
          if (calibHasGravity()) {
              calibSaveGravityToNvs(); 
              USBSerial.println("[Main] Gravity/Scale Saved!");
              
              // UI Success
              lv_label_set_text(ui_calibStatusLabel, "Gravity Step Complete!");
              lv_obj_set_style_text_color(ui_calibStatusLabel, lv_color_hex(0x00FF00), LV_PART_MAIN); // Green
              lv_bar_set_value(ui_calibProgressBar, 100, LV_ANIM_ON);
          }
      }

      // --- CASE 2: Forward Calibration Finished ---
      else if (currentState == CALIB_READY_TO_COMPUTE) {
          if (calibComputeRotation()) {
              calibSaveRotationToNvs(); 
              USBSerial.println("[Main] Rotation Saved!");
              
              // UI Success
              lv_label_set_text(ui_calibStatusLabel, "Calibration Success!");
              lv_obj_set_style_text_color(ui_calibStatusLabel, lv_color_hex(0x00FF00), LV_PART_MAIN); // Green
              lv_bar_set_value(ui_calibProgressBar, 100, LV_ANIM_ON);
          } else {
              USBSerial.println("[Main] Computation Failed.");
              // UI Logic Error
              lv_label_set_text(ui_calibStatusLabel, "Computation Failed");
              lv_obj_set_style_text_color(ui_calibStatusLabel, lv_color_hex(0xFF0000), LV_PART_MAIN); // Red
              lv_bar_set_value(ui_calibProgressBar, 0, LV_ANIM_OFF);
          }
      } 
      
      // --- CASE 3: Error Occurred ---
      else if (currentState == CALIB_ERROR) {
            USBSerial.println("[Main] Calibration Error.");
            
            // UI Error
            lv_label_set_text(ui_calibStatusLabel, "Calibration Failed - Try Again");
            lv_obj_set_style_text_color(ui_calibStatusLabel, lv_color_hex(0xFF0000), LV_PART_MAIN); // Red
            lv_bar_set_value(ui_calibProgressBar, 0, LV_ANIM_OFF);
      }

      prevCalibState = currentState;
  }
  
  // ---------------------------------------------------------
  // 2. HANDLE PROGRESS BAR (Continuous Update)
  // ---------------------------------------------------------
  // Simplified: Let the library do the math!
  if (currentState == CALIB_GRAVITY_SAMPLING || currentState == CALIB_FORWARD_SAMPLING) {
      uint8_t percent = calibGetProgressPercent();
      lv_bar_set_value(ui_calibProgressBar, percent, LV_ANIM_OFF);
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
// ========== INCLINOMETER FUNCTIONS ==========
//***************************************************************************************************
// Event handler for Inclinometer/Calibration Screen
// Resets the calibration UI when the user enters the screen
void screenInclinometer_event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_SCREEN_LOADED) {
        // Only reset if we are NOT currently in the middle of a calibration
        // (Just in case the user somehow switched screens during the 5s wait)
        CalibState cs = calibGetState();
        
        if (cs == CALIB_IDLE || cs == CALIB_DONE || cs == CALIB_ERROR) {
            USBSerial.println("Calibration Screen Loaded: Resetting UI elements");
            
            // Reset Label to Neutral
            lv_label_set_text(ui_calibStatusLabel, "Ready to Calibrate");
            lv_obj_set_style_text_color(ui_calibStatusLabel, lv_color_hex(0xFFFFFF), LV_PART_MAIN); // White
            
            // Reset Progress Bar
            lv_bar_set_value(ui_calibProgressBar, 0, LV_ANIM_OFF);
        }
    }
}

//***************************************************************************************************
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
void Arduino_IIC_Touch_Interrupt(void) {
  if (FT3168) { // Safety check: ensure object exists before using it
    FT3168->IIC_Interrupt_Flag = true;
  }
}


//***************************************************************************************************
void adcOn() {
  if (i2c_mutex && xSemaphoreTakeRecursive(i2c_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    pmic.enableTemperatureMeasure();
    pmic.enableBattDetection();
    pmic.enableVbusVoltageMeasure();
    pmic.enableBattVoltageMeasure();
    pmic.enableSystemVoltageMeasure();
    xSemaphoreGiveRecursive(i2c_mutex);  
  }
  adc_switch = true;
}


//***************************************************************************************************
void adcOff() {
  if (i2c_mutex && xSemaphoreTakeRecursive(i2c_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    pmic.disableTemperatureMeasure();
    pmic.disableBattDetection();
    pmic.disableVbusVoltageMeasure();
    pmic.disableBattVoltageMeasure();
    pmic.disableSystemVoltageMeasure();
    xSemaphoreGiveRecursive(i2c_mutex);  
  }
  adc_switch = false;
}


//***************************************************************************************************
void updatePowerStatus() {
  static bool prevVbusPresent = false;  // Track previous state to detect transitions
  
  // START MUTEX PROTECTION
  if (i2c_mutex && xSemaphoreTakeRecursive(i2c_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      vbusPresent = pmic.isVbusIn();
      batteryConnected = pmic.isBatteryConnect();
      xSemaphoreGiveRecursive(i2c_mutex);
  }

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

  // START MUTEX PROTECTION for the rest
  if (i2c_mutex && xSemaphoreTakeRecursive(i2c_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
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
    xSemaphoreGiveRecursive(i2c_mutex);
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

// IMU helper moved to src/imu/imu_module.cpp


// IMU MQTT publisher moved to src/imu/imu_module.cpp





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
    lv_obj_add_event_cb(ui_Screen3, screen3_event_handler, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_InclinometerScreen, screenInclinometer_event_handler, LV_EVENT_SCREEN_LOADED, NULL);
    USBSerial.println("  Screen event handlers registered");  

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
 * Initialize IMU/Motion Sensor (QMI8658)
 * Configures accelerometer and Wake-on-Motion
 */
// IMU functions moved to src/imu/imu_module.cpp

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
  i2c_mutex = xSemaphoreCreateRecursiveMutex();

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
  // Set I2C timeout to 50 milliseconds. 
  // If a sensor hangs, Wire will return an error instead of freezing the code.
  Wire.setTimeOut(50); 
  delay(50); 
  
  // Debug delay to allow printing to serial monitor.  Comment out for production.
  // delay(900); // Allow time for hardware to stabilize

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

  ImageFetcherConfig imageCfg{
    screenWidth,
    screenHeight,
    ui_Screen1,
    ui_Screen2,
    ui_Screen3,
    ui_InclinometerScreen,
    ui_imgScreen2Background
  };
  imageFetcherInit(imageCfg);

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
  updateGMeterDisplay(imuGetAccelInertialVert(), imuGetAccelInertialHoriz());
  updateCalibration();  // Update calibration logic

  // --- Task 2: Handle MQTT communications if connected ---
  if (WiFi.status() == WL_CONNECTED) {
    mqttClient.loop();  // Always call loop() to maintain connection
    if(mqttSuccess) {
      checkMQTT();  // Only attempt reconnection if initial connection succeeded
    }
  }

  // --- Task 3: Process HTTP response if in progress
  imageFetcherLoop();

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

  // --- Task 6: Update inclinometer display at 2 Hz
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