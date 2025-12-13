#include "imu_module.h"
#if defined(__has_include) && __has_include("secrets_private.h")
#include "secrets_private.h"
#else
#include "secrets.h"
#endif
#include "pin_config.h"
#include <math.h>

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

// Stationary detection timing
const unsigned long MOTION_CHECK_INTERVAL = 20;  // Motion state update interval (10ms) 10

// Motion state
bool g_isCurrentlyMoving = false;

// Timing helpers
static unsigned long lastMotionCheckTime = 0;
static unsigned long lastMotionTime = 0;
static int startupIgnoreCount = 20; // Ignore first 20 intervals (2 seconds) for sensor stabilization

// Forward declarations (internal)
static void applyInertialTransform(float sensor[3], float display[3]);
static void integrateGyroAngles(float dt, float gyro_vert, float gyro_horiz, float gyro_up);
static void calculateAccelAngles(float &pitch_accel, float &roll_accel);
static void computeFusedAngles(float dt, float fwd_g, float &out_pitch, float &out_roll);
static void updateInclinometerWithFreeze(float dt);
static void initializeInclinometer();
static void calculateGyroBias();

// Externs provided by companion.ino
extern float ROTATION_MATRIX[3][3];
extern const unsigned long MOTION_TIMEOUT;
extern unsigned long lastMotionTXTime;
extern void myCalibMqttSender(const char* topic, const char* payload);

//***************************************************************************************************
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
    USBSerial.print("Sampling freq: ");
    USBSerial.print(sampling_frequency);
    USBSerial.println(" Hz"); 
    USBSerial.println();
 
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

  }
}

//***************************************************************************************************
void updateImuData() {

  // START MUTEX PROTECTION
  if (i2c_mutex && xSemaphoreTakeRecursive(i2c_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {

    if (qmi.getDataReady()) {
      qmi.getAccelerometer(acc.x, acc.y, acc.z);
      qmi.getGyroscope(gyr.x, gyr.y, gyr.z);

      // Release Mutex immediately after reading
      xSemaphoreGiveRecursive(i2c_mutex);       
      
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
    } else {
        // If data wasn't ready, we still must release the mutex
        xSemaphoreGiveRecursive(i2c_mutex);
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
    else                    tau = 2.0f;   // 1.5

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

float imuGetAccelInertialVert() {
  return acc_inertial.vert;
}

float imuGetAccelInertialHoriz() {
  return acc_inertial.horiz;
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
    
    // 1. Protect the Ready Check Loop
    while (!data_ready && (millis() - start_wait < 50)) {
      
      // Take Mutex to check status
      if (i2c_mutex && xSemaphoreTakeRecursive(i2c_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          data_ready = qmi.getDataReady();
          xSemaphoreGiveRecursive(i2c_mutex); // Give back immediately
      }

      if (!data_ready) {
        delay(1); // Wait a bit before trying again
      }
    }
    
    if (data_ready) {
      // 2. Protect the Data Read
      if (i2c_mutex && xSemaphoreTakeRecursive(i2c_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          
          // Read raw gyro - DON'T transform yet!
          qmi.getGyroscope(gyr.x, gyr.y, gyr.z);
          
          xSemaphoreGiveRecursive(i2c_mutex); // Give back immediately

          // Math can happen outside the mutex
          sum_x += gyr.x;
          sum_y += gyr.y;
          sum_z += gyr.z;
          valid_samples++;
      }
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

