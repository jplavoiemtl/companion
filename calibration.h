// imu_calibration.h
#pragma once

#include <Arduino.h>

// ============================================================================
// PUBLIC CALIBRATION API
// ============================================================================

// How calibration is progressing (for UI feedback)
enum CalibState {
  CALIB_IDLE = 0,
  CALIB_GRAVITY_SAMPLING, // Measuring stationary gravity & scale factor
  CALIB_FORWARD_SAMPLING, // Measuring linear acceleration (driving)
  CALIB_READY_TO_COMPUTE, // Sampling done, ready to calculate matrix
  CALIB_DONE,             // Calibration computed successfully
  CALIB_ERROR             // Calibration failed (instability, timeout, bad data)
};

// Call once in setup() to initialize internal state.
void calibInit();

// ----------------------------------------------------------------------------
// NVS STORAGE (Load / Save)
// ----------------------------------------------------------------------------

// Try to load whatever calibration data is available in NVS.
// This function is flexible:
// 1. It looks for Gravity Vector & Scale Factor.
// 2. It looks for Rotation Matrix.
// Returns true if at least the Gravity/Scale was loaded successfully.
bool calibLoadFromNvs();

// Save ONLY the Gravity Vector and Scale Factor to NVS.
// Call this immediately after calibStartGravity() finishes successfully.
// This allows you to save the "Garage" calibration before driving.
bool calibSaveGravityToNvs();

// Save ONLY the Rotation Matrix to NVS.
// Call this after calibComputeRotation() returns true.
bool calibSaveRotationToNvs();

// ----------------------------------------------------------------------------
// STEP 1: STATIONARY (Gravity + Scale) CALIBRATION
// ----------------------------------------------------------------------------

// Start stationary calibration (car parked, module fixed).
// This will:
// 1. Clear previous gravity averages.
// 2. Sample for CALIB_STATIONARY_DURATION_MS.
// 3. Calculate the Gravity Vector (down direction).
// 4. Calculate the Scale Factor (magnitude of 1G for this sensor).
void calibStartGravity();

// ----------------------------------------------------------------------------
// STEP 2: FORWARD (Driving) CALIBRATION
// ----------------------------------------------------------------------------

// Start forward calibration (user drives straight).
// IMPORTANT RULE: The user must ACCELERATE FIRST from a stop.
// The algorithm uses the first movement to define "Backward Force".
// Any subsequent braking (Forward Force) will be automatically inverted (rectified)
// so it improves the calibration data instead of cancelling it out.
//
// Prerequisite: A valid gravity vector must exist (from NVS or Step 1).
void calibStartForward();

// Abort any ongoing calibration step and reset accumulators.
void calibAbort();

// ----------------------------------------------------------------------------
// MAIN LOOP UPDATE
// ----------------------------------------------------------------------------

// This must be called from your main loop at IMU rate (e.g., 100Hz),
// with the *raw* accelerometer values (sensor frame).
// accel[0] = ax, accel[1] = ay, accel[2] = az
// (Do not divide by gravity or scale here; pass raw data).
void calibUpdate(const float accel[3]);

// ----------------------------------------------------------------------------
// COMPUTATION
// ----------------------------------------------------------------------------

// After forward calibration is done (CALIB_READY_TO_COMPUTE),
// call this to calculate the full rotation matrix.
// It uses the stored gravity + forward samples (same math as the Python script).
// Returns true on success; also updates the global ROTATION_MATRIX.
bool calibComputeRotation();

// ----------------------------------------------------------------------------
// QUERY / UI HELPERS
// ----------------------------------------------------------------------------

CalibState calibGetState();

bool calibHasGravity();       // Is a valid gravity vector available?
bool calibHasRotation();      // Is a valid rotation matrix available?

// Get the calculated Scale Factor.
// Use this in your main loop to normalize raw readings to 1.0G.
// Example: float g_force = raw_reading / calibGetScaleFactor();
float calibGetScaleFactor();

// Get remaining time (ms) in the current sampling window.
// Returns 0 if not currently sampling.
uint32_t calibGetRemainingMs();

// Copy the current gravity vector (down) out
// (3 floats: [gx, gy, gz] in sensor coordinates).
bool calibGetGravity(float out[3]);

// Copy the current rotation matrix out (3x3).
// Row 0: display_x (vertical)
// Row 1: display_y (horizontal)
// Row 2: up (reference)
bool calibGetRotationMatrix(float out[3][3]);

// Get the progress of the current calibration step (0 to 100%).
// Returns 0 if not currently sampling.
uint8_t calibGetProgressPercent();

// ----------------------------------------------------------------------------
// MQTT REPORTING
// ----------------------------------------------------------------------------

// Define the Callback signature: takes a topic and a payload
typedef void (*MqttCallback)(const char* topic, const char* payload);

// Register the callback function (connects Library to Main Sketch)
void calibSetMqttCallback(MqttCallback cb);

// Set the Car Unit flag (call this in setup based on #ifdef CAR)
void calibSetCarUnit(bool isCar);

// Manually trigger an MQTT report (e.g., at startup).
// Sends with "new_calib": false
void calibReportStatus();

// ----------------------------------------------------------------------------
// GLOBAL MATRIX USED BY FIRMWARE
// ----------------------------------------------------------------------------
// This is the matrix your code should use in applyInertialTransform().
// It is initialized with your "factory" defaults, and updated dynamically
// whenever calibComputeRotation() succeeds or calibLoadFromNvs() loads one.
extern float ROTATION_MATRIX[3][3];