// imu_calibration.h
#pragma once

#include <Arduino.h>

// =========================
// Public calibration API
// =========================

// How calibration is progressing (for UI)
enum CalibState {
  CALIB_IDLE = 0,
  CALIB_GRAVITY_SAMPLING,
  CALIB_FORWARD_SAMPLING,
  CALIB_READY_TO_COMPUTE,
  CALIB_DONE,
  CALIB_ERROR
};

// Call once in setup()
void calibInit();

// Try to load previously saved gravity, scale, & rotation from NVS.
// Returns true if all data loaded successfully.
bool calibLoadFromNvs();

// Save current gravity, scale, & rotation to NVS.
// (You normally call this after a successful compute step.)
bool calibSaveToNvs();

// ---- Stationary (gravity + scale) calibration ----

// Start stationary calibration (car parked, module fixed).
// This will:
// 1. Clear previous gravity averages.
// 2. Calculate the Gravity Vector (down).
// 3. Calculate the Scale Factor (magnitude of gravity).
void calibStartGravity();

// ---- Forward calibration ----

// Start forward calibration (user drives straight).
// RULE: User must ACCELERATE FIRST from a stop.
// Subsequent braking will be mathematically rectified to help calibration.
// Requires a valid gravity vector (from current session or NVS) to start.
void calibStartForward();

// Abort any ongoing calibration step.
void calibAbort();

// This must be called from your main loop at IMU rate,
// with the *raw* accelerometer values (sensor frame).
// accel[0] = ax, accel[1] = ay, accel[2] = az (in whatever units sensor outputs).
void calibUpdate(const float accel[3]);

// After forward calibration is done, compute the full rotation matrix.
// Returns true on success; also updates the global matrix used by your firmware.
bool calibComputeRotation();

// ---- Query / UI helpers ----
CalibState calibGetState();
bool calibHasGravity();       // gravity vector available
bool calibHasRotation();      // rotation matrix available

// Get the calculated scale factor (1G magnitude).
// Use this in your main loop: raw_val / scale_factor = Gs.
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

// -------------------------------
// Rotation matrix used by firmware
// -------------------------------
//
// This is the matrix your code should use in applyInertialTransform().
// It is initialized with your current "factory" matrix, and updated
// whenever calibComputeRotation() succeeds or calibLoadFromNvs() loads one.
//
extern float ROTATION_MATRIX[3][3];