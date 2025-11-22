// imu_calibration.cpp
#include "calibration.h"
#include <Preferences.h>
#include <math.h>

// ============================================================================
// NVS CONFIGURATION
// ============================================================================

static Preferences prefs;
static const char* NVS_NAMESPACE = "imuCal";
static const char* NVS_KEY_GRAV  = "grav";   // 3 floats (gravity vector)
static const char* NVS_KEY_SCALE = "scale";  // 1 float  (scale factor)
static const char* NVS_KEY_ROT   = "rot";    // 9 floats (rotation matrix)

// ============================================================================
// PUBLIC GLOBAL MATRIX (Factory Defaults)
// ============================================================================
// Initialize with your existing "good" matrix as a default.
// This ensures the system works even if NVS is empty.
float ROTATION_MATRIX[3][3] = {
  {-0.037424f, -0.135703f,  0.990042f},  // display_x (vertical)
  {-0.998829f, -0.025327f, -0.041228f},  // display_y (horizontal)
  { 0.030670f, -0.990426f, -0.134596f}   // up (reference)
};

// ============================================================================
// INTERNAL STATE VARIABLES
// ============================================================================

static CalibState g_state = CALIB_IDLE;

// Gravity accumulation (stationary)
static float g_gravitySum[3] = {0, 0, 0};
static uint32_t g_gravityCount = 0;
static float g_gravityRaw[3] = {0, 0, 0};  // mean gravity vector (down)
static bool g_hasGravity = false;

// Scale Factor (Magnitude of Gravity)
static float g_scaleFactor = 1.0f; // Default to 1.0 if not calibrated

// Forward accumulation (driving)
static float g_forwardSum[3] = {0, 0, 0};
static uint32_t g_forwardCount = 0;
static bool g_hasForward = false;

// Sampling timing
static uint32_t g_sampleStartMs = 0;
static uint32_t g_sampleDurationMs = 0;

// Flags for successful rotation matrix computation
static bool g_hasRotation = false;

// ============================================================================
// CONSTANTS (Durations & Thresholds)
// ============================================================================

// Duration to sample data for each step
static const uint32_t CALIB_STATIONARY_DURATION_MS = 5000;  // 5s parked
static const uint32_t CALIB_FORWARD_DURATION_MS    = 10000;  // 10s driving

// Minimum number of samples required to consider the data valid
static const uint32_t CALIB_MIN_STATIONARY_SAMPLES = 1000;
static const uint32_t CALIB_MIN_FORWARD_SAMPLES    = 1000;

// Threshold for accepting forward linear accel sample (in Gs).
// This filters out engine vibration when idling before the car moves.
static const float CALIB_FORWARD_MIN_G = 0.05f;

// ============================================================================
// VECTOR MATH HELPERS (Inline for speed)
// ============================================================================

static inline void vecClear(float v[3]) {
  v[0] = v[1] = v[2] = 0.0f;
}

static inline void vecAdd(const float a[3], float b[3]) {
  b[0] += a[0];
  b[1] += a[1];
  b[2] += a[2];
}

static inline void vecCopy(const float src[3], float dst[3]) {
  dst[0] = src[0];
  dst[1] = src[1];
  dst[2] = src[2];
}

static inline float vecDot(const float a[3], const float b[3]) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

static inline float vecNorm(const float v[3]) {
  return sqrtf(vecDot(v, v));
}

static inline bool vecNormalize(float v[3]) {
  float n = vecNorm(v);
  if (n < 1e-6f) return false;
  v[0] /= n; v[1] /= n; v[2] /= n;
  return true;
}

static inline void vecScale(const float v[3], float s, float out[3]) {
  out[0] = v[0] * s; out[1] = v[1] * s; out[2] = v[2] * s;
}

static inline void vecSub(const float a[3], const float b[3], float out[3]) {
  out[0] = a[0] - b[0]; out[1] = a[1] - b[1]; out[2] = a[2] - b[2];
}

static inline void vecCross(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1]*b[2] - a[2]*b[1];
  out[1] = a[2]*b[0] - a[0]*b[2];
  out[2] = a[0]*b[1] - a[1]*b[0];
}

// ============================================================================
// NVS LOAD / SAVE IMPLEMENTATION
// ============================================================================

bool calibLoadFromNvs() {
  if (!prefs.begin(NVS_NAMESPACE, true)) { // read-only
    Serial.println("[Calib] NVS begin failed (load).");
    return false;
  }

  Serial.println("[Calib] ----------------------------------------");
  Serial.println("[Calib] Loading Calibration from NVS...");

  // 1. Attempt to load Gravity & Scale (Can exist without Rotation)
  size_t lenG = prefs.getBytesLength(NVS_KEY_GRAV);
  size_t lenS = prefs.getBytesLength(NVS_KEY_SCALE);

  if (lenG == sizeof(float)*3 && lenS == sizeof(float)) {
    float g_tmp[3];
    float s_tmp;
    prefs.getBytes(NVS_KEY_GRAV, g_tmp, sizeof(g_tmp));
    prefs.getBytes(NVS_KEY_SCALE, &s_tmp, sizeof(s_tmp));

    // Sanity check: Scale factor shouldn't be zero or absurd
    if (s_tmp > 0.1f && s_tmp < 20000.0f) {
      vecCopy(g_tmp, g_gravityRaw);
      g_scaleFactor = s_tmp;
      g_hasGravity = true;
      
      // --- DEBUG PRINT GRAVITY ---
      Serial.println("[Calib] [OK] Gravity & Scale loaded:");
      Serial.printf("    Scale Factor: %.4f\n", g_scaleFactor);
      Serial.printf("    Gravity Vec:  [% .4f, % .4f, % .4f]\n", 
                    g_gravityRaw[0], g_gravityRaw[1], g_gravityRaw[2]);

    } else {
      Serial.printf("[Calib] [ERR] NVS scale factor invalid (%.4f), ignoring.\n", s_tmp);
    }
  } else {
    Serial.println("[Calib] [INFO] No valid Gravity/Scale in NVS (New unit?).");
  }

  // 2. Attempt to load Rotation Matrix (Can be loaded independently if Gravity exists)
  size_t lenR = prefs.getBytesLength(NVS_KEY_ROT);
  
  if (lenR == sizeof(float)*9) {
    float R_tmp[3][3];
    prefs.getBytes(NVS_KEY_ROT, R_tmp, sizeof(R_tmp));

    // Copy into global matrix
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        ROTATION_MATRIX[i][j] = R_tmp[i][j];
      }
    }
    g_hasRotation = true;

    // --- DEBUG PRINT ROTATION ---
    Serial.println("[Calib] [OK] Rotation Matrix loaded:");
    Serial.printf("    Row 0 (Vert): [% .4f, % .4f, % .4f]\n", 
                  ROTATION_MATRIX[0][0], ROTATION_MATRIX[0][1], ROTATION_MATRIX[0][2]);
    Serial.printf("    Row 1 (Horz): [% .4f, % .4f, % .4f]\n", 
                  ROTATION_MATRIX[1][0], ROTATION_MATRIX[1][1], ROTATION_MATRIX[1][2]);
    Serial.printf("    Row 2 (Up):   [% .4f, % .4f, % .4f]\n", 
                  ROTATION_MATRIX[2][0], ROTATION_MATRIX[2][1], ROTATION_MATRIX[2][2]);

  } else {
    Serial.println("[Calib] [INFO] No valid Rotation Matrix in NVS (Using defaults).");
  }

  Serial.println("[Calib] ----------------------------------------");

  prefs.end();
  
  // We return true if we at least have the basic gravity/scale data.
  // It is valid to have Gravity but no Rotation (if user only did step 1).
  return g_hasGravity;
}

bool calibSaveGravityToNvs() {
  if (!g_hasGravity) {
    Serial.println("[Calib] Cannot save Gravity: data invalid.");
    return false;
  }
  if (!prefs.begin(NVS_NAMESPACE, false)) return false; // read-write

  size_t writtenG = prefs.putBytes(NVS_KEY_GRAV, g_gravityRaw, sizeof(g_gravityRaw));
  size_t writtenS = prefs.putBytes(NVS_KEY_SCALE, &g_scaleFactor, sizeof(g_scaleFactor));
  
  prefs.end();

  bool ok = (writtenG == sizeof(g_gravityRaw) && writtenS == sizeof(g_scaleFactor));
  Serial.printf("[Calib] Saved Gravity & Scale to NVS (ok=%d)\n", ok);
  return ok;
}

bool calibSaveRotationToNvs() {
  if (!g_hasRotation) {
    Serial.println("[Calib] Cannot save Rotation: data invalid.");
    return false;
  }
  if (!prefs.begin(NVS_NAMESPACE, false)) return false; // read-write

  size_t writtenR = prefs.putBytes(NVS_KEY_ROT, ROTATION_MATRIX, sizeof(float)*9);
  
  prefs.end();

  bool ok = (writtenR == sizeof(float)*9);
  Serial.printf("[Calib] Saved Rotation Matrix to NVS (ok=%d)\n", ok);
  return ok;
}

// ============================================================================
// API GETTERS / SETTERS
// ============================================================================

void calibInit() {
  // Reset state for a clean slate (does not wipe NVS)
  g_state = CALIB_IDLE;
  g_hasGravity = false;
  g_hasForward = false;
  g_hasRotation = false;
  g_scaleFactor = 0.966f; // Default safety
}

CalibState calibGetState() { return g_state; }
bool calibHasGravity() { return g_hasGravity; }
bool calibHasRotation() { return g_hasRotation; }
float calibGetScaleFactor() { return g_scaleFactor; }

uint32_t calibGetRemainingMs() {
  if (g_state != CALIB_GRAVITY_SAMPLING &&
      g_state != CALIB_FORWARD_SAMPLING) {
    return 0;
  }
  uint32_t now = millis();
  uint32_t elapsed = now - g_sampleStartMs;
  if (elapsed >= g_sampleDurationMs) return 0;
  return g_sampleDurationMs - elapsed;
}

bool calibGetGravity(float out[3]) {
  if (!g_hasGravity) return false;
  vecCopy(g_gravityRaw, out);
  return true;
}

bool calibGetRotationMatrix(float out[3][3]) {
  if (!g_hasRotation) return false;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      out[i][j] = ROTATION_MATRIX[i][j];
  return true;
}

void calibAbort() {
  g_state = CALIB_IDLE;
  vecClear(g_gravitySum);
  vecClear(g_forwardSum);
  g_gravityCount = 0;
  g_forwardCount = 0;
  g_hasForward = false;
  Serial.println("[Calib] Aborted.");
}

// ============================================================================
// STEP 1: STATIONARY CALIBRATION
// ============================================================================

void calibStartGravity() {
  calibAbort(); // reset all accumulators

  g_state = CALIB_GRAVITY_SAMPLING;
  g_sampleStartMs = millis();
  g_sampleDurationMs = CALIB_STATIONARY_DURATION_MS;
  vecClear(g_gravitySum);
  g_gravityCount = 0;

  Serial.println("[Calib] Gravity calibration started (keep car still).");
}

// ============================================================================
// STEP 2: FORWARD CALIBRATION
// ============================================================================

void calibStartForward() {
  // We require a valid gravity vector (from Step 1 or NVS)
  if (!g_hasGravity) {
    Serial.println("[Calib] Cannot start forward calib: gravity not calibrated/loaded.");
    return;
  }

  // Reset forward accumulators
  g_state = CALIB_FORWARD_SAMPLING;
  g_sampleStartMs = millis();
  g_sampleDurationMs = CALIB_FORWARD_DURATION_MS;
  vecClear(g_forwardSum);
  g_forwardCount = 0;
  g_hasForward = false;

  Serial.println("[Calib] Forward calibration started.");
  Serial.println("[Calib] RULE: Accelerate FIRST from a stop.");
  Serial.println("[Calib] Braking data will be automatically rectified.");
}

// ============================================================================
// STREAMING UPDATE (Main Logic)
// ============================================================================

void calibUpdate(const float accel[3]) {
  uint32_t now = millis();

  // --------------------------------------------------------------------------
  // LOGIC FOR STATIONARY PHASE
  // Gravity calibration sampling frequency: 2936 / 10 seconds = 293.6 Hz
  // --------------------------------------------------------------------------
  if (g_state == CALIB_GRAVITY_SAMPLING) {
    // Accumulate raw gravity over the window
    vecAdd(accel, g_gravitySum);
    g_gravityCount++;

    if (now - g_sampleStartMs >= g_sampleDurationMs) {
      // End of sampling window
      if (g_gravityCount < CALIB_MIN_STATIONARY_SAMPLES) {
        Serial.println("[Calib] Gravity calibration FAILED: not enough samples.");
        g_state = CALIB_ERROR;
        return;
      }

      // 1. Calculate Mean gravity vector (down direction)
      g_gravityRaw[0] = g_gravitySum[0] / g_gravityCount;
      g_gravityRaw[1] = g_gravitySum[1] / g_gravityCount;
      g_gravityRaw[2] = g_gravitySum[2] / g_gravityCount;

      // 2. Calculate Scale Factor (Magnitude of gravity)
      // If sensor is perfect, this is 1.0 (or 9.8, or 16384 depending on sensor unit).
      // We save this to normalize future readings.
      g_scaleFactor = vecNorm(g_gravityRaw);

      Serial.printf("[Calib] Gravity DONE. Samples=%d, mean=[%.4f, %.4f, %.4f], Scale=%.4f\n",
                    g_gravityCount, 
                    g_gravityRaw[0], g_gravityRaw[1], g_gravityRaw[2], 
                    g_scaleFactor);

      // 3. Validate Scale Factor
      if (g_scaleFactor > 0.1f && g_scaleFactor < 20000.0f) {
        g_hasGravity = true;
        g_state = CALIB_IDLE;  // Success
      } else {
        Serial.println("[Calib] Gravity vector invalid (mag near 0?), calibration failed.");
        g_scaleFactor = 1.0f; // Revert to safe default
        g_hasGravity = false;
        g_state = CALIB_ERROR;
      }
    }
    return;
  }

  // --------------------------------------------------------------------------
  // LOGIC FOR FORWARD PHASE (Rectified)
  // --------------------------------------------------------------------------
  if (g_state == CALIB_FORWARD_SAMPLING) {
    // 1. Need normalized "down" direction from gravity
    float down[3];
    vecCopy(g_gravityRaw, down);
    if (!vecNormalize(down)) {
      g_state = CALIB_ERROR;
      return;
    }

    // 2. Isolate Linear Acceleration
    // Project acceleration onto gravity, then subtract it.
    // a_lin = a - (a·down)*down
    float proj = vecDot(accel, down);
    float grav_comp[3];
    vecScale(down, proj, grav_comp);

    float a_lin[3];
    vecSub(accel, grav_comp, a_lin);

    // 3. Check Magnitude Threshold
    // We normalize the threshold check using the known scale factor
    // so it works regardless of sensor units.
    float lin_norm = vecNorm(a_lin);
    float lin_g = lin_norm / g_scaleFactor;

    // Only accumulate significant movement (ignore engine idle vibration)
    if (lin_g > CALIB_FORWARD_MIN_G) {
      
      // --- RECTIFICATION LOGIC (ACCEL FIRST) ---
      
      if (g_forwardCount == 0) {
        // FIRST SAMPLE:
        // We define this direction as "Backward Force" (Acceleration).
        // The vector 'a_lin' is added directly to the sum.
        vecAdd(a_lin, g_forwardSum);
        g_forwardCount++;
      } 
      else {
        // SUBSEQUENT SAMPLES:
        // We check if this new sample points roughly in the same direction
        // as our accumulated average, or opposite.
        float current_sum_copy[3] = {g_forwardSum[0], g_forwardSum[1], g_forwardSum[2]};
        
        float alignment = vecDot(a_lin, current_sum_copy);
        
        if (alignment >= 0) {
             // Points same way (Acceleration). Add directly.
             vecAdd(a_lin, g_forwardSum);
        } else {
             // Points opposite way (Braking).
             // Invert it so it points "Backward", then add.
             // This means Braking data REINFORCES the calibration line.
             float inverted[3];
             vecScale(a_lin, -1.0f, inverted);
             vecAdd(inverted, g_forwardSum);
        }
        g_forwardCount++;
      }
    }

    // Check for Timeout
    if (now - g_sampleStartMs >= g_sampleDurationMs) {
      if (g_forwardCount < CALIB_MIN_FORWARD_SAMPLES) {
        Serial.println("[Calib] Forward calibration FAILED: not enough accel samples.");
        g_state = CALIB_ERROR;
        return;
      }

      // Calculate the average of the rectified sum
      float avg[3] = {
        g_forwardSum[0] / g_forwardCount,
        g_forwardSum[1] / g_forwardCount,
        g_forwardSum[2] / g_forwardCount
      };

      Serial.printf("[Calib] Forward calibration DONE. Samples=%d, avg_lin=[%.4f, %.4f, %.4f]\n",
                    g_forwardCount, avg[0], avg[1], avg[2]);

      vecCopy(avg, g_forwardSum);  // Reuse g_forwardSum to store the final vector
      Serial.printf("[Calib] Final Forward Vector (g_forwardSum): [%.4f, %.4f, %.4f]\n", 
                    g_forwardSum[0], g_forwardSum[1], g_forwardSum[2]);

      g_hasForward = true;
      g_state = CALIB_READY_TO_COMPUTE;
    }
    return;
  }

  // Other states: nothing to do
}

// ============================================================================
// COMPUTE ROTATION MATRIX
// ============================================================================

bool calibComputeRotation() {
  if (!g_hasGravity || !g_hasForward) {
    Serial.println("[Calib] Cannot compute rotation: missing gravity or forward.");
    return false;
  }

  // ---- STEP 1: Gravity unit vector (down) ----
  float gravity_unit[3];
  vecCopy(g_gravityRaw, gravity_unit);
  if (!vecNormalize(gravity_unit)) {
    Serial.println("[Calib] Gravity normalization failed.");
    return false;
  }

  // ---- STEP 2: Forward raw from forwardSum (linear accel mean) ----
  float forward_raw[3];
  vecCopy(g_forwardSum, forward_raw);

  // Remove component parallel to gravity: horizontal forward
  // (Should be mostly removed already, but mathematically precise here)
  float parallel[3];
  float proj = vecDot(forward_raw, gravity_unit);
  vecScale(gravity_unit, proj, parallel);

  float forward_horizontal[3];
  vecSub(forward_raw, parallel, forward_horizontal);

  if (!vecNormalize(forward_horizontal)) {
    Serial.println("[Calib] Forward horizontal normalization failed.");
    return false;
  }

  // ---- STEP 3: Left direction = down × forward_horizontal ----
  float left_unit[3];
  vecCross(gravity_unit, forward_horizontal, left_unit);
  if (!vecNormalize(left_unit)) {
    Serial.println("[Calib] Left vector normalization failed.");
    return false;
  }

  // ---- STEP 4: Build rotation matrix ----
  //
  // We want the INERTIAL frame (where dots move opposite to acceleration).
  //
  // Physics: When you accelerate Forward, the Force vector points BACKWARD.
  // So 'forward_horizontal' actually points BACKWARD relative to the car.
  //
  // To get Car-Forward in the Inertial Display Frame:
  // We want Negative Z (Forward Accel) to map to Negative Display X (Dot Down).
  //
  // Row 0 (Display X) = forward_horizontal ( negative acceleration pushes backward toward the rear of the car)
  // Row 1 (Display Y) = left_unit
  // Row 2 (Display Z) = gravity_unit
  //
  ROTATION_MATRIX[0][0] = forward_horizontal[0];
  ROTATION_MATRIX[0][1] = forward_horizontal[1];
  ROTATION_MATRIX[0][2] = forward_horizontal[2];

  ROTATION_MATRIX[1][0] = left_unit[0];
  ROTATION_MATRIX[1][1] = left_unit[1];
  ROTATION_MATRIX[1][2] = left_unit[2];

  ROTATION_MATRIX[2][0] =  gravity_unit[0];
  ROTATION_MATRIX[2][1] =  gravity_unit[1];
  ROTATION_MATRIX[2][2] =  gravity_unit[2];

  // Simple orthogonality sanity checks (optional debug)
  float dot_xy = vecDot(ROTATION_MATRIX[0], ROTATION_MATRIX[1]);
  float dot_xz = vecDot(ROTATION_MATRIX[0], ROTATION_MATRIX[2]);
  float dot_yz = vecDot(ROTATION_MATRIX[1], ROTATION_MATRIX[2]);

  Serial.println("[Calib] Rotation matrix computed:");
  Serial.printf("  Row0 (X): [%.6f, %.6f, %.6f]\n",
                ROTATION_MATRIX[0][0], ROTATION_MATRIX[0][1], ROTATION_MATRIX[0][2]);
  Serial.printf("  Row1 (Y): [%.6f, %.6f, %.6f]\n",
                ROTATION_MATRIX[1][0], ROTATION_MATRIX[1][1], ROTATION_MATRIX[1][2]);
  Serial.printf("  Row2 (Z): [%.6f, %.6f, %.6f]\n",
                ROTATION_MATRIX[2][0], ROTATION_MATRIX[2][1], ROTATION_MATRIX[2][2]);

  Serial.printf("  Orthogonality check: xy=%.6f xz=%.6f yz=%.6f\n",
                dot_xy, dot_xz, dot_yz);

  g_hasRotation = true;
  g_state = CALIB_DONE;

  return true;
}