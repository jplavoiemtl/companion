// imu_calibration.cpp
#include "calibration.h"
#include <Preferences.h>
#include <math.h>

// =========================
// NVS NAMESPACE / KEYS
// =========================

static Preferences prefs;
static const char* NVS_NAMESPACE = "imuCal";
static const char* NVS_KEY_GRAV  = "grav";   // 3 floats (gravity_raw)
static const char* NVS_KEY_SCALE = "scale";  // 1 float (scale factor)
static const char* NVS_KEY_ROT   = "rot";    // 9 floats (rotation matrix)

// =========================
// PUBLIC GLOBAL MATRIX
// =========================
//
// Initialize with your existing "good" matrix as a default.
float ROTATION_MATRIX[3][3] = {
  {-0.037424f, -0.135703f,  0.990042f},  // display_x (vertical)
  {-0.998829f, -0.025327f, -0.041228f},  // display_y (horizontal)
  { 0.030670f, -0.990426f, -0.134596f}   // up (reference)
};

// This scales your specific sensor so 1G = 1.0
// It was determined in step 1 of the Python gravity calibration script.
// Magnitude: 0.966 G at rest instead of 1.0 G
/*
=== STEP 1: Gravity Vector (Stationary) ===
Raw gravity vector: [ 0.02962264 -0.95660377 -0.13      ]
X: 0.030 (lateral)
Y: -0.957 (vertical/down)
Z: -0.130 (forward/back)
Magnitude: 0.966
Normalized (down direction): [ 0.03066999 -0.99042576 -0.13459632]
*/

// =========================
// INTERNAL STATE
// =========================

static CalibState g_state = CALIB_IDLE;

// Gravity accumulation (stationary)
static float g_gravitySum[3] = {0, 0, 0};
static uint32_t g_gravityCount = 0;
static float g_gravityRaw[3] = {0, 0, 0};  // mean gravity vector (down)
static bool g_hasGravity = false;

// Scale Factor (Magnitude of Gravity)
static float g_scaleFactor = 1.0f; // Default to 1.0 if not calibrated

// Forward accumulation (driving)
static float g_forwardSum[3] = {0, 0, 0}; // Stores the Accumulated vector (rectified)
static uint32_t g_forwardCount = 0;
static bool g_hasForward = false;

// Sampling timing
static uint32_t g_sampleStartMs = 0;
static uint32_t g_sampleDurationMs = 0;

// Flags for successful rotation matrix computation
static bool g_hasRotation = false;

// =========================
// CONSTANTS (durations etc.)
// =========================

// You can tweak these durations in the future if needed.
static const uint32_t CALIB_STATIONARY_DURATION_MS = 10000; // 10s parked is usually enough
static const uint32_t CALIB_FORWARD_DURATION_MS    = 30000; // 30s driving

// Minimum number of samples to accept calibration
static const uint32_t CALIB_MIN_STATIONARY_SAMPLES = 200;
static const uint32_t CALIB_MIN_FORWARD_SAMPLES    = 200;

// Threshold for accepting forward linear accel sample (in g)
// This helps filter out engine vibration when idling before the car moves.
static const float CALIB_FORWARD_MIN_G = 0.05f;

// =========================
// SMALL VECTOR HELPERS
// =========================

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
  v[0] /= n;
  v[1] /= n;
  v[2] /= n;
  return true;
}

static inline void vecScale(const float v[3], float s, float out[3]) {
  out[0] = v[0] * s;
  out[1] = v[1] * s;
  out[2] = v[2] * s;
}

static inline void vecSub(const float a[3], const float b[3], float out[3]) {
  out[0] = a[0] - b[0];
  out[1] = a[1] - b[1];
  out[2] = a[2] - b[2];
}

static inline void vecCross(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1]*b[2] - a[2]*b[1];
  out[1] = a[2]*b[0] - a[0]*b[2];
  out[2] = a[0]*b[1] - a[1]*b[0];
}

// =========================
// NVS LOAD / SAVE
// =========================

bool calibLoadFromNvs() {
  if (!prefs.begin(NVS_NAMESPACE, true)) { // read-only
    Serial.println("[Calib] NVS begin failed (load).");
    return false;
  }

  size_t lenG = prefs.getBytesLength(NVS_KEY_GRAV);
  size_t lenS = prefs.getBytesLength(NVS_KEY_SCALE);
  size_t lenR = prefs.getBytesLength(NVS_KEY_ROT);

  bool ok = false;

  // Check if all keys exist and have correct size
  if (lenG == sizeof(float)*3 && lenS == sizeof(float) && lenR == sizeof(float)*9) {
    float g_tmp[3];
    float s_tmp;
    float R_tmp[3][3];

    prefs.getBytes(NVS_KEY_GRAV, g_tmp, sizeof(g_tmp));
    prefs.getBytes(NVS_KEY_SCALE, &s_tmp, sizeof(s_tmp));
    prefs.getBytes(NVS_KEY_ROT, R_tmp, sizeof(R_tmp));

    // Basic sanity check on gravity magnitude (should be ~1.0 if normalized,
    // but g_gravityRaw is NOT normalized, it's the raw sensor reading).
    // Sanity check the Scale Factor instead.
    if (s_tmp > 0.1f && s_tmp < 20000.0f) { // Range covers floats (1.0) or raw ints (16384)
      
      // Load Gravity
      vecCopy(g_tmp, g_gravityRaw);
      g_hasGravity = true;

      // Load Scale
      g_scaleFactor = s_tmp;

      // Load Matrix
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          ROTATION_MATRIX[i][j] = R_tmp[i][j];
        }
      }
      g_hasRotation = true;
      ok = true;

      Serial.printf("[Calib] Loaded from NVS. Scale=%.4f\n", g_scaleFactor);
    } else {
      Serial.println("[Calib] NVS scale factor invalid, ignoring.");
    }
  } else {
    Serial.println("[Calib] NVS keys missing or wrong size, no calibration loaded.");
  }

  prefs.end();
  return ok;
}

bool calibSaveToNvs() {
  // We need Gravity and Rotation to save. Scale is implied if Gravity is valid.
  if (!g_hasGravity || !g_hasRotation) {
    Serial.println("[Calib] Cannot save: gravity or rotation not valid.");
    return false;
  }

  if (!prefs.begin(NVS_NAMESPACE, false)) { // read-write
    Serial.println("[Calib] NVS begin failed (save).");
    return false;
  }

  size_t writtenG = prefs.putBytes(NVS_KEY_GRAV, g_gravityRaw, sizeof(g_gravityRaw));
  size_t writtenS = prefs.putBytes(NVS_KEY_SCALE, &g_scaleFactor, sizeof(g_scaleFactor));
  size_t writtenR = prefs.putBytes(NVS_KEY_ROT, ROTATION_MATRIX, sizeof(float)*9);

  prefs.end();

  bool ok = (writtenG == sizeof(g_gravityRaw) && 
             writtenS == sizeof(g_scaleFactor) && 
             writtenR == sizeof(float)*9);
             
  Serial.printf("[Calib] Saved calib to NVS (ok=%d). Scale=%.4f\n", ok ? 1 : 0, g_scaleFactor);
  return ok;
}

// =========================
// API IMPLEMENTATION
// =========================

void calibInit() {
  g_state = CALIB_IDLE;
  g_hasGravity = false;
  g_hasForward = false;
  g_hasRotation = false;
  g_scaleFactor = 0.966f; // car module default
}

CalibState calibGetState() {
  return g_state;
}

bool calibHasGravity() {
  return g_hasGravity;
}

bool calibHasRotation() {
  return g_hasRotation;
}

float calibGetScaleFactor() {
  return g_scaleFactor;
}

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

// -------------------------
// Stationary (gravity) step
// -------------------------

void calibStartGravity() {
  calibAbort(); // reset all accumulators

  g_state = CALIB_GRAVITY_SAMPLING;
  g_sampleStartMs = millis();
  g_sampleDurationMs = CALIB_STATIONARY_DURATION_MS;
  vecClear(g_gravitySum);
  g_gravityCount = 0;

  Serial.println("[Calib] Gravity calibration started (keep car still).");
}

// -------------------------
// Forward (driving) step
// -------------------------

void calibStartForward() {
  // We can start forward calib as long as we have a valid gravity vector
  // (either from NVS or from a just-finished gravity calibration).
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
  Serial.println("[Calib] RULE: Accelerate FIRST. Braking data will be auto-rectified.");
}

// -------------------------
// Streaming update
// -------------------------

void calibUpdate(const float accel[3]) {
  uint32_t now = millis();

  // ============================================================
  // STATE: GRAVITY SAMPLING (Stationary)
  // ============================================================
  if (g_state == CALIB_GRAVITY_SAMPLING) {
    // Accumulate raw gravity over the window
    vecAdd(accel, g_gravitySum);
    g_gravityCount++;

    if (now - g_sampleStartMs >= g_sampleDurationMs) {
      if (g_gravityCount < CALIB_MIN_STATIONARY_SAMPLES) {
        Serial.println("[Calib] Gravity calibration FAILED: not enough samples.");
        g_state = CALIB_ERROR;
        return;
      }

      // 1. Calculate Mean gravity vector
      g_gravityRaw[0] = g_gravitySum[0] / g_gravityCount;
      g_gravityRaw[1] = g_gravitySum[1] / g_gravityCount;
      g_gravityRaw[2] = g_gravitySum[2] / g_gravityCount;

      // 2. Calculate Scale Factor (Magnitude of that vector)
      g_scaleFactor = vecNorm(g_gravityRaw);

      Serial.printf("[Calib] Gravity DONE. mean=[%.4f, %.4f, %.4f], Magnitude/Scale=%.4f\n",
                    g_gravityRaw[0], g_gravityRaw[1], g_gravityRaw[2], g_scaleFactor);

      // Sanity check
      if (g_scaleFactor > 0.1f && g_scaleFactor < 20000.0f) {
          g_hasGravity = true;
          g_state = CALIB_IDLE;  // ready for forward step
      } else {
          Serial.println("[Calib] Gravity vector invalid (mag near 0?), calibration failed.");
          g_state = CALIB_ERROR;
          g_scaleFactor = 1.0f; // revert to safe default
          g_hasGravity = false;
      }
    }
    return;
  }

  // ============================================================
  // STATE: FORWARD SAMPLING (Driving)
  // ============================================================
  if (g_state == CALIB_FORWARD_SAMPLING) {
    // Need normalized "down" direction from gravity
    float down[3];
    vecCopy(g_gravityRaw, down);
    if (!vecNormalize(down)) {
      Serial.println("[Calib] Forward calib error: invalid gravity vector.");
      g_state = CALIB_ERROR;
      return;
    }

    // 1. Remove gravity component: a_lin = a - (a·down)*down
    // Note: We use 'accel' (raw) here. 'down' is normalized.
    // a_lin will be in raw units, which is fine for determining direction.
    float proj = vecDot(accel, down);
    float grav_comp[3];
    vecScale(down, proj, grav_comp);

    float a_lin[3];
    vecSub(accel, grav_comp, a_lin);

    float lin_norm = vecNorm(a_lin);

    // 2. Threshold Filter (ignore idle vibrations)
    // We compare raw magnitude. If scale is ~1.0, CALIB_FORWARD_MIN_G works as Gs.
    // If scale is ~16384, we need to adjust threshold? 
    // Better approach: Normalize threshold check using the known scale factor.
    float lin_g = lin_norm / g_scaleFactor;

    if (lin_g > CALIB_FORWARD_MIN_G) {
      
      // 3. Rectification Logic (The "Accel First" Rule)
      
      if (g_forwardCount == 0) {
        // FIRST SAMPLE: This defines "Backward" force (Acceleration).
        // We assume the user followed instructions and accelerated first.
        vecAdd(a_lin, g_forwardSum);
        g_forwardCount++;
        // Serial.println("[Calib] First movement detected (Reference Direction).");
      } 
      else {
        // SUBSEQUENT SAMPLES: Check alignment with the running sum.
        // If dot product is negative, the force is opposite (Braking).
        // We invert braking forces so they ADD to the calibration definition
        // rather than cancelling it out.
        
        // Current accumulated average direction
        float current_sum_copy[3] = {g_forwardSum[0], g_forwardSum[1], g_forwardSum[2]};
        
        float alignment = vecDot(a_lin, current_sum_copy);
        
        if (alignment >= 0) {
            // Aligned (Acceleration). Add directly.
            vecAdd(a_lin, g_forwardSum);
        } else {
            // Opposed (Braking). Invert then Add.
            float inverted[3];
            vecScale(a_lin, -1.0f, inverted);
            vecAdd(inverted, g_forwardSum);
            // Serial.print("."); // debug indicator for rectified sample
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

      // Calculate average of the rectified sum
      float avg[3] = {
        g_forwardSum[0] / g_forwardCount,
        g_forwardSum[1] / g_forwardCount,
        g_forwardSum[2] / g_forwardCount
      };

      Serial.printf("[Calib] Forward calibration DONE. Samples=%d, avg_lin=[%.4f, %.4f, %.4f]\n",
                    g_forwardCount, avg[0], avg[1], avg[2]);

      vecCopy(avg, g_forwardSum);  // reuse g_forwardSum as the final "forward_raw" vector
      g_hasForward = true;
      g_state = CALIB_READY_TO_COMPUTE;
    }
    return;
  }

  // Other states: nothing to do
}

// -------------------------
// Compute rotation matrix
// -------------------------

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
  // The vector 'forward_horizontal' we calculated comes from ACCELERATION forces.
  // Physics: When you accelerate Forward, the Force vector points BACKWARD.
  // So 'forward_horizontal' actually points BACKWARD relative to the car.
  //
  // To get Car-Forward in the Inertial Display Frame:
  // We want Negative Z (Forward Accel) to map to Negative Display X (Dot Down).
  //
  // Let's stick to the matrix definition that worked in your Python script:
  // Row 0 (Display X) = -forward_horizontal (Negating the backward force -> Forward)
  // Row 1 (Display Y) = -left_unit
  // Row 2 (Display Z) = gravity_unit
  
  ROTATION_MATRIX[0][0] = -forward_horizontal[0];
  ROTATION_MATRIX[0][1] = -forward_horizontal[1];
  ROTATION_MATRIX[0][2] = -forward_horizontal[2];

  ROTATION_MATRIX[1][0] = -left_unit[0];
  ROTATION_MATRIX[1][1] = -left_unit[1];
  ROTATION_MATRIX[1][2] = -left_unit[2];

  ROTATION_MATRIX[2][0] =  gravity_unit[0];
  ROTATION_MATRIX[2][1] =  gravity_unit[1];
  ROTATION_MATRIX[2][2] =  gravity_unit[2];

  g_hasRotation = true;
  g_state = CALIB_DONE;
  
  Serial.println("[Calib] Rotation matrix computed successfully.");
  return true;
}