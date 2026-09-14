// screen_memory.cpp
// Persists the active screen to NVS and restores it on boot

#include "screen_memory.h"
#include <Preferences.h>
#include <Arduino.h>
#include "HWCDC.h"

extern HWCDC USBSerial;

// ============================================================================
// NVS CONFIGURATION
// ============================================================================

static Preferences screenPrefs;
static const char* SCREEN_NVS_NAMESPACE = "screenMem";
static const char* SCREEN_NVS_KEY = "lastScr";

// ============================================================================
// STATE VARIABLES
// ============================================================================

static ScreenMemoryConfig cfg;
static bool g_initialized = false;

// Current saved screen (what's in NVS)
static ScreenId g_savedScreenId = SCREEN_ID_NONE;

// Pending screen to save (after debounce)
static ScreenId g_pendingScreenId = SCREEN_ID_NONE;

// Debounce timing
static uint32_t g_screenEnteredMs = 0;
static bool g_screenSaveScheduled = false;

// Temporary image/video/calibration screens do not select a new boot preference.
static ScreenId g_selectedScreenId = SCREEN_ID_NONE;
static bool g_temporaryScreenActive = false;
static uint32_t g_temporaryEnteredMs = 0;

// Debounce delay: 30 seconds before saving to NVS
static const uint32_t SCREEN_SAVE_DELAY_MS = 30000;

// ============================================================================
// INTERNAL HELPERS
// ============================================================================

/**
 * Map a screen pointer to its ScreenId
 * Returns SCREEN_ID_NONE for temporary screens (Screen2, calibration)
 */
static ScreenId getScreenId(lv_obj_t* screen) {
    if (!g_initialized) return SCREEN_ID_NONE;

    if (screen == *cfg.screen1) {
        return SCREEN_ID_MAIN;
    } else if (screen == *cfg.screen3) {
        return SCREEN_ID_GMETER;
    } else if (screen == *cfg.inclinometerScreen) {
        return SCREEN_ID_INCLINOMETER;
    }
    // Screen2 (image viewer) and calibrationScreen return NONE
    return SCREEN_ID_NONE;
}

/**
 * Load a screen by its ID
 * Uses _ui_screen_change pattern to ensure proper initialization
 */
static void loadScreenById(ScreenId id) {
    if (!g_initialized) return;

    switch (id) {
        case SCREEN_ID_MAIN:
            // Screen1 is already loaded by ui_init(), just ensure it's active
            if (*cfg.screen1 == NULL && cfg.screen1_init) {
                cfg.screen1_init();
            }
            if (*cfg.screen1) {
                lv_disp_load_scr(*cfg.screen1);
            }
            break;

        case SCREEN_ID_GMETER:
            // Initialize Screen3 if needed and load it
            if (*cfg.screen3 == NULL && cfg.screen3_init) {
                cfg.screen3_init();
            }
            if (*cfg.screen3) {
                lv_scr_load_anim(*cfg.screen3, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, false);
            }
            break;

        case SCREEN_ID_INCLINOMETER:
            // Initialize InclinometerScreen if needed and load it
            if (*cfg.inclinometerScreen == NULL && cfg.inclinometer_init) {
                cfg.inclinometer_init();
            }
            if (*cfg.inclinometerScreen) {
                lv_scr_load_anim(*cfg.inclinometerScreen, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, false);
            }
            break;

        default:
            // SCREEN_ID_NONE or invalid - stay on Screen1 (default)
            break;
    }
}

/**
 * Save the screen ID to NVS
 */
static void saveScreenToNvs(ScreenId id) {
    screenPrefs.begin(SCREEN_NVS_NAMESPACE, false); // read-write mode
    screenPrefs.putUChar(SCREEN_NVS_KEY, (uint8_t)id);
    screenPrefs.end();

    g_savedScreenId = id;
    USBSerial.printf("[ScreenMem] Saved screen ID %d to NVS\n", id);
}

/**
 * Load the screen ID from NVS
 * Returns SCREEN_ID_NONE if no valid data found
 */
static ScreenId loadScreenFromNvs() {
    screenPrefs.begin(SCREEN_NVS_NAMESPACE, true); // read-only mode
    uint8_t savedId = screenPrefs.getUChar(SCREEN_NVS_KEY, 0);
    screenPrefs.end();

    // Validate the ID
    if (savedId >= SCREEN_ID_MAIN && savedId <= SCREEN_ID_INCLINOMETER) {
        return (ScreenId)savedId;
    }
    return SCREEN_ID_NONE;
}

// ============================================================================
// PUBLIC API
// ============================================================================

void screenMemoryInit(const ScreenMemoryConfig& config) {
    cfg = config;
    g_initialized = true;

    // Load saved screen from NVS
    ScreenId savedId = loadScreenFromNvs();
    g_savedScreenId = savedId;
    g_selectedScreenId = savedId != SCREEN_ID_NONE ? savedId : SCREEN_ID_MAIN;
    g_pendingScreenId = SCREEN_ID_NONE;
    g_screenSaveScheduled = false;
    g_temporaryScreenActive = false;

    if (savedId != SCREEN_ID_NONE) {
        USBSerial.printf("[ScreenMem] Restoring to saved screen ID: %d\n", savedId);
        loadScreenById(savedId);
    } else {
        USBSerial.println("[ScreenMem] No saved screen, starting on Screen1");
    }
}

void screenMemoryUpdate() {
    if (!g_initialized || !g_screenSaveScheduled || g_temporaryScreenActive) return;
    // Do not commit a preference while another screen is being displayed.
    if (getScreenId(lv_scr_act()) != g_pendingScreenId) return;

    // Check if debounce period has elapsed
    if (millis() - g_screenEnteredMs >= SCREEN_SAVE_DELAY_MS) {
        // Only save if the pending screen differs from saved screen
        if (g_pendingScreenId != g_savedScreenId && g_pendingScreenId != SCREEN_ID_NONE) {
            saveScreenToNvs(g_pendingScreenId);
        }
        g_screenSaveScheduled = false;
    }
}

void screenMemoryOnScreenLoaded(lv_obj_t* screen) {
    if (!g_initialized) return;

    ScreenId newId = getScreenId(screen);

    // Pause an existing selection timer across temporary screens. Repeated
    // image requests and the still-to-Live handover must not restart this pause.
    if (newId == SCREEN_ID_NONE) {
        if (!g_temporaryScreenActive) {
            g_temporaryScreenActive = true;
            g_temporaryEnteredMs = millis();
        }
        return;
    }

    if (g_temporaryScreenActive) {
        g_temporaryScreenActive = false;
        if (newId == g_selectedScreenId) {
            if (g_screenSaveScheduled) {
                g_screenEnteredMs += millis() - g_temporaryEnteredMs;
                USBSerial.printf("[ScreenMem] Returned to screen %d; resuming previous selection timer\n", newId);
            } else {
                USBSerial.printf("[ScreenMem] Returned to screen %d; no preference save needed\n", newId);
            }
            return;
        }
    }

    // Restoring/reloading the selected screen is not a new user selection.
    if (newId == g_selectedScreenId) return;
    g_selectedScreenId = newId;

    // Returning to the already-saved preference cancels any other pending save.
    if (newId == g_savedScreenId) {
        g_pendingScreenId = SCREEN_ID_NONE;
        g_screenSaveScheduled = false;
        USBSerial.printf("[ScreenMem] Screen %d already saved; no NVS write needed\n", newId);
        return;
    }

    g_pendingScreenId = newId;
    g_screenEnteredMs = millis();
    g_screenSaveScheduled = true;

    USBSerial.printf("[ScreenMem] Screen %d selected, save scheduled in %lu seconds\n",
                     newId, SCREEN_SAVE_DELAY_MS / 1000);
}

void screenMemoryEventHandler(lv_event_t* e) {
    if (lv_event_get_code(e) == LV_EVENT_SCREEN_LOADED) {
        lv_obj_t* screen = lv_event_get_target(e);
        screenMemoryOnScreenLoaded(screen);
    }
}
