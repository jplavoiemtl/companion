# Screen Memory Implementation Plan

## Overview

This document outlines the implementation plan for persisting the active screen to NVS (Non-Volatile Storage) so the device restores the last active screen on power-up.

## Important Constraint

**Do NOT modify SquareLine Studio generated files.** All files in the `ui/` directory are auto-generated and would be overwritten when regenerating from SquareLine Studio. This includes:
- `ui.c`, `ui.h`
- `ui_Screen1.c`, `ui_Screen2.c`, `ui_Screen3.c`
- `ui_InclinometerScreen.c`, `ui_calibrationScreen.c`
- `ui_helpers.c`, `ui_helpers.h`
- `ui_events.c`, `ui_events.h`

**Solution**: All customizations are done in `companion.ino` (or new custom modules) by:
1. Adding event callbacks using `lv_obj_add_event_cb()` after screens are initialized
2. Overriding default handlers using `lv_obj_remove_event_cb()` then adding custom ones
3. This pattern is already established in `initUIHandlers()` (see lines 1859-1896 of companion.ino)

## Requirements

1. **Persist active screen to NVS** - Save the current screen when the user navigates to it
2. **Restore on boot** - Load the saved screen after power-up instead of always starting on Screen1
3. **30-second debounce** - Only save to NVS after the user has been on a screen for 30 seconds (reduces flash wear)
4. **Exclude temporary screens**:
   - `ui_Screen2` (Image viewer) - Temporary screen for viewing images
   - `ui_calibrationScreen` - Temporary screen for IMU calibration

## Screens to Track

| Screen Variable | Description | Save to NVS? |
|-----------------|-------------|--------------|
| `ui_Screen1` | Main dashboard | Yes |
| `ui_Screen2` | Image viewer | **No** (temporary) |
| `ui_Screen3` | G-meter display | Yes |
| `ui_InclinometerScreen` | Pitch/Roll display | Yes |
| `ui_calibrationScreen` | Calibration UI | **No** (temporary) |

## Implementation Design

### 1. Screen ID Enumeration

Create a simple enum to identify screens (safer than storing pointers):

```cpp
// In companion.ino or a new header (e.g., screen_memory.h)
typedef enum {
    SCREEN_ID_MAIN      = 1,  // ui_Screen1 (Dashboard)
    SCREEN_ID_GMETER    = 2,  // ui_Screen3 (G-meter)
    SCREEN_ID_INCLINOMETER = 3,  // ui_InclinometerScreen
    // Note: Screen2 (image viewer) and calibrationScreen are NOT included
} ScreenId;
```

**Improvement Note**: Using explicit IDs starting at 1 allows NVS value of 0 to indicate "no saved screen" (default to Screen1).

### 2. NVS Configuration

Follow the existing pattern from `calibration.cpp`:

```cpp
#include <Preferences.h>

static Preferences screenPrefs;
static const char* SCREEN_NVS_NAMESPACE = "screenMem";
static const char* SCREEN_NVS_KEY = "lastScr";
```

### 3. State Variables for Debounce

```cpp
static ScreenId g_currentScreenId = SCREEN_ID_MAIN;
static ScreenId g_pendingScreenId = SCREEN_ID_MAIN;
static uint32_t g_screenEnteredMs = 0;
static bool g_screenSaveScheduled = false;

static const uint32_t SCREEN_SAVE_DELAY_MS = 30000; // 30 seconds
```

### 4. Core Functions

#### Function: `screenMemoryInit()`
- Called in `setup()` after `initUIHandlers()`
- Opens NVS and reads the saved screen ID
- If valid, navigates to that screen instead of remaining on Screen1

```cpp
void screenMemoryInit() {
    screenPrefs.begin(SCREEN_NVS_NAMESPACE, true); // read-only
    uint8_t savedId = screenPrefs.getUChar(SCREEN_NVS_KEY, 0);
    screenPrefs.end();

    if (savedId > 0 && savedId <= SCREEN_ID_INCLINOMETER) {
        loadScreenById((ScreenId)savedId);
        g_currentScreenId = (ScreenId)savedId;
        USBSerial.printf("Restored to screen ID: %d\n", savedId);
    } else {
        USBSerial.println("No saved screen, starting on Screen1");
    }
}
```

#### Function: `loadScreenById(ScreenId id)`
- Maps screen ID to actual LVGL screen object and loads it

```cpp
void loadScreenById(ScreenId id) {
    switch (id) {
        case SCREEN_ID_MAIN:
            lv_disp_load_scr(ui_Screen1);
            break;
        case SCREEN_ID_GMETER:
            _ui_screen_change(&ui_Screen3, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, &ui_Screen3_screen_init);
            break;
        case SCREEN_ID_INCLINOMETER:
            _ui_screen_change(&ui_InclinometerScreen, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, &ui_InclinometerScreen_screen_init);
            break;
    }
}
```

#### Function: `screenMemoryOnScreenLoaded(lv_obj_t* screen)`
- Called from screen event handlers when a screen is loaded
- Determines if this is a "persistent" screen and schedules the NVS save

```cpp
void screenMemoryOnScreenLoaded(lv_obj_t* screen) {
    ScreenId newId = SCREEN_ID_MAIN; // default

    if (screen == ui_Screen1) {
        newId = SCREEN_ID_MAIN;
    } else if (screen == ui_Screen3) {
        newId = SCREEN_ID_GMETER;
    } else if (screen == ui_InclinometerScreen) {
        newId = SCREEN_ID_INCLINOMETER;
    } else {
        // Screen2 or calibrationScreen - ignore, don't save
        return;
    }

    // Schedule save (debounce)
    g_pendingScreenId = newId;
    g_screenEnteredMs = millis();
    g_screenSaveScheduled = true;

    USBSerial.printf("Screen %d entered, save scheduled in 30s\n", newId);
}
```

#### Function: `screenMemoryUpdate()`
- Called in `loop()` to check if 30 seconds have elapsed and save to NVS

```cpp
void screenMemoryUpdate() {
    if (!g_screenSaveScheduled) return;

    if (millis() - g_screenEnteredMs >= SCREEN_SAVE_DELAY_MS) {
        // Only save if still on the same screen
        if (g_pendingScreenId != g_currentScreenId) {
            screenPrefs.begin(SCREEN_NVS_NAMESPACE, false); // read-write
            screenPrefs.putUChar(SCREEN_NVS_KEY, (uint8_t)g_pendingScreenId);
            screenPrefs.end();

            g_currentScreenId = g_pendingScreenId;
            USBSerial.printf("Screen %d saved to NVS\n", g_currentScreenId);
        }
        g_screenSaveScheduled = false;
    }
}
```

### 5. Integration Points

#### A. Modify `setup()` in companion.ino

Add call after UI initialization:

```cpp
void setup() {
    // ... existing initialization ...

    initLVGL();
    initUIHandlers();

    // NEW: Initialize screen memory and restore last screen
    screenMemoryInit();

    // ... rest of setup ...
}
```

#### B. Modify `loop()` in companion.ino

Add the update check:

```cpp
void loop() {
    lv_timer_handler();

    // NEW: Check if we need to save screen to NVS
    screenMemoryUpdate();

    // ... rest of loop ...
}
```

#### C. Hook into Screen Load Events

**Option A: Central Screen Handler** (Recommended)

Create a single handler that listens to all persistent screens:

```cpp
void screen_memory_event_handler(lv_event_t* e) {
    if (lv_event_get_code(e) == LV_EVENT_SCREEN_LOADED) {
        lv_obj_t* screen = lv_event_get_target(e);
        screenMemoryOnScreenLoaded(screen);
    }
}

// In initUIHandlers():
lv_obj_add_event_cb(ui_Screen1, screen_memory_event_handler, LV_EVENT_SCREEN_LOADED, NULL);
lv_obj_add_event_cb(ui_Screen3, screen_memory_event_handler, LV_EVENT_SCREEN_LOADED, NULL);
lv_obj_add_event_cb(ui_InclinometerScreen, screen_memory_event_handler, LV_EVENT_SCREEN_LOADED, NULL);
```

**Option B: Integrate into Existing Handlers**

Add calls to `screenMemoryOnScreenLoaded()` in existing handlers like `screen3_event_handler` and `screenInclinometer_event_handler`.

### 6. Edge Cases and Considerations

1. **First Boot**: If NVS has no saved screen (ID = 0), default to Screen1

2. **Corrupted Data**: Validate the stored ID is within valid range before using

3. **Screen Initialization**: When restoring to G-meter or Inclinometer screen, ensure the screen is properly initialized using `_ui_screen_change()` (not just `lv_disp_load_scr()`)

4. **Debounce Reset on Screen Change**: If user switches screens within 30 seconds, the timer resets for the new screen (only saves after 30s on the final destination)

5. **Power Loss During Debounce**: User won't lose their preference since the previously saved screen is still valid; worst case is one "lost" navigation not being remembered

## File Changes Summary

**Only custom files are modified - NO changes to SquareLine Studio generated files!**

| File | Changes |
|------|---------|
| `companion.ino` | Add screen memory state variables, functions, and integrate into setup/loop |
| `companion.ino` | Modify `initUIHandlers()` to add screen event listeners |

**Alternative (Recommended)**: Create a dedicated module following the project's existing pattern:
- `src/screen_memory/screen_memory.h` - Header with function declarations
- `src/screen_memory/screen_memory.cpp` - Implementation

This keeps `companion.ino` cleaner and follows the same structure as `src/image/image_fetcher.cpp` and `calibration.cpp`.

## Suggested Improvements

### 1. Immediate Save on First Switch (Optional)

Consider saving immediately when switching FROM Screen1 (first navigation after boot), then using the 30-second debounce for subsequent changes. This captures intent faster while still limiting NVS writes.

### 2. Consider Screen1 as "Reset" Screen

You might want a way to "reset" the boot screen back to Screen1. Options:
- Long-press a button to clear NVS and reset to Screen1
- A hidden setting or gesture

### 3. Debug Output Toggle

Add a compile-time flag to enable/disable serial debug output for screen memory operations.

## Testing Checklist

- [ ] Boot with no NVS data - should start on Screen1
- [ ] Navigate to G-meter, wait 30+ seconds, reboot - should restore to G-meter
- [ ] Navigate to Inclinometer, wait 30+ seconds, reboot - should restore to Inclinometer
- [ ] Navigate to Image viewer (Screen2), wait 30+ seconds, reboot - should NOT save Screen2, should restore previous valid screen
- [ ] Navigate to Calibration screen, wait 30+ seconds, reboot - should NOT save calibration, should restore previous valid screen
- [ ] Navigate between screens rapidly (within 30s) - should only save the final destination after settling
- [ ] Power loss during 30s debounce window - should restore to previously saved screen (not the in-progress one)

## Summary

This implementation provides a robust screen persistence mechanism that:
- Follows existing NVS patterns in the codebase (Preferences library)
- Minimizes NVS wear with 30-second debounce
- Properly excludes temporary screens
- Handles edge cases gracefully
- Integrates cleanly with the existing LVGL event system
