// screen_memory.h
// Persists the active screen to NVS and restores it on boot
// Does NOT modify SquareLine Studio generated files

#ifndef SCREEN_MEMORY_H
#define SCREEN_MEMORY_H

#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

// Screen IDs for NVS storage
// Using explicit values starting at 1 so 0 = "no saved screen"
typedef enum {
    SCREEN_ID_NONE        = 0,  // No saved screen (default to Screen1)
    SCREEN_ID_MAIN        = 1,  // ui_Screen1 (Dashboard)
    SCREEN_ID_GMETER      = 2,  // ui_Screen3 (G-meter)
    SCREEN_ID_INCLINOMETER = 3  // ui_InclinometerScreen
    // Note: Screen2 (image viewer) and calibrationScreen are NOT included
} ScreenId;

// Configuration struct passed during initialization
typedef struct {
    lv_obj_t** screen1;              // Pointer to ui_Screen1
    lv_obj_t** screen3;              // Pointer to ui_Screen3 (G-meter)
    lv_obj_t** inclinometerScreen;   // Pointer to ui_InclinometerScreen
    void (*screen1_init)(void);      // ui_Screen1_screen_init
    void (*screen3_init)(void);      // ui_Screen3_screen_init
    void (*inclinometer_init)(void); // ui_InclinometerScreen_screen_init
} ScreenMemoryConfig;

/**
 * Initialize screen memory module
 * Must be called after ui_init() and initUIHandlers()
 * Will restore the saved screen if one exists in NVS
 *
 * @param config Configuration with screen pointers and init functions
 */
void screenMemoryInit(const ScreenMemoryConfig& config);

/**
 * Update function - call from loop()
 * Checks if 30-second debounce has elapsed and saves to NVS if needed
 */
void screenMemoryUpdate(void);

/**
 * Notify that a screen has been loaded
 * Call this from screen event handlers on LV_EVENT_SCREEN_LOADED
 *
 * @param screen The screen object that was loaded
 */
void screenMemoryOnScreenLoaded(lv_obj_t* screen);

/**
 * Get the LVGL event handler for screen memory
 * Add this to persistent screens using lv_obj_add_event_cb()
 *
 * Usage in initUIHandlers():
 *   lv_obj_add_event_cb(ui_Screen1, screenMemoryEventHandler, LV_EVENT_SCREEN_LOADED, NULL);
 *   lv_obj_add_event_cb(ui_Screen3, screenMemoryEventHandler, LV_EVENT_SCREEN_LOADED, NULL);
 *   lv_obj_add_event_cb(ui_InclinometerScreen, screenMemoryEventHandler, LV_EVENT_SCREEN_LOADED, NULL);
 */
void screenMemoryEventHandler(lv_event_t* e);

#ifdef __cplusplus
}
#endif

#endif // SCREEN_MEMORY_H
