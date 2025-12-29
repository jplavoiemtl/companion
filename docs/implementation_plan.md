# Return to Previous Screen (No Gen-Code Mod)

This plan achieves the same goal—returning to the previous screen after image viewing—but respects the constraint of NOT modifying SquareLine Studio generated files (`ui.h`, `ui.c`, `ui_Screen2.c`).

## User Review Required
> [!NOTE]
> This plan reverts previous changes to generated files and moves logic to `companion.ino` and `image_fetcher.cpp`.

## Proposed Changes

### 1. Revert Generated Files
-   **[MODIFY] [ui.h](file:///e:/DataJPL/arduino/arduino_maker/companion/ui.h)**: Remove `ui_previous_screen` declaration.
-   **[MODIFY] [ui.c](file:///e:/DataJPL/arduino/arduino_maker/companion/ui.c)**: Remove `ui_previous_screen` definition/initialization.
-   **[MODIFY] [ui_Screen2.c](file:///e:/DataJPL/arduino/arduino_maker/companion/ui_Screen2.c)**: Revert `ui_event_Button2` to original state.

### 2. State Management & Logic
#### [MODIFY] [companion.ino](file:///e:/DataJPL/arduino/arduino_maker/companion/companion.ino)
-   **Define Global**: `lv_obj_t* ui_previous_screen = NULL;`
-   **New Handler**: Create `custom_buttonBack_event_handler(lv_event_t * e)`.
    -   Checks `ui_previous_screen`.
    -   Calls `lv_disp_load_scr(ui_previous_screen)` if set, else `_ui_screen_change(...)` or `lv_disp_load_scr(ui_Screen1)`.
-   **Setup Hook**: In `setup()`, after `initUIHandlers()` (or inside it):
    -   Remove the default handler: `lv_obj_remove_event_cb(ui_Button2, ui_event_Button2);`
    -   Add custom handler: `lv_obj_add_event_cb(ui_Button2, custom_buttonBack_event_handler, LV_EVENT_CLICKED, NULL);`

#### [MODIFY] [src/image/image_fetcher.cpp](file:///e:/DataJPL/arduino/arduino_maker/companion/src/image/image_fetcher.cpp)
-   **Extern Declaration**: Add `extern lv_obj_t* ui_previous_screen;`.
-   **Logic**: Keep the logic that sets `ui_previous_screen` in `prepareForRequest` and uses it in timeouts. (This part was valid, just needs the variable to be available).

## Verification Plan
1.  **Revert check**: Ensure `ui_Screen2.c` is clean.
2.  **Functional check**: Same as before (Manual Back, Timeout, G-Meter return).
