# Connection status on Screen 1, G-meter and inclinometer

Author: Codex, September 27, 2026. Branch: `codex/connection-status-screens`.
Stable baseline: main `1cc02a6`. JP requested the same connection label on the two
measurement screens, excluding calibration and download, and supplied a SquareLine
export. Implementation ready for Claude code review; no firmware build or flash.

## Change

JP's export creates `ui_labelConnectionStatusGmeter` on Screen3 and
`ui_labelConnectionStatusInclinometer` on InclinometerScreen. Both exactly match the
original label: direct screen child, center alignment, x=8/y=-137, content size,
Montserrat 20. Generated init/destroy functions declare/create/clear the pointers.
All three screens are eagerly initialized by ui_init() and retained for this boot.
The runtime G-meter container is moved to the background, below generated labels.

A single main-task `setConnectionStatusLabels(text, color)` helper reads the three
current pointers and updates each non-null label. All former direct status writes
now use it: Connecting, retry count, No WiFi, No WiFi - Shutdown, MQTT Remote/Local,
WiFi Connected, Offline, Sleeping and Shutdown. Text/color selection, connection
change cache, serial messages and one-per-transition UI_CONNECTION recording retain
their existing behavior. Hidden labels receive changes too, so changing screens does
not need a new connection event. No timers, tasks, networking changes or global overlay.
Calibration and download contain no copy of this label.

## SquareLine export review and asset correction

The export advances generator comments from 1.5.4 to 1.6.2. It also changes the generated
_ui_screen_delete helper to take a destroy callback; no callers exist in the compiled
project source, so the signature change has no existing call-site impact. Other
non-image generated changes are the two labels and generator version comments.
Codex did not hand-edit generated screen/helper files.

The five re-exported images unexpectedly used LVGL 9 `lv_image_dsc_t` descriptors and
RGB565A8 planar data, incompatible with this LVGL 8 firmware. JP explicitly authorized
restoring the previous images because he only added labels. Restored byte-for-byte from
main: ui_img_1435680676.c, ui_img_2121104240.c, ui_img_button_back_png.c,
ui_img_button_latst_png.c, ui_img_button_new_png.c. They carry no diff in this change.
Future SquareLine exports must retain LVGL 8-compatible assets; this restoration does
not correct the exporter configuration itself. No attempt was made to migrate LVGL.

## Validation

All **386 checks in 17 host suites pass**: existing 379 unchanged, plus seven new
connection_status_ui checks. The new suite executes adapted real helper/updater bodies
with LVGL/network mocks: transition fan-out, local/remote ports, unchanged-state cache
and nonduplicated events, startup/power messages, null pointers, exact generated layout,
and LVGL 8 image descriptors. These are host source simulations, not a firmware compile
or proof of physical layout/touch behavior. No existing test assertions were weakened.

Suite counts: connection_status_ui 7; http_lifecycle 43; http_transfer 35; log_time 20;
media_admission 16; mqtt_owner 53; mqtt_recovery 22; mqtt_service 14;
network_diagnostics 16; operation_diagnostics 8; reader_session 28; retrieval_mode 42;
retrieval_ui 15; sd_log_browser 20; touch_contact 19; usb_connection_guard 16;
usb_logger_gate 12.

## Claude review and next gate

Please check that every existing label message/color reaches exactly the three intended
labels, lifecycle assumptions agree with ui_init(), and connection event/cache behavior
is unchanged. Review the supplied SquareLine changes, particularly the helper signature
and restored image compatibility. No additional implementation scope is intended.

After clearance, JP builds/flashes the same amoled-1-8-core-3-3-11 profile. The generated
build/build_amoled-1-8-core-3-3-11/sketch/companion.ino.cpp must be absent before rebuilding
because companion.ino changed. No build/flash is authorized for Codex.

Proposed single bench gate after review: visit the three screens, check identical text,
color and position, then do one ordinary hotspot off/on cycle and check the G-meter and
inclinometer reflect the changing connection state. Confirm calibration/download still
have no added label. No performance campaign is proposed for this small UI change.
Do not start the hardware case until review clears the code.

## Claude code review - September 27, 2026 (83459e2 against 1cc02a6)

**Verdict: cleared for JP's build and the single bench gate.** No blockers. Host checks
re-run: **386 pass in 17 suites**. The only test change is the new
`connection_status_ui` suite. Not compiled, per the workflow.

- **Shared updates.** A search finds no remaining direct write to
  `ui_labelConnectionStatus` outside `setConnectionStatusLabels`. All ten former call
  sites (Connecting, Retry n/m, No WiFi - Shutdown, No WiFi, MQTT Remote/Local, WiFi
  Connected, Offline, Sleeping, Shutdown) keep their exact text and colour. MQTT
  Remote/Local now pass green in the same call instead of a following colour call. The
  `updateConnectionStatusUI` change cache, its early return and the one-per-transition
  UI_CONNECTION record are untouched, so diagnostic output is unchanged.
- **Screen lifetime.** Both new labels are direct screen children created in the
  generated `*_screen_init`, which `ui_init()` runs eagerly, and nothing deletes Screen3
  or InclinometerScreen at runtime. The G-meter's runtime container is created and
  deleted on load and unload, but the label is not inside it. `lv_obj_move_background`
  on that container predates this change (companion.ino:283 at 1cc02a6), so the label
  already draws above the opaque circle. The inclinometer load handler does not clear
  children. Null pointers are skipped.
- **Touch and layout.** The labels match Screen1's properties exactly (x 8, y -137,
  content size, Montserrat 20, centre). They are not clickable, so taps still reach the
  transparent navigation buttons beneath (Screen3 Button1/Button3, inclinometer
  Button4/Button5). No coordinate collides with existing text: braking (-156,-163),
  motion icons (190,-155), pitch/roll widgets at y >= -65.
- **SquareLine export.** Apart from the version comments, the generated diff is the two
  labels and `_ui_screen_delete`'s new callback signature. That function has no callers
  in compiled source; the only matches are stale copies under `build/`. The five root
  images are unchanged from main and use LVGL 8 `lv_img_dsc_t`, which the new host check
  asserts.
- **Side benefit.** "Sleeping..." and "Shutdown..." are now visible when the device
  powers down from the G-meter or inclinometer screen. Previously they only appeared on
  Screen1.

Nonblocking:
- **Unsafe copy source.** The export folder `ui/` still holds the five LVGL 9 images
  (`lv_image_dsc_t`). Copying `ui/` wholesale into the root would break the build. Before
  the next export, correct the SquareLine project's image and LVGL output settings, or
  copy only the screen, helper and header files. The root host check will catch a
  regression.
- **Dot hidden under the label.** On the G-meter, the dot and trail pass beneath the
  label near the top of the circle. This is cosmetic; look at it during the bench gate.
- **Pre-existing unused files.** `ui_Screen4.c` and three images (`camera_button`,
  `150082900`, `956914132`) are tracked and compiled but referenced nowhere. This is
  outside this change; optional cleanup later.
