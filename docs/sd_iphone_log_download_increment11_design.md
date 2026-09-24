# Increment11 - on-device download controls, design revision 2

September24,2026. JP accepted increment10 after the final normal-use case passed.
Increments1-10 (including6A) are accepted under the September24 reduced bench scope.
Revision 2 records JP's entry/exit decisions and the resolutions from Claude's review of
0e573dd against current source. JP granted the pending-handover waiver and handed
implementation to Codex (see the end). No firmware edits, builds or flashes accompany
this document.
Historical plan stays verbatim.

## Decisions (JP, September24)

1. **Entry is on the IMU calibration screen, not the dashboard.** Reached from the
   inclinometer by swiping left. A **hold of about one second on the top band** of that
   screen requests download mode. A short tap there still returns to Screen1 exactly as
   today. The dashboard layout and its connection-status label are unchanged.
2. **Exit returns to Screen1.** Stop, idle expiry, USB-power loss and every other path to
   OFF return to the dashboard, not to the calibration screen.
3. **The download screen has its own Stop button**, built in custom code. No SquareLine or
   generated-file work is needed from JP.

## Entry gesture - calibration screen top band

The visible title `ui_calibLabel` ("IMU Calibration", y -154) cannot take the press:
`ui_Button6`, an invisible (bg_opa0) 442x142 button created after it
(ui_calibrationScreen.c:111-120, y -185 to -43), covers the whole top band. Its generated
handler `ui_event_Button6` goes to Screen1 on CLICKED, and LVGL still sends CLICKED on the
release that follows a long press. Adding only a long-press callback would enter the mode
and then immediately navigate to Screen1 on release.

Use the existing Back-button override pattern (companion.ino, `initUIHandlers()`); the
generated files are untouched:

- `lv_obj_remove_event_cb(ui_Button6, ui_event_Button6)` and add one custom handler.
- Local one-second threshold: record the tick on PRESSED, fire once on PRESSING when
  1000 ms have elapsed, reset on RELEASED and PRESS_LOST. The installed config leaves
  `LV_INDEV_DEF_LONG_PRESS_TIME` at LVGL's 400 ms default; global input timing is not changed.
- A fired hold requests entry and marks the press consumed. CLICKED on a consumed press
  does nothing. Any other CLICKED performs exactly the generated action:
  `_ui_screen_change(&ui_Screen1, LV_SCR_LOAD_ANIM_NONE, 0, 0, &ui_Screen1_screen_init)`.
- `diagnosticNavigationEvent` is also registered on `ui_Button6` CLICKED (companion.ino:2038).
  Move that navigation record into the custom handler so it is written only when a
  navigation actually happens, not after a hold.
- The 442x142 target needs no extended click area. No repeat handler.

**Armed only after setup.** Screen memory can restore the inclinometer before `initWiFi()`,
and LVGL runs inside the Wi-Fi and MQTT setup loops (`runBackgroundTick()` at
companion.ino:2193 and :2235), so the calibration screen is reachable before setup ends.
USB entry never was (commands are parsed only in `loop()`). The override must exist from
boot because it owns navigation home, so it carries a flag set after
`diagnosticsSetupComplete()`. Until then a hold does nothing and the tap goes home as today.

**Calibration in progress.** Refuse panel entry while `calibGetState()` is
CALIB_GRAVITY_SAMPLING, CALIB_FORWARD_SAMPLING or CALIB_READY_TO_COMPUTE: leaving the screen
would hide the result `updateCalibration()` reports there. This is a panel-only check, so
USB entry behaviour is unchanged.

One deliberate hold requests entry directly; no second confirmation. Admission uses the
same authoritative path as USB. If refused, stay on the calibration screen and show a short
readable reason (USB power required, reconnect hotspot, logger unavailable, wait for
image/video, calibration running). Never cancel a pending handover or media operation to
obtain entry. A hold does not stop logging.

## Download screen

Suggested content, large text and wrapping, no horizontal scrolling, laid out with the
current LVGL display resolution (landscape 448x368 through `lv_disp_get_hor_res()`; do
not assume portrait 368x448 or rotate the display):

    Download logs
    Starting... / Ready / Hotspot disconnected / Stopping...
    On your iPhone, open Safari:
    http://172.20.10.2/
    Latest and Live are paused in this mode.
    Closes after 5 minutes without activity.
    [ Stop and return ]

- **Stop and return** is a large button at the bottom. Enabled in STARTING and ACTIVE.
  One press calls exit once and shows Stopping...; further presses do nothing. An
  in-progress download is cancelled cleanly by the existing stop ordering.
- Keep the top ~64 px free of anything essential: `diaghttp::notice` places an opaque,
  non-clickable two-line warning at the top of `lv_layer_top` (diagnostics_http.cpp:555-562).
  It passes touches through but covers what is under it.
- The address is the current station address, never a hardcoded 172.20.10.2. Hide it while
  starting or disconnected; update it after reconnect. Build it in a bounded 32-byte buffer
  and set it with `lv_label_set_text` (LVGL copies it), not the `_static` variant.
- Existing trusted-hotspot, no-token decision. No DNS, QR, AP fallback, upload/delete or
  new server endpoints. The phone listing remains the place to select files and inspect
  the last result. USB power means a powered cable/car supply, not a PC connection.
- Register `activity_event_handler` for CLICKED on this screen, as the generated screens
  do, so touches here count for the power inactivity timer (this matters in TEST_POWER
  builds, where that timer ignores VBUS). `logRetrievalTouch()` from `read_touch` remains
  the only mode-idle refresh.

## Screen lifetime and memory

**Build the download screen once, after setup, and keep it for the boot**, like every
SquareLine screen. Reason: `lv_conf.h` sets `LV_USE_ASSERT_MALLOC 1` with
`LV_ASSERT_HANDLER while(1);`, so a failed LVGL allocation halts the main task rather than
returning NULL. A construct-on-demand path with failure cleanup cannot exist in this
configuration. Building once removes, from revision 1: construction-failure handling,
off-screen preparation and re-admission, deferred deletion and the delete-inside-callback
hazard, the per-entry leak check, and remembering the invoking screen.

LVGL objects come from LVGL's fixed 48 KB internal pool (`LV_MEM_CUSTOM 0`), not the
system heap, so the 20480-byte internal largest-block gate does not measure UI cost. Log
`lv_mem_monitor()` (free, largest free block, fragmentation) before and after building
the screen; the first bench case records it. No images, canvas, task or stack allocation.

## Navigation and persistence

- The download screen is loaded only by the user's hold. Nothing loads it automatically.
- A hold in any non-OFF phase (STARTING, ACTIVE, STOPPING), including a serial-entered
  mode, reopens the same screen without re-entering or resetting the mode. Stop therefore
  stays reachable if anything else was loaded meanwhile. Serial entry itself does not
  force the screen.
- **On observed OFF:** if the download screen is active, load `ui_Screen1` without
  animation. If it is not active, do nothing; do not pull the user back from another screen.
- Register `screenMemoryEventHandler` for SCREEN_LOADED once. The screen maps to
  `SCREEN_ID_NONE`, so the preference timer pauses. Do not repurpose `ui_previous_screen`.
  No new NVS key; a reboot always starts retrieval OFF.
- **Accepted side effect of returning to Screen1:** as with today's top-band tap, Screen1
  becomes the saved boot screen if it stays active for 30 s. JP accepted this.
- Stay on the screen through STOPPING, including active-download cleanup. Automatic idle,
  USB-power loss, logger/interface failure and startup rollback use the same OFF rule, and
  show the exit reason briefly on Screen1.
- Transient notices (refusal on the calibration screen, exit reason on Screen1) use one
  custom non-clickable label per screen, created once, hidden after about 3 s. Do not use
  `ui_calibStatusLabel` (calibration code writes it) or `ui_labelConnectionStatus`
  (rewritten every second). Never cover the stuck notice.
- Hotspot loss keeps ACTIVE and shows reconnect guidance; it does not close the screen.
- If release is stuck, keep the screen and exclusion, show waiting/restart guidance, and
  do not force OFF, free shared buffers, auto-reboot or call `httpd_stop` on main.

**Power-down hook.** There is no separate shutdown screen: `goToShutdown()` and
`goToDeepSleep()` write "Shutdown..." / "Sleeping..." into `ui_labelConnectionStatus` on
Screen1 (companion.ino:1770, :1728). Reachable case: USB lost, mode STOPPING with a slow or
stuck release, then 30 s grace plus 60 s without touch. At the top of both functions, if
the download screen is active, load `ui_Screen1` without animation before the message is
written. Delete nothing and do not change the mode; `diagnosticsClose()` still owns
teardown. The resulting SCREEN_LOADED is harmless because `screenMemoryUpdate()` does not
run again after shutdown begins.

## Single owner and API boundary

Create custom `diagnostics_retrieval_ui.h/.cpp` under src/diagnostics; generated ui files
are untouched. All LVGL work and mode requests stay on main. Worker, HTTP and writer never
call UI code. Refresh only changed text, at most 4 Hz, from the existing main background
path. No recursion into `lv_timer_handler`, no blocking on network or SD.

Refactor the private `enter()` into a common main-task entry, e.g.
`logRetrievalEnter(Origin::Usb/Panel)`, returning accepted/refused plus the existing
stable reason codes from `entryRefusal()`:

- Move the pre-entry `logRetrievalTick()` from the `log mode on` wrapper into the common
  entry, so a panel entry also lets STOPPING settle first. USB keeps the identical sequence.
- `trigger=usb` is currently hardcoded in the refused, worker_start-failed and starting
  events in `enter()` and in the `result=ok` event in `logRetrievalTick()`. The refused
  event uses the **requesting caller's** origin. The origin is stored in a static only when
  admission succeeds, and the stored value is used for worker_start-failed, starting and the
  asynchronous `result=ok`. A refused request, including `not_off` during an active session,
  never overwrites the stored origin. USB output is unchanged. (Correction raised by Codex.)
- Panel Stop calls `logRetrievalExit("panel_stop")`, alongside the existing `usb_command`.
  It keeps the existing `diagnosticsUsbCommand("log abort")` for an in-flight USB transfer.
- `not_off` from the entry path means reopen the existing screen.
- The UI maps reason codes to readable text; it does not clone admission predicates. A
  hidden or disabled control is never the admission guard.

Expose a read-only value snapshot: phase OFF/STARTING/ACTIVE/STOPPING, link state, last
reason, release-stuck flag, and idle remaining if displayed. Reads never change `activityAt`.

Retain all existing entry checks: OFF, VBUS, logger ready/not closing, STA/interface,
WiFi connected, no reader reservation, no image/live/pending-display/handover.
STARTING/STOPPING continue media exclusion exactly as before.

## Bounded validation

Host checks, alongside the existing 48 console/USB checks and the relevant
retrieval/lifecycle/media checks, without weakening any assertion:

- common USB/panel admission, including battery-only and pending handover
- refused events use the caller origin and never overwrite an active session origin;
  stored origin preserved across async startup; USB event text unchanged
- snapshot does not refresh activity
- hold armed only after setup; short tap still navigates; consumed hold does not navigate
- panel refusal during calibration sampling
- reopen in every non-OFF phase; OFF-only return to Screen1; no return when not active
- power-down hook loads Screen1 only when the download screen is active
- USB-enter/panel-open and panel-enter/USB-exit
- repeated entry/stop with the persistent screen

Revision 1's construction-failure and deferred-deletion tests are dropped; that code no
longer exists.

**First hardware case**, after code review and JP build: enter by holding the calibration
top band with USB power and hotspot, read the address, save current.log in Safari, Stop,
confirm return to Screen1 and that logging resumes. Capture UI appearance, transfer/result,
`lv_mem_monitor()` before/after screen build, internal memory and writer/HTTP stack. Where
feasible use a power-only supply for the operation, reconnecting USB afterwards for
evidence, accounting for the known serial-monitor transition limitation. No PC-hotspot
requirement. Steps issued one case at a time when ready.

Retain one short battery-only panel refusal check. Reuse existing HTTP cancellation,
power, idle and network evidence; a focused stop/return check covers only the new UI.
After panel acceptance, prepare the agreed CAR build, blank FAT32 card and parked-car
retrieval workflow; JP performs all builds and flashes.

## Pending-handover waiver - GRANTED by JP, September24

JP granted the waiver of precision-timed hardware reproduction of the pending-handover
refusal. Panel and USB share one `entryRefusal()`, so the panel adds no admission logic.
The handover state (`imageDisplayTimeoutActive || motionTriggered`) exists only while a
fetch is running, which `imageFetcherIsBusy()` already refuses, or while Screen2 is
displayed, when the calibration screen cannot be touched. Host predicate tests remain
required. Previously waived increment7 cases stay closed.

## Status and handoff

JP approved the decisions and waiver above and hands implementation to Codex. Codex
reviews this revision against current source first and raises any disagreement before
coding. Implementation stays within this document; JP performs all builds and flashes,
and the first hardware case is issued only after Claude code review.
