# Increment11 implementation - for Claude code review

September24,2026. Implements approved design revision2 plus origin correction dd8c6cd.
No firmware build, flash or hardware test performed. No hardware case issued.
JP performs builds/flashes only after review. Historical plan and generated ui/ unchanged.

## Change

New diagnostics_retrieval_ui.h/.cpp owns a persistent custom download screen and two
reusable transient notices. It is built once after diagnosticsSetupComplete(), logging
LVGL-pool free/largest/fragmentation before and after to USB and queued SD events.
The Button6 generated callback is replaced during initUIHandlers(); the separate Button6
navigation diagnostic callback is removed. A local1000ms PRESSED/PRESSING timer requests
entry once; RELEASED/PRESS_LOST stop timing but preserve consumption through CLICKED.
Short tap retains the exact generated Screen1 action and emits the navigation record once.
Setup gating leaves pre-setup tap navigation operational; panel calibration sampling or
ready-to-compute refuses with a transient notice, without changing USB admission.

Shared logRetrievalEnter(origin) performs the original pre-entry tick/admission/report
sequence. Refusal uses caller origin without replacing the accepted origin; starting,
worker failure and async success use stored origin. USB message formats/values remain
unchanged for USB callers. The small main-task snapshot is read-only. Non-OFF panel
holds reopen the same screen without calling entry or modifying origin/activity/reason.

The custom screen reserves the top64px for the existing stuck notice. It displays the
live station URL only in connected ACTIVE, status and Stop and return. At most4Hz background
refresh changes label text only when needed. Stop calls panel_stop once, stays through
STOPPING, and only observed OFF returns an active download screen to Screen1. Other
active screens are never forcibly redirected. Both power-down functions first restore
Screen1 only if this screen is active. No buffers/tasks are created by the UI and no
network or storage wait is added. LVGL objects persist for the boot in its existing48KiB
pool; hardware pool usage and appearance remain unmeasured.

screenMemoryEventHandler is attached for SCREEN_LOADED; screen and Stop click callbacks
use activity_event_handler. Existing read_touch remains the sole panel mode-idle refresh.
No new NVS key, screen preference scheme, global long-press change or ui_previous_screen use.

## Host validation

All269 Node host checks pass, without C++ compilation or hardware access:

| Suite | Checks |
|---|---:|
| HTTP lifecycle |43|
| HTTP transfer |35|
| Log time |20|
| Media admission |15|
| Network diagnostics |16|
| Operation diagnostics |8|
| Reader/session |28|
| Retrieval mode |41|
| Retrieval UI |15|
| SD log browser |20|
| USB connection/pacing |16|
| USB logger gate |12|

Existing48 console/USB checks remain unchanged. Existing29 retrieval assertions remain;
only the extraction harness follows the typed entry API with a USB alias for old direct
enter() calls. Twelve added checks cover panel admission, origin refusal/async behaviour,
USB payload text, snapshot immutability and stop interplay. Fifteen new UI checks execute
real adapted C++ bodies with LVGL mocks plus the mode source harness, including persistent
construction, hold consumption, calibration, non-OFF reopening, URL/link states, stop/return,
power hooks and integration registration. Source simulation does not establish compilation,
actual LVGL event delivery, label fit or memory headroom; those remain for review/JP build
and the already planned bounded UI bench. git diff --check passes.

## Review focus

- Local hold consumption through release/click and setup/calibration gating.
- No accepted-origin overwrite on refusal; unchanged USB wrapper sequence/output.
- Persistent object lifetime, temporary screen registration and conditional OFF/power return.
- No idle refresh from rendering and no main-thread network/storage waits.
- LVGL-pool measurement and screen layout below the top notice, without generated edits.

Selected profile remains amoled-1-8-core-3-3-11 (core3.3.11,PSRAM enabled,240MHz).
Diagnostics enabled, writerPSRAM1, test hooks0, USB fixture0. No config changes.
companion.ino changed: its stale generated build/build_amoled-1-8-core-3-3-11/sketch/
companion.ino.cpp was removed after implementation, ready for JP's eventual rebuild. No build or flash is authorized by this
handoff; wait for Claude code-review clearance. No bench instructions issued here.

## September24 - Claude review cleared with include correction applied

JP supplied Claude's review: no logic blockers;269 checks independently passed.
Changed the UI include to sketch-root ui.h and calibration include to calibration.h,
matching existing modules. The ignored ui/ copy is no longer a compilation dependency.
No generated files edited. Cosmetic exit fallback wording and temporary notice overlap
are deferred until actual screen inspection; no behaviour change for either minor note.

Claude verified installed LVGL8.4 press-lock/release ordering, setup/calibration gating,
origin/output preservation, persistent lifetime and conditional OFF/power navigation.
Clear for JP's build after the include fix. Existing generated sketch was removed after
companion.ino edits; check it remains absent before the rebuild. JP builds/flashes only.

First hardware case must start with a stationary1-second hold on calibration Button6.
The current touch path only reports pressed on interrupt-marked reads; stationary-finger
interrupt behaviour is unmeasured. If the hold instead returns home, stop that case and
send console plus the observed behaviour; investigate touch acquisition before changing
hold timing or asking for repetitions. Do not assume host gesture simulations establish
this hardware property. If entry works, continue the same case with address/current-file
retrieval/Stop and logging continuation. Capture LVGL before/after pool records.

## First hardware outcome

JP's stationary hold failed after17cfb41, while short tap worked. See
sd_iphone_log_download_increment11_touch_fix.md for the touch-acquisition correction
awaiting Claude review.281 host checks pass; no new firmware build/flash performed.
The first gate is not passed; previous build clearance does not cover this new fix.
