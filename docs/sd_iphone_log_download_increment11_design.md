# Increment11 - on-device download controls, design for review

September24,2026. JP accepted increment10 after the final normal-use case passed.
Increments1-10 (including6A) are accepted under the September24 reduced bench scope.
This document proposes the next implementation; it is NOT implementation approval.
No firmware edits, builds or flashes accompany this design. Historical plan stays verbatim.

## Intended workflow and proposed entry gesture

On the dashboard, press and hold the connection-status text for about one second to
request download mode. Proposed target: ui_labelConnectionStatus; add clickability and
one LONG_PRESSED callback from custom code after initUIHandlers(), without changing
SquareLine files. Use the existing LVGL long-press threshold unless inspection during
implementation requires a local one-second timer; do not change the global input timing.
The exact threshold is a review item, not a promise of one second with today's default.
No repeat handler, no action on the release click, and no overlap with media buttons.
This gesture is a proposal requiring JP's agreement with the design. It avoids placing
another button over the crowded dashboard. A visible Logs button is an alternative if
JP prefers discoverability over retaining the current layout.

One deliberate hold requests entry directly; no second confirmation is needed. Before
changing the active screen, use the same authoritative admission path as USB. If refused,
keep the current screen and show a short readable reason (USB power required, reconnect
hotspot, logger unavailable, or wait for image/video). Never cancel a pending handover
or media operation to obtain entry. A hold on the dashboard does not stop logging.
When accepted, display a temporary screen, sized using the current LVGL display resolution
(the dashboard uses landscape448x368; do not assume portrait368x448 or rotate the display).

Suggested content, large text and wrapping, no horizontal scrolling:

    Download logs
    Starting... / Ready / Hotspot disconnected / Stopping...
    On your iPhone, open Safari:
    http://172.20.10.2/
    Latest and Live are paused in this mode.
    Closes after 5 minutes without activity.
    [ Stop and return ]

The address is the current station address, never a hardcoded172.20.10.2. Hide it while
starting or disconnected; update it after reconnect. Use the existing trusted-hotspot,
no-token decision; no DNS, QR dependency, AP fallback, upload/delete, or server endpoints
are added. The phone listing remains the place to select files and inspect last result.
USB power means a powered cable/car supply, not a PC or serial connection.

## Single owner and API boundary

Create custom diagnostics_retrieval_ui.h/.cpp under src/diagnostics; generated ui files
are untouched. All LVGL work and mode requests stay on main. Worker, HTTP and writer
never call UI code. Refresh only changed text/state, at most4Hz, from the existing main
background path. UI ticks must not recurse into lv_timer_handler or block on network/SD.

Refactor the existing private enter() into a typed common main-task entry API, e.g.
logRetrievalEnter(Origin::Usb/Panel), returning accepted/refused plus a stable reason.
USB command parsing remains a wrapper and preserves existing serial reply behaviour.
Persist the accepted origin across STARTING so enter result=ok and any startup failure
are recorded with trigger=panel or usb consistently. Do not hardcode trigger=usb for
panel entry. Exit uses existing non-waiting logRetrievalExit("panel") and existing stop
ordering. No UI calls through the text command parser and no dependency on a USB reader.

Expose a small value snapshot: phase OFF/STARTING/ACTIVE/STOPPING, connected state,
last reason, release-stuck flag, and idle age/remaining time if displayed. Snapshot reads
must not reset activity. Keep all decisions in diagnostics_retrieval.cpp; do not clone
entry predicates in UI. A disabled/hidden UI control is never the security/admission guard.
Current connection address is obtained on main only when needed and copied into a bounded
IPv4 URL buffer (32 bytes is sufficient); strings shown to LVGL have valid owned lifetime.

Retain all existing entry checks: OFF, VBUS, logger ready/not closing, STA/interface,
WiFi connected, no reader reservation, no image/live/pending-display/handover.
STARTING/STOPPING continue media exclusion exactly as before. The main-task snapshot
and panel entry API get source-level host tests alongside the existing USB wrapper tests.

## UI allocation, navigation and persistence

Prepare the temporary UI off-screen on first panel request before accepting mode entry;
if construction fails, clean up and remain on the original screen, with no server start.
Check allocations; never show a half-built screen or enter without a usable Stop control.
Re-evaluate authoritative admission after preparation immediately before accepting. On
refusal dispose the unused screen and keep navigation unchanged. On accepted STARTING
load it without animation, remembering the invoking dashboard pointer. No media or
network calls in construction. No images, large canvas, or new task/stack allocation.
Measure UI memory in the first bench case; keep the existing20480 largest-block gate.

Register screenMemoryEventHandler for this screen's SCREEN_LOADED event. It maps to
SCREEN_ID_NONE, pausing pending preference timing. Returning to the existing dashboard
uses its existing registered handler; no new NVS key, saved screen ID or debounce change.
Do not repurpose ui_previous_screen, which belongs to media navigation. The UI must not
become the remembered boot screen. A reboot always starts retrieval OFF.

Stop is enabled in STARTING and ACTIVE. Pressing it calls exit once and changes to
Stopping; further presses cannot start another request. Stay on the screen through
STOPPING, including active-download cleanup. Only observed OFF permits automatic return
to the saved dashboard and cleanup of the custom screen. Perform deletion from a later
UI tick after the screen is no longer active; never free the event target inside its own
callback. Clear all widget pointers. No retained page allocation or per-entry LVGL leak.

Automatic idle, USB-power loss, logger/interface failure and startup rollback use the
same OFF return rule; show the exit/failure reason briefly on the restored dashboard.
Use one bounded transient notice owned by the UI, not one allocation per repeated event.
Do not let a notice intercept unrelated input. Existing diaghttp::notice on lv_layer_top
remains authoritative for stuck teardown and survives screen loads; never cover it with
an opaque top-layer overlay. If release is stuck, show waiting/restart guidance and keep
exclusion; do not force OFF, free shared buffers, auto-reboot or call httpd_stop on main.
Late completion clears the existing warning and allows the normal return/delete path.

Serial-entered modes retain current navigation (no forced new screen). A panel hold while
already ACTIVE may open the controls for the existing mode without re-entering it or
resetting it except for the actual physical touch. USB exit must also close a visible
panel screen through observed OFF. During other non-OFF phases do not create duplicate
screens. Already-visible panel requests are no-ops. UI creation failure while a serial
mode exists leaves that mode unchanged and preserves serial exit. Revalidate current
mode after UI construction in all paths.

Existing read_touch calls logRetrievalTouch for valid touches. Reuse that path; never
refresh idle on rendering, polling, countdown updates, entry-status reads or link recovery.
Hotspot loss keeps ACTIVE and displays reconnect guidance; it does not close the panel.
Other app screen changes must not bypass retrieval exclusion; if an unexpected screen
load occurs, maintain state/Stop accessibility without forcibly fighting power shutdown
screens. Review this interaction with the current shutdown path before implementation.

## Bounded validation and outstanding admission gaps

Host checks: common USB/panel admission including battery-only and pending handover;
origin across async startup; snapshot does not refresh activity; repeated entry/stop;
OFF-only return and deferred deletion; temporary-screen persistence; failure construction
cleanup; USB-enter/panel-open and panel-enter/USB-exit. Keep existing48 console/USB and
relevant retrieval/lifecycle/media checks; do not weaken existing assertions.

First hardware case, after code review and JP build: enter by panel with USB power and
hotspot, read address, save current.log in Safari, Stop, return and prove logging resumes.
Capture UI appearance, transfer/result, memory and writer/HTTP stack. Where feasible use
a power-only supply for the operation, reconnecting USB only afterwards for evidence;
account explicitly for the known serial-monitor transition limitation. No PC-hotspot
requirement. Issue precise steps only when ready, one case at a time.

Retain one short battery-only panel refusal check now that entry no longer needs serial.
Pending-handover timing is still a hardware gap. Recommend retaining host predicate tests
and requesting an explicit waiver of precision-timed hardware reproduction with JP's
implementation approval, rather than adding fault-injection firmware. That waiver is
PROPOSED here, not granted. Do not reopen previously waived increment7 cases.

A focused UI stop/return and persistence check follows only for unproven new UI behaviour;
reuse existing HTTP cancellation/power/idle/network evidence. Keep this to essential new
controls, not the full earlier suite. After panel acceptance, prepare the agreed CAR build,
blank FAT32 card, and parked-car retrieval workflow; JP performs all builds/flashes.

## Review questions / decisions before implementation

1. JP: is holding the dashboard connection-status text acceptable, or prefer a visible
   Logs button? Verify actual hit area and long-press timing in the installed LVGL config.
2. Claude: verify screen lifetime, screen-memory callback and power-screen interaction;
   settle a precise unexpected-screen-load policy rather than introduce navigation loops.
3. Claude: check the minimal typed snapshot/entry refactor preserves serial behaviour and
   async origin; confirm no hidden dependencies on current generated UI callbacks.
4. JP: approve or decline the proposed pending-handover hardware waiver. Battery-only
   refusal stays in the short UI validation. Implementation requires explicit approval
   after review; no code is changed by this document.
