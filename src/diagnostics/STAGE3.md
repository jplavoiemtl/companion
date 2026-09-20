# Stage 3 operation context - first build and bench handoff

## September 20: Stage 2 accepted; Stage 3 implementation prepared

JP explicitly accepted Stage2: "Yes I accept let's proceed". Stage2 evidence
through dff6597 is accepted with existing limitations; Stage3 operation-context
logging is now prepared for JP's first build/bench. Stage4 is unstarted.

Stage3 adds still/Live request IDs, outcome/timeout/cancellation reasons, network
span links, bytes and timings, first frame and bounded >2s frame gaps. It adds
processed UI actions, observed screens/colors, USB/motion transitions, power
entry decisions and >1s main-loop entry gaps with longest measured span and
unattributed remainder. No per-frame success records, raw payloads or secrets.
Optional tail remains deferred until lifecycle instrumentation is bench-tested.
See src/diagnostics/STAGE3.md for semantics, limits and the first case.

Profile remains amoled-1-8-core-3-3-11; enabled1/hooks0/fixture0/PSRAM1,
new tag stage3-context. Selected generated sketch removed before handoff.
JP alone builds/flashes; last measured board remains Stage2 boot87.
Host validation: 16 network +47 USB +8 operation checks pass. These are
source contracts/replays, not firmware compilation or hardware validation.
Network timeouts, retry/media guards, Live pacing, writer core and storage/
USB policy remain. No generated SquareLine files changed. iPhone plan untouched.

Next single case: JP builds/flashes in VS Code; stop and send first compiler
error if any. Web console DTR=true/RTS=false, all test switches off. With
hotspot/MQTT connected, status; Latest once, wait until displayed, navigate
back to dashboard; remain there about70s, status, download current, status.
Send console, log location and visual observations. Expect stage3-context,
paired IMAGE_BEGIN/IMAGE_HTTP/IMAGE_END resultok with matching bytes, processed
UI actions/screens, OP_HEALTH, CRCOK, zero drops/errors, memory>=20480.
Do not add Live, outages, history or calibration to this first case.
Stage3 remains unaccepted. Commit/push this prepared batch per JP preference.

## Evidence semantics

- Media IDs share a main-task counter, separate from Stage2 network IDs.
  IMAGE_HTTP and LIVE_CONNECT link media id to net_id. No new filesystem,
  task, queue or dynamic allocation is introduced by the observer.
- IMAGE_BEGIN precedes UI preparation; total time includes it. End is emitted
  once for success, definitive failure, replacement or screen departure while
  pending. Display expiration after a successful request is UI_RETURN, not a
  second IMAGE_END. Partial receive bytes survive cleanup for failure evidence.
  HTTP headers time includes the blocking GET. Download time includes body phase
  and connection teardown; decode time brackets the decoder. A zero timing
  means that phase was not completed/measured, not a guaranteed instantaneous
  operation. IMAGE_ERROR preserves decoder error codes. Notification refusals
  reuse Stage2's bounded MQTT_IMAGE records; button refusals use IMAGE_REFUSED.
- LIVE_BEGIN can terminate in an early startup error before active=true.
  LIVE_END otherwise accompanies existing teardown. Durations/frame counts allow
  FPS calculation; sum timing fields are totals in microseconds, not averages.
  First frame is recorded once; gaps strictly over2s are limited to one record
  per5s with suppressed counts; pending count is retained in the end summary.
  Error codes/labels are recorded without headers, endpoint URLs or tokens.
- UI_ACTION means an event handler processed a click, not that an unprocessed
  touch during a stall was detected. Generated navigation gets observation-only
  callbacks; generated files and existing action registrations remain untouched.
  UI_SCREEN is main-loop observation (IDs0 other,1 dashboard,2 media,3 G-meter,
  4 inclinometer,5 calibration), not an exact LVGL event timestamp. Short changes
  between observations can be missed. Colors reflect connection-label updates.
  POWER_USB is the existing PMIC snapshot, and MOTION the existing state;
  POWER_DECISION records sleep/shutdown entry without changing its policy.
- LOOP_GAP measures main-loop entry-to-entry, including scheduler time. No boot
  setup gap is synthesized. It links the longest measured network/decoder/blit/
  command/MQTT-loop/calibration/NVS span; nested spans are not added together.
  other_ms is the rest of the interval, not proof of SD, network or any other
  cause. Slow gaps are limited to one record per5s with suppressed counts.
  OP_HEALTH retains boot maximum/count and pending suppression beside HEALTH.
  A fatal stall with no next loop entry has no completed gap record; existing
  breadcrumbs remain the reset evidence. Background sensor/UI time may remain
  unattributed. No raw IMU or individual healthy-loop samples are emitted.

## Validation scope and remaining work

Run tools/tests/operation_diagnostics.test.cjs for actual observer-body replay
and lifecycle/source contracts. Network/USB host suites retain previous coverage.
These do not validate C++ compilation, device stack use, contention or behavior.
JP's first case above validates successful Latest and navigation/health/USB.
Later cases, one at a time, cover history/cancellation/error/timeouts, full and
cancelled Live, controlled outage gap attribution, power/motion and repeated
media/NVS cycles. Repeat paired performance checks after instrumentation changes.
Optional tail is not implemented or accepted by this checkpoint.
