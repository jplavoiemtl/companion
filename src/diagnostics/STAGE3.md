# Stage 3 operation context - first build and bench handoff

## September 20 09:34-09:35: Stage 3 full Live passes, boot 89

JP reports normal video/test. LIVE_BEGIN id7, one FIRST_FRAME, two successful
LIVE_CONNECT links to net12/13 (770/681ms), one LIVE_END reason=duration,
failure=none/http200 and observed return2->1. End181 frames/60410ms agrees
with console181/60.4s; independent probe60412ms includes2ms teardown boundary.
Probe-based FPS=2.9961. First1301678us, max gap1007386us, no >2s
frame-gap records or suppression. Totals: HTTP58685215us, TTFB27758391us,
xfer30926824us, decode10928883us, blit14506662us, bytes3292179; agree with
rounded serial averages324/153/171/60/80ms and17.8KB. No per-frame success
records. This is functional/timing consistency evidence, not a same-session
logging-off/on performance comparison against the earlier Stage2 run.

Full Live/TLS sampled largest minimum31732 (6041/77/68 samples), above20480.
Retained boot minimum26612 was already present before case; writer margin3096
stable. Queuehigh7, zero drops/errors/truncation/suppression/slow writes, USB
idle/unpaused/resultok/no link losses. Same boot; no reset/stall reported.
OP_HEALTH max883ms/gaps0, including after Live; long-gap path still unexercised.
HEALTH during initial TLS is explicitly a1127ms-old snapshot of the previous
idle state, not proof Live was inactive. Post-Live inbound advances to14.

Download 503653 bytes, CRC32 0E3A4E1E, SHA256
fb23c69e96bb83fabb987e7feb8c991fc4d2fbd3af7e6a501e6faa7bc867b5d1. Browser CRCOK,5.13s. Prior495206-byte
snapshot exact prefix;153 contiguous boot89 records,13 paired network spans
and7 paired media lifecycles. Sources:
[console](../../docs/bench_data/sd_stage3_2026-09-20_boot89_live/console.txt) and
[current log](../../docs/bench_data/sd_stage3_2026-09-20_boot89_live/89-current.log.txt).

Next single case: controlled MQTT off/on to validate new Stage3 long-gap
attribution. Same build, no rebuild, hotspot stays on, dashboard, web console
DTR=true/RTS=false/test switches off. Status; off once; after first TEST MQTT
attempt END send on promptly. Wait for real broker recovery, remain dashboard
about70s for OP_HEALTH, status/download current/status. No media. Expected
failed test attempt is about5s; commands/UI can wait during existing blocking
call. LOOP_GAP should identify mqtt_connect and its network attempt ID with
measured duration and separate other_ms; no invented SD root cause. Require
recovery, CRCOK, zero drops/errors and memory>=20480. Send console/log and
observations. Stage3 unaccepted; optional tail deferred; Stage4 unstarted.
No source changes/build/flash; commit/push completed evidence per JP request.

Earlier entries below are historical.


## September 20 09:30-09:31: Stage 3 Latest/return passes, boot 89

JP reports the test proceeded fine. Board confirms build=stage3-context,
compiled Sep20 09:23:17. Latest media id6 links to network id11/HTTP200;
IMAGE_END is exactly once, resultok/reasondisplayed, expected=received=34623,
headers757ms, download383ms, decode141ms, total1385ms. Console total1384ms;
one-ms boundary difference is expected. Processed Latest/navigation-back and
observed dashboard/media/dashboard transitions are captured. Motion transitions
and accepted publishes are present. OP_HEALTH max883ms, zero >1s loop gaps
through uptime122014; this case does not exercise long-gap attribution.

Earlier same-boot SD records, before the supplied console, also preserve:
Latest id1/1387ms, history ids2/3/4 at1366/1248/1410ms, each HTTP200 with
matching expected/received bytes (33550/31469/34707 for history). Live id5
has paired connect net10, first-frame1188071us, then reason=screen_left,
failure=none,30 frames/10649ms. http_code=0 at cancellation means the prefetched
response had not supplied its status, not a failed HTTP status. Reuse these
records for history and early Live exit; they are not a full-duration Live
performance run. Earlier G-meter/inclinometer navigation is also recorded.
The button5 route emits both its generated G-meter action and observation-only
navigation record; that alone is not evidence of duplicate navigation.

Latest HTTPS probe largest31732 (75 samples); retained boot minimum26612,
already present at initial status, stays above20480. Internal-free minimum37616,
writer margin3608 ->3096 after retrieval, valid PSRAM placement. Queuehigh7,
zero drops/errors/suppressed/truncated/slow writes; ready/synced, USB ends
idle/unpaused/resultok with no retained link loss. No reset during this capture.
Boot89 itself reports a preceding task_watchdog from88, idle breadcrumbs;
this capture does not identify its cause. Known monitor-close limitation stays
unresolved. Approximate startup wall time is corrected by SNTP +164313ms;
use uptime/sequence for ordering across that correction.

Download 495206 bytes, CRC32 688D501B, SHA256
dd76a8ab84445832a61a3b67bbc5c134a9441499bfd219de81c772b4af627aba.
Browser CRC OK,5.10s. Prior262208-byte snapshot is an exact prefix; boot89
records1-117 contiguous, all11 network spans and six media lifecycles paired;
no format error. Sources: [console](../../docs/bench_data/sd_stage3_2026-09-20_boot89_latest/console.txt)
and [current log](../../docs/bench_data/sd_stage3_2026-09-20_boot89_latest/89-current.log.txt).

Next single case: same flashed Stage3 build, no rebuild. Hotspot/MQTT connected,
web console DTR=true/RTS=false, test switches off. Status; one full Live cycle,
let it return automatically (no early Back, outages or downloads during Live).
Then status, download current, status. Send console/log and visual observations.
Expect LIVE_BEGIN/FIRST_FRAME/END reason=duration, paired connection evidence,
coherent frames/timing, normal video, CRCOK, zero drops/errors and memory>=20480.
Stage3 remains unaccepted; optional tail deferred, Stage4 unstarted. No firmware
changes/build/flash. Commit/push the completed evidence per JP preference.

Earlier handoff entries below are historical.


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
