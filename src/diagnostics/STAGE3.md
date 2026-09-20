# Stage 3 operation context - first build and bench handoff

## September 20 09:58-09:59: USB-power loss/restoration passes, boot 89

JP reports normal test. POWER_USB present0 at1827688 and present1 at1839147,
11.459s apart, both initial0/sourcepmic_snapshot. Same boot89 before/after,
uptime1821075->1861465; no restart, sleep or shutdown. Browser device-lost/read
stream ended is the expected physical removal; reconnect DTRtrue/RTSfalse
succeeds. No transfer was active during removal. USB transfer-loss count0
therefore does not mean the cable was never disconnected.

Ready/synced, Wi-Fi/MQTT connected afterward, queuehigh8, zero drops/errors/
suppressed/truncated/slow writes. Writer margin3096, retained largest26612 and
internal-free minimum37616 unchanged; final USB idle/unpaused/resultok. No new
LOOP_GAP record. Last OP_HEALTH preceded removal, so post-removal aggregate
health is not claimed from this snapshot. Existing approximately31-minute
same-boot observation remains stable; this is not an exhaustive soak.

Download 557815 bytes, CRC32 71C4ECA2, SHA256
898ea417b325f6f84240f35af93a5f3ba707656addb2a8efb17a3062e43807d4. Browser CRCOK,5.55s. Prior553406-byte
snapshot exact prefix;392 contiguous boot89 records,24 paired network spans,
12 media lifecycles. Sources: [console](../../docs/bench_data/sd_stage3_2026-09-20_boot89_usb_power/console.txt)
and [current log](../../docs/bench_data/sd_stage3_2026-09-20_boot89_usb_power/89-current.log.txt).

Next single case: fresh Stage3 logging-on performance baseline, current build,
no rebuild. Keep hotspot/MQTT connected, dashboard, DTRtrue/RTSfalse/test
switches off. Status, Latest once, return dashboard, one full Live cycle with
no downloads/outages. Then status/download current/status. Send console/log
and visual observations. If unsolicited remote media overlaps, record it for
assessment. Use frames/duration, Latest timings, image size/network metrics,
CRC, zero drops/errors and largest>=20480. This starts the required paired
performance check after Stage3 instrumentation changes. Follow-up logging-off
comparison and restored logging-on small-bundle overlap are separate later
cases, not instructions to rebuild now. Restore normal flags before acceptance.

Reuse passed functional cases and prior unchanged storage/close/NVS gates.
Coverage limits remain explicit: Stage3 POWER_DECISION sleep/shutdown records,
forced response/decode failures, still in-flight cancellation and >2s frame-gap
suppression have not each been induced on hardware. No claim of exhaustive
branch coverage. Optional tail deferred, Stage3 unaccepted, Stage4 unstarted.
No firmware changes/build/flash; completed evidence commit/push per JP request.

Earlier entries below are historical.


## September 20 09:54-09:55: screen-preference save/media case passes

JP reports normal test, same boot89. Console confirms G-meter selected09:54:11.527
and saved09:54:41.532 (30.005s); dashboard selected09:54:49.620, media round
trip pauses the preference timer, return09:54:54.172 resumes it, dashboard
saved09:55:22.887. The screen-memory ID2 means G-meter; diagnostics UI_SCREEN
ID3 means G-meter. These separate ID schemes are not a contradiction.
No reboot persistence test or forced simultaneous NVS/TLS write is claimed.
Both ordinary saves occurred while SD logging remained active; dashboard
preference is restored. Save confirmations come from serial, not new NVS events.

IMAGE_BEGIN id12/net24 -> HTTP200 -> one END ok/displayed, expected=received33843,
headers775/download395/decode140/total1415ms (console1414ms). UI transitions
1->3->1->2->1 and motion changes are coherent. HTTPS largest31732 (78 samples),
retained26612 unchanged, both above20480. Writer margin3096 stable, queuehigh8,
zero drops/errors/suppressed/truncated/slow writes. OP_HEALTH remains5235ms/1gap,
the earlier known MQTT attempt; no new >1s gap around saves/media. Post-case
HEALTH internal_free104576/largest55284, Wi-Fi/MQTT connected. No new reset.
USB ends idle/unpaused/resultok/no losses. All supplied same-boot cases now span
about27 minutes without a new retained memory low or stack-margin decline
since the initial download; this is bounded observation, not a leak-proof soak.

Download 553406 bytes, CRC32 593EB0F1, SHA256
40e52d92d2bd131e51db56046aaa71e7d875ff39592b5f55e1aaa80830873f40. Browser CRCOK,5.58s. Previous543193-byte
snapshot exact prefix;374 contiguous boot89 records,24 paired network spans,
12 media lifecycles. Sources: [console](../../docs/bench_data/sd_stage3_2026-09-20_boot89_screen_save/console.txt)
and [current log](../../docs/bench_data/sd_stage3_2026-09-20_boot89_screen_save/89-current.log.txt).

Next single case: brief USB-power transition, same build/no rebuild. Current
health confirms battery present at4.121V. Keep hotspot on and dashboard active;
normal browser DTR=true/RTS=false/test switches off. Status, no transfer active;
physically unplug USB for10s, then reconnect (within existing30s USB-loss grace).
Board should stay on battery. Reconnect web console if needed with same signals,
status/download current/status. Send console/log and whether screen stayed on
or a reboot occurred. Expect POWER_USB present0 then1, same boot, logger ready,
CRCOK and zero drops/errors. Expected USB transport disconnect itself is not a
logger failure. This does not test shutdown/sleep POWER_DECISION records.
Stage3 remains unaccepted; optional tail deferred; Stage4 unstarted. No source
changes/build/flash; completed evidence commit/push authorized by JP.

Earlier entries below are historical.


## September 20 09:49-09:50: Stage 3 Live network-loss exit passes

JP reports normal test and recovery much faster than90s. The90s instruction
was a maximum wait before reporting failed recovery, not expected latency.
Same boot89, Live id11/net21 connects680ms, first frame1231562us. Following
Wi-Fi driver loss at1265270, LIVE_END occurs1265274: reasonfetch_error,
failureconnection_closed, http_code0 (no response status),16 frames/6618ms.
The4ms is driver-observation-to-terminal-record time, not physical hotspot
switch latency. UI_SCREEN2->1 follows7ms later. No duplicated terminal,
no claimed response timeout, no reset or unexpected stall. Retry defers until
media clears. Fast failed MQTT22 lasts2ms; its zero-sample probe is not a
sampled memory gate. Wi-Fi association1290548 -> GOT_IP1291656 -> MQTT
connected1292428:1.880s association-to-MQTT, with attempt23 taking667ms.
Hotspot toggle time is not recorded, so total hotspot-to-recovery is unknown.
Subscriptions/calibration accepted; UI orange/red/green captured.

Live/TLS sampled largest31732 (661/68 samples), recovery55284 (67 samples),
all above20480. Retained26612 unchanged; writer margin3096 stable. Queuehigh8,
zero drops/errors/suppressed/truncated/slow writes; final USB idle/unpaused,
resultok/no losses. OP_HEALTH before loss retains prior max5235/gaps1; no new
LOOP_GAP record in this snapshot. No new health window after recovery yet.

Download 543193 bytes, CRC32 097A1D6E, SHA256
7629ae1576c16c575a05e8878a13536afaa0e07176bd28eaef50ded72b2f5616. Browser CRCOK,5.55s. Previous529873-byte
snapshot exact prefix;330 contiguous boot89 records,23 paired network spans,
11 media lifecycles. Sources: [console](../../docs/bench_data/sd_stage3_2026-09-20_boot89_live_loss/console.txt)
and [current log](../../docs/bench_data/sd_stage3_2026-09-20_boot89_live_loss/89-current.log.txt).

Next single case: screen-preference save/media interaction, same build/no rebuild.
Hotspot/MQTT connected, web console DTR=true/RTS=false/test switches off.
Status; navigate dashboard->G-meter, remain40s for existing30s preference
save. Return dashboard, run Latest once promptly, then navigate back after
image displayed. Remain dashboard40s to allow its preference save, then
status/download current/status. Capture ScreenMem save lines, console/log and
visual behavior. This exercises ordinary NVS saves around media and SD logging;
it does not claim a deliberately forced simultaneous NVS/TLS write. Dashboard
is restored as preferred screen. Require paired image lifecycle, correct screen
observations, zero drops/errors/resets, CRCOK and memory gate. No calibration
changes, firmware modifications/build/flash. Stage3 unaccepted, optional tail
deferred, Stage4 unstarted; completed evidence committed/pushed per JP preference.

Earlier entries below are historical.


## September 20 09:43-09:45: offline Latest failure/return passes, boot 89

JP reports normal test. IMAGE_BEGIN id10/triggerlatest/wifi0/mqtt0 has exactly
one IMAGE_END failed/wifi_offline in99ms, zero bytes and unmeasured phase times0.
UI_RETURN says request failed to start; no HTTP network span for this request.
The brief loading-screen round trip occurs inside one loop: no UI_SCREEN record
is expected from the documented polling observer. Console confirms load/unload
and return; subsequent HEALTH screen1/idle agrees. No invented HTTP timeout.

Driver loss reason2/auth_expired, MQTT_LOST state-3/TLS48 freshness unknown;
fast real attempt19 fails2ms/state-2. Its1ms probe has zero samples, so is not a
new sampled TLS gate. Offline repeats correctly reject raw RSSI-128 and retain
-46 as last_valid, with suppression. NET_HEALTH reports26 suppressed at964242.
Association/GOT_IP recover; real MQTT20 succeeds657ms/state0/recovery1, all
subscriptions/calibration accepted. UI connection orange/red/green observed.
Recovery probe largest51188/66 samples above20480; retained boot26612 unchanged.
OP_HEALTH stays max5235/gaps1 (the prior planned MQTT gap). No new reset/stall.
Queuehigh8, zero drops/errors/suppressed/truncated/slow writes, writer margin3096
stable; final USB idle/unpaused/resultok/no link loss, Wi-Fi/MQTT connected.

Download 529873 bytes, CRC32 57E80067, SHA256
fc87ba81026f3ae1f3c4be12d12f8c3e999cb2bcc2a9a90bdee40d34d0c7d8f2. Browser CRCOK,5.30s. Previous516494-byte
snapshot exact prefix;272 contiguous boot89 records,20 paired network spans,
10 media lifecycles. Sources: [console](../../docs/bench_data/sd_stage3_2026-09-20_boot89_offline_latest/console.txt)
and [current log](../../docs/bench_data/sd_stage3_2026-09-20_boot89_offline_latest/89-current.log.txt).

Next single case: Live network-loss exit, same build/no rebuild. USB/browser
remain connected, dashboard, DTR=true/RTS=false/test switches off. Status;
start Live with hotspot/MQTT connected. Once video has played about5s, turn
hotspot off. Wait for automatic failure/return; do not press Back. If it has
not returned within30s, send status, restore hotspot and report what happened.
After return, restore hotspot (settings open), allow up to90s for automatic
Wi-Fi/MQTT recovery. Status/download current/status; send console/log and video
behavior. Expected bounded Live error exit with one terminal record, actual
failure reason (do not require a specific timeout vs connection-close label),
recovery, CRCOK, zero drops/errors and memory gate. Stop if recovery fails.
Stage3 remains unaccepted; optional tail deferred; Stage4 unstarted.

Earlier entries below are historical.


## September 20 09:39-09:40: MQTT loop-gap attribution passes despite remote Live

JP reports board/test normal, with a remote-triggered Live overlapping the case.
No repeat needed: the planned blocking attempt occurred after Live ended and
its complete span/gap evidence is present. Extra media is recorded separately.

Earlier MQTT_IMAGE accepted -> IMAGE_BEGIN id8/trigger=mqtt -> HTTP200 with
expected=received33843, total1305ms. After1002ms display, LIVE_BEGIN id9 has
trigger=motion_handover; first1106135us. Serial off occurs while Live active;
MQTT_RETRY_POLICY deferred/media precedes the next Live TLS reconnect.
Navigation Back ends Live at39559ms/114frames, reason=screen_left/failure=none;
probe39561ms, minimum26612 (3956 samples). No MQTT attempt while active.
Release/media_clear at649054, test attempt id17 begins649154. It fails in
5003ms/state-2/TLS-1 generic, freshness unknown. LOOP_GAP at654160:
elapsed_ms5235, observed_span=mqtt_connect, span_id17, span_ms5003,
other_ms232, suppressed0. The remainder is not assigned a fabricated cause.
UI_SCREEN is observed after this blocking call, consistent with its documented
main-observation semantics rather than an exact LVGL event timestamp.

On restores real broker: id18 succeeds537ms/state0/recovery1; subscriptions,
calibration and periodic motion publishes accepted. UI orange then green.
Two OP_HEALTH intervals retain max5235/gaps1/pending_suppressed0; no extra
unexplained >1s gap. Inbound counts advance21->23 after recovery. Suppression
of repeated long gaps is not exercised by this single gap.

MQTT test probe largest63476 (501 samples), recovery53236 (53 samples),
all above20480; retained26612 unchanged. Writer margin3096 stable; queuehigh8
(previous7), zero drops/errors/suppressed/truncated/slow writes. USB ends idle,
unpaused/resultok/no losses. Same boot89, no new reset or unexpected stall.
Download 516494 bytes, CRC32 D4227FF5, SHA256
4665b588645a77d9c9c976950fa4c5897764633aab36efd46c4e2e872406def5.
CRCOK,5.13s; prior503653-byte snapshot exact prefix,215 contiguous boot89
records,18 paired network spans and9 media lifecycles. Sources:
[console](../../docs/bench_data/sd_stage3_2026-09-20_boot89_mqtt_gap/console.txt) and
[current log](../../docs/bench_data/sd_stage3_2026-09-20_boot89_mqtt_gap/89-current.log.txt).

Next single case: offline Latest failure/return, same build (no rebuild).
Keep USB/browser connected DTR=true/RTS=false, test switches off, dashboard.
Status; turn hotspot off, wait about30s and confirm Wi-Fi OFFLINE with status.
Tap Latest once; expect quick return to dashboard without a new image. Do not
retry. Restore hotspot, leave settings open and allow up to90s for automatic
Wi-Fi/real MQTT recovery; if recovery fails stop and send console/status.
Otherwise status, download current, final status; send log/console/observations.
Expect exactly one IMAGE_END failed/wifi_offline, UI_RETURN and no HTTP span
for that offline request, recovery, CRCOK and zero drops/errors. This exercises
failure evidence, not an HTTP timeout. Stage3 unaccepted, optional tail deferred,
Stage4 unstarted. No firmware changes; completed results committed/pushed per JP.

Earlier entries below are historical.


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
