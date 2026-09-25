# P001 increment 2 — responsiveness telemetry handoff

September 25, 2026. Author: Codex. Status: ready for Claude review, not build clearance.
JP accepted increment 1 after boot 130's successful worker handshake and authorized
increment 2 with "please proceed". Source base: 36a9819. No build or flash by Codex.

## Scope and implementation

Complete section 9's per-attempt service measurements without changing reconnect,
client, deadline, admission, image, IMU sampling, power or WiFi policies. Existing worker
phase/transport timing stays unchanged. Actual failure/flap validation remains hardware
work after review; host simulations do not replace those three cases.

New `src/diagnostics/diagnostics_mqtt_service.h/.cpp` is a fixed-size, main-task-only
observer. No task, allocation, network call, SD access, wait or per-service record.
`netCheckMqtt` starts a window only after an accepted request, using the owner's request
clock. The common `takeResult` path finishes it after success/failure/cancellation
adoption, including READY acknowledgement and calibration enqueue. It emits exactly one
`MQTT_CONNECT_SERVICE` at that main completion time. Worker END retains its original
completion stamp; SERVICE intentionally has a later stamp and can have a larger window.
A stuck worker's window remains open until late completion; no invented completion record.

Actual calls are instrumented:
- Every authored `lv_timer_handler` in companion.ino, image_fetcher and video_stream has
  a scoped UI observer, preserving the original call and control flow. Generated UI is
  untouched. Nested LVGL calls collapse into the outer service duration.
- `updateImuData` has an RAII observer covering every return, including no-fresh-data
  and mutex-refusal paths. This measures IMU servicing, **not sample rate or freshness**.
- Existing `diagop::Loop` observes entry and exit of the actual Arduino loop, including
  early returns. Background setup ticks are not relabelled as main-loop turns.
- Existing `diagop::block` supplies the longest completed main span overlapping the
  window. Existing LOOP_GAP thresholds, attribution and records remain unchanged.

Fields (milliseconds unless otherwise named):
- `id`, `window_ms`: request to main result adoption.
- For `ui`, `imu`, `loop`: `*_gap_ms` maximum entry-to-entry interval, `*_call_ms`
  maximum call duration, `*_n` outer service calls intersecting the window, and
  `*_over100` intervals strictly greater than 100 ms.
- Window start to first service and final service to adoption are included as bounded
  intervals; if no service occurs, the entire window is the gap and count is zero.
  Thus setup may legitimately report loop_n=0. Calls already open at request are clipped
  to request time and counted once; calls still open at adoption are clipped there.
  No pre-request gap contaminates an attempt. Nested spans are never summed.
- `contexts`: bit mask of `diag::Phase` values observed on main at window/service
  boundaries. Decode using diagnostics_internal.h; it is not a worker phase or a
  duration measurement. `span` / `span_ms` identify the longest completed overlapping
  main operation. An unfinished span may appear only in contexts. These are observations,
  not proof that MQTT caused a gap; operations wholly between observations may be absent.
- Saturating 32-bit duration/count fields avoid wrap on an exceptionally late completion.
  Total window uses 64 bits. The host check bounds the worst-width SERVICE below the
  456-byte queued-field limit. There are no per-frame/sample/phase queue records.

Memory reporting now also samples `dmaLargestMin` at the existing busy-only sampling
points (main and worker). It resets with each attempt and appears in MQTT_CONNECT_MEM
and serial MQTT OWNER. No permanent idle/ONLINE heap walk was introduced.
Normal NET_HEALTH uses a fixed copied main snapshot for worker phase/id/age, lease and
backoff; it adds no health event. Status includes backoff too. Backoff is remaining
15-second rate-limit time only, not a promise of eligibility: link, lease, held resources,
bench bypass and give-up policy still apply. Health age describes snapshot capture time;
existing snapshot_age_ms shows how old that capture is when written.

## Verification and limitations

**355 host checks in 15 suites pass**: the existing 341 plus 14 service checks.
The operation harness only adds a mock for the new observer calls; its original assertions
remain. The owner heap mock now expects four walks (DMA largest is the fourth) and
initializes its new minimum. No pre-existing behavior assertion is removed.
New tests execute observer bodies for boundary clipping, no-service windows, strict
100 ms threshold, nesting, stale/duplicate completion, saturation, reset, completed span
attribution, context masks, record width, service call-site coverage, common result
adoption placement, normal health fields and backoff rollover. No compiler/hardware claim.

The parser tests and all 10/1 ms polling guards remain intact. The generated selected
profile sketch is removed because companion.ino changed. JP compiles only after review.
The car installation and raw F001/F002 evidence are untouched.

## Claude focused review request

Check the observer's clock/window semantics and all early/nested return paths, especially
request/adoption inside an already-active Loop or UI scope. Verify once-only emission for
success, failure and cancellation, and that omitted or tail intervals cannot hide a stall.
Check main-only ownership, overlapping-operation attribution limits, bounded field widths,
health snapshot sizing, DMA sampling cost/scope and unchanged client/admission behavior.
Re-run all suites and check C++ APIs/includes. Append concrete findings here. No build,
flash or hardware test before JP receives the verdict.

## Retained bench scope after review — three cases, issued one at a time

JP requested essentials only, approximately 20–30 minutes if cases pass; this is an
estimate, not a time guarantee. No SD/iPhone regression matrix is repeated.

1. Successful reconnect after a previous Latest/Live cycle: watch G-meter/spinner/touch,
   collect SERVICE/END/MEM and status, verify subscriptions and a subsequent image.
2. Two failed broker attempts while WiFi stays associated, then recovery: responsiveness,
   failure phase, absolute wait evidence, backoff, memory/stack and zero log drops.
   A fast rejection from the documentation endpoint cannot prove long-wait behavior;
   if necessary prepare a controlled delayed endpoint within this same case rather
   than claiming a timeout was exercised. Do not improvise an unreviewed firmware hook.
3. One hotspot down/up during a pending attempt: old epoch cannot revive the connection,
   bounded owner cleanup and next-attempt recovery. Reuse the same-IP evidence when
   assigned; no extra radio matrix. Include one media/retrieval refusal observation in
   a relevant case rather than a standalone test.

Keep stack >=2048 and internal largest >=20480, no resets/allocation failures/log drops.
Report measured service maxima and visible behavior; do not invent a new numeric UI
acceptance threshold. Unexpected gaps get attributed from contexts/spans and the full log
before deciding a correction is warranted. Additional testing needs an observed failure
or unresolved ambiguity. After acceptance: one ordinary car ride with full export; no
outage means normal-use evidence, not proof of reconnect responsiveness. Increment 3 is
limited to evidence-justified corrections and field rollout, not extra speculative scope.

## Claude review - September 25, 2026 (1cdbf41 against 36a9819)

**Verdict: cleared for JP's build and the three retained cases.** No blockers. One
one-line fix is recommended before building (N1). Host checks re-run: **355 pass in 15
suites**. Not compiled, per the handoff.

### Verified

- **Window clock.** `begin()` uses the owner's request timestamp, the same
  `esp_timer` millisecond clock as the observer's `now()`, immediately after an accepted
  `request()`. `finish(result.id)` is the last statement of the common `takeResult`
  branch, after READY acknowledgement, calibration enqueue and failure-budget handling.
  That gives exactly one SERVICE record for success, failure and cancellation.
  `request()` refuses while a result or loss is unconsumed or during Fault, so windows
  cannot overlap. A stuck worker leaves the window open without inventing a record, as
  documented.
- **Clipping and tails.** Every metric's `last` resets to the request time. Calls open at
  `begin` are counted once and their cost is clipped to the request time. `finish()`
  closes every tail gap and clips still-open calls at adoption. No pre-request interval
  leaks in, and a stall at the start or end of the window cannot be omitted.
- **Nesting and early returns.** `Call` is RAII. The depth counter collapses nesting and
  cannot underflow. The Loop observer lives in `diagop::Loop`'s constructor and
  destructor, so it covers the shutdown gatekeeper's early return. The `updateImuData`
  RAII covers the mutex-refused path. The request (`netCheckMqtt`) and adoption
  (`netMainTick`) both run outside any `lv_timer_handler` scope; a surrounding Loop scope
  is handled by the depth-at-begin count and the clip at finish.
- **Coverage.** A source search finds no uninstrumented `lv_timer_handler()` in authored
  code; the only remaining matches are two comments in video_stream.cpp. `lv_refr_now`
  is not a service call. That is conservative: time spent in it appears inside gaps
  rather than hiding them.
- **Bounded records.** SERVICE uses a 456-byte buffer, the queued-field limit, and the
  host check bounds its worst-case width. NET_HEALTH is written by the writer into a
  704-byte buffer. `DIAG_ENABLED` reaches the observer through `diagnostics_internal.h`
  -> `sd_diagnostics.h` -> `diagnostics_config.h`. `mainBreadcrumb()` is always compiled.
- **Reconnect behaviour is unchanged.** net_module only adds `begin`/`finish`, the health
  snapshot and a read-only backoff calculation. The worker only adds the DMA-largest
  sample inside the existing busy-only `sample()`. Client, deadlines, lease, admission and
  retry timing are untouched.
- **Tests not weakened.** The only edits to existing suites are the fourth heap walk in
  the owner sampler mock and observer stubs in the operation harness. All original
  assertions remain.

### Nonblocking

- **N1 - phase name truncated (recommended before build).**
  `DiagnosticsHealth::mqttWorkerPhase[12]` truncates `mqtt_exchange` (13 characters) to
  `mqtt_exchan` in NET_HEALTH. Use `[16]`, so bench evidence carries the correct phase
  label.
- **N2 - the measurement slightly inflates what it measures.** During an attempt, main
  now performs four heap walks every 20 ms, as does the worker. The main-side walks add a
  little to the UI and loop gaps they measure (expected well under 1 ms per sample, not
  measured). Keep this in mind if service maxima land near a threshold.
- **N3 - reading the new fields.** `contexts` is sampled only at observation boundaries,
  and `span` is the longest completed main operation. Both are attribution hints, as the
  handoff states, not causes.


## Retained case 1 — September 25, 2026, Codex

**PASS.** JP reports no G-meter freezing during MQTT reconnection and calls the test
passed. Reviewed source checkpoint a83c96a; exact firmware Git hash is not embedded.
Boot 132 remained unchanged throughout the supplied console. Evidence preserved in
`evidence/2026-09-25-p001-case1/` (ignored by Git); Downloads original retained.
- `132-current.log`: 117179 bytes; SHA-256 `a723434c0de8d5b6ca252ee400b19e7bebbbb7bdeffa8f0a48d13100df8a4b8a`; CRC32 `B3FE49E2`.
- `console.txt`: 12991 bytes; SHA-256 `1fa96ffc0a7a83916b97324f0b66c01aea87a42ed4c4de4261786292727e0196`; CRC32 `6417FB3F`.

USB export: 117179 bytes, CRC OK. The current snapshot includes prior boots; this result
uses boot 132 only. No new reset, logger error, queue/packet drops or stuck worker during
the retained interval. Subscriptions accepted (SUBACK remains unobserved).

- Prior Live completed normally: 161 frames / 60.426 s = 2.664 FPS, max frame gap
  962.311 ms. This is heap preconditioning, not a paired FPS regression comparison.
- Serial off at 11:37:57.450; on at 11:38:04.431, about 7 seconds later rather than
  within 2 seconds. A test-endpoint connection was already pending. Do not repeat case 1:
  the extra cancellation supplies useful evidence without invalidating real recovery.
- Attempt 2 (test) result=cancelled, TCP setup 5003 ms, total 5016 ms. SERVICE window
  5027 ms: UI gap/call 27/17 ms, IMU 23/5 ms, loop 23/23 ms; all over100 counts zero.
  The main on command invalidated the epoch while native TCP setup ran; owner cleanup
  completed before the replacement real attempt. This is cancellation during a long
  TCP wait, not an uninterrupted failed attempt or a WiFi-flap test.
- Attempt 3 (real) result=ok: DNS 1 ms, TCP setup 164 ms, TLS 627 ms, MQTT exchange
  58 ms, total 888 ms. SERVICE window 899 ms: UI gap/call 23/15 ms, IMU 26/6 ms,
  loop 26/26 ms; all over100 counts zero. Context=idle; longest completed main span
  calibration 1 ms. UI_n=111, IMU_n=111, loop_n=113.
- Worker PSRAM/internal TCB confirmed; stack minimum 7228 >=2048. Real-attempt internal
  largest and DMA largest minima 51188 >=20480. Retained logger/media largest minimum
  24564 also stays above gate. Logger drops=0, truncated=0, error=none; owner drops=0,
  cancelled=1 expected, stuck=0, lease=0 and connected=1 after recovery.
- Post-recovery Latest completed: 36947/36947 bytes, displayed in 1604 ms. Touch
  navigation back from G-meter and Latest request are present in the console.
- Boot setup's separate SERVICE id=1 reports loop_n=0 / loop_gap_ms=624 because setup
  has not entered Arduino loop yet; UI and IMU were serviced. This is not a 624 ms
  reconnect freeze. No >1 s LOOP_GAP record appears for boot 132 in this export.

Next retained case: two uninterrupted failed broker attempts with hotspot kept connected,
then real recovery, with one USB retrieval-entry refusal during a held reconnect lease.
No new code, rebuild or extra regression case is needed for this result. Case 3 hotspot
flap remains pending; the serial on cancellation does not replace it.


## Retained case 2 — September 25, 2026, Codex

**PASS.** JP reports the G-meter remained responsive without freezing. Same boot 132,
reviewed source checkpoint a83c96a; no code/rebuild since case 1. Evidence preserved under
`evidence/2026-09-25-p001-case2/` (ignored by Git), original Downloads export retained.
- `132-current (1).log`: 133865 bytes; SHA-256 `d2c9e4349e75fcf73f2ef4893bd2684d5a4709f05f0d3e1fbfd3e37efbe9fcf6`; CRC32 `3634D6E3`.
- `console.txt`: 11245 bytes; SHA-256 `1c2fb9e5fca9f14db10877de4a02dc034f70b6dde47ec9ae69435ca27056cb14`; CRC32 `6FADCA72`.

USB capture: 133865 bytes, 1.23 s, CRC OK. Only attempts 4–6 are this case; prior attempts
remain in the full export. No reset, new cancellation, stuck worker, queue/packet drops,
truncation or logger error in the retained case. WiFi remained connected in the supplied
observations; the retrieval status line's cached link=down is not a WiFi driver event.

| Attempt | Result | TCP setup | Total | UI / IMU / loop max service gap |
|---|---|---:|---:|---:|
| 4 test | tcp_setup_failed, not cancelled | 5003 ms | 5022 ms | 21 / 21 / 21 ms |
| 5 test | tcp_setup_failed, not cancelled | 5003 ms | 5022 ms | 21 / 21 / 21 ms |
| 6 real | ok | 254 ms | 1087 ms | 21 / 20 / 20 ms |

All SERVICE over100 counts zero. Attempt 4 SERVICE window 5033 ms, UI/IMU calls 801;
attempt 5 window 5026 ms, calls 710; attempt 6 window 1100 ms, calls 156. Completed
main spans were calibration/screen_nvs, each at most 1 ms, contexts=idle.

Attempt 5 began at up_ms=510758, 15004 ms after attempt 4 main SERVICE/adoption at
495754 (END capture 495743). This supports the retained 15-second post-completion
backoff. JP sent on after the second failure, and the real attempt recovered. Its
DNS/TCP/TLS/MQTT phases were 122/254/637/54 ms respectively. The two real five-second
TCP waits meet the long-wait requirement; no controlled endpoint is needed for this case.
They do not independently exercise slow DNS, TLS failure or CONNACK failure on hardware.

USB log mode on during attempt 4 refused with `mqtt_reconnecting`; server remained off.
Worker stack 7228 >=2048, PSRAM stack/internal TCB confirmed. Failed-attempt internal
largest minima 53236; recovery largest and DMA largest 51188 >=20480. Retained logger
largest minimum stayed 24564. Owner lease returned to zero, connected=1, stuck=0,
cancelled=1 unchanged from case 1, all drop/failure-to-publish counters zero.

Next and final retained bench case: one hotspot down/up while a connection is pending,
then normal recovery. No new firmware or repeat failure case is indicated by this result.
