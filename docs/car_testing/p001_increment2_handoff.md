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
