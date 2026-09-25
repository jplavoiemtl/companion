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
