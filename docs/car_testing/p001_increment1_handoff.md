# P001 increment 1 — code-review handoff

September 25, 2026. Author: Codex. Status: ready for Claude review, **not build clearance**.
JP approved revision 2 and all four section-11 decisions after focused review e3ae528.
Scope follows section 10 increment 1. No firmware compile, flash or hardware action.

## Branch and return point

- Implementation branch: `codex/car-improvements-p001`, requested by JP during implementation.
- Base/return point: `iphone-log-retrieval` at `e3ae528` (reviewed design, pre-P001 firmware).
- The existing car installation is unchanged. Do not infer its flashed commit from HEAD.
- Historical iPhone plan, generated UI and raw car evidence are unchanged.

## What changed

`src/net/net_worker.cpp/.h` owns both MQTT transports and the project-local client.
The worker is lazy, persistent, core 0 / priority 1, with a 12288-byte PSRAM stack and
internal static TCB. No automatic internal-stack fallback. Queue allocation is checked
against an 8 KiB PSRAM limit; fixed control metadata has a 2 KiB compile-time limit,
excluding the static TCB. Stack reporting checks a local worker address, not just the
allocation pointer. No worker UI, calibration, NVS, SD formatting or USB printing.

`src/net/mqtt_client/` contains renamed PubSubClient 2.8, original MIT license and
provenance. Its CONNACK wait, byte wait, remaining-length reader, packet reader and
optional chunked write loop each use `vTaskDelay(1)` plus an absolute operation guard.
Oversize/malformed packets close rather than run an unbounded discard loop. Rejected
packets have a counter. CONNACK type/length/flags are checked. Finite local buffer copies
and header encoders are not network polling loops. The Client facade forbids implicit
connect and all MQTT I/O before TLS readiness. No installed library was edited.

Connection work is DNS (15 s from dispatch including submission), TCP setup (5 s),
TLS (5 s), MQTT exchange (5 s absolute), subscriptions (2 s nominal); attempt budget
35 s, stuck threshold 40 s. Native operations cannot be forcibly preempted. Both
transports retain constant `setConnectionTimeout(5000)`. Secure setup uses the resolved
IPv4 plus original hostname and CA, with `setPlainStart()` then `startTLS()` before
MQTT bytes. TLS error freshness stays unknown when startTLS did not refresh lastError.

Two DNS slots survive timeout/cancellation as tombstones until callback acknowledgement.
Submission mailbox ERR_MEM and resolver-table ERR_MEM release/defer without consuming
the initial failure budget. IPv6 literals are explicitly refused. Slots are never freed
because the normal ~21 s resolver schedule elapsed. No application DNS cache or extra
hostname lookup is added.

Main grants the resource lease only after DNS, excluding active/pending media and any
non-OFF retrieval mode. Wait expiry at 100 ms revokes even a late grant before TCP.
The 20480-byte internal-largest gate is checked before TCP/TLS allocation. All media
paths refuse while the lease is held; shared retrieval admission refuses both panel
entry and USB `log mode on`, with a readable retry message. Existing raw USB file
transfer commands were not given a new admission policy.

Driver association/disconnection/GOT_IP/LOST_IP/STOP events invalidate epochs even for
same-IP recovery. Each new connection also gets a distinct message epoch. Only the
worker closes its transports. Failure results are published after cleanup; a successful
READY retains the lease until main adopts it. A cancelled READY has an owner cleanup
acknowledgement. A stuck worker holds resources and any lease, posts one visible notice,
and may recover after late cleanup; no forced task deletion/socket close. Shutdown
invalidates and returns without waiting.

`net_module.cpp`, `companion.ino`, and IMU publishers now use snapshots and fixed queues.
Main handles at most two RX messages per tick and checks a 2 ms dispatch budget between
callbacks; existing image work is not made asynchronous by this increment. Callback
contents and calibration reporting still run on main. Setup services background work
while waiting and retains three attempts/three-second intervals; held faults cannot
trap setup. Five genuine initial failures and 15 s post-completion retry remain.
Bench off/on/status remain available while connection work is pending; test connections
never use production credentials. First bench-off delay remains five seconds after
owner cleanup, and an explicit restore may retry after cleanup.

Publish admission means queued, not sent. Worker completion supplies accepted status.
Motion/calibration records retain worker completion stamps; IMU telemetry stays quiet
as before and is counted in owner totals, avoiding extra SD queue pressure during a
current.log snapshot. Wire limit remains 512 bytes despite the 768-byte TX slot. Actual
motion/IMU topic lengths are 16/13 bytes; calibration is 21. A representative zero-valued
IMU JSON is 416 bytes (not a worst-case claim), fitting 436 bytes with topic/header.
Larger values can exceed the old wire limit and are now visibly counted as size refusals;
neither wire buffer nor application payload formats were expanded.

`diag::recordAt` preserves an explicit captured Stamp. Existing `record` delegates to it
with a current stamp and keeps its queue, capacity and drop behavior. MQTT END and
publish completion can therefore arrive later on main without false event timestamps.
BEGIN/END/MEM and `[MQTT OWNER]` provide phase, timing validity, DNS result, stack placement,
heap minima, lease, cancellation, fault, queue and packet counters. Worker duration is
not wrapped as a blocking main-thread MQTT span. Existing real LOOP_GAP evidence remains.

## Host verification

All **329 checks in 14 suites pass**. The previous 288 checks remain (assertions tied to
old APIs/locations were adapted); 39 MQTT-owner checks and two admission checks were added.

| Suite | Checks |
|---|---:|
| HTTP lifecycle / transfer | 43 / 35 |
| Log time / media admission | 20 / 16 |
| MQTT owner / network diagnostics | 39 / 16 |
| Operation / reader session | 8 / 28 |
| Retrieval mode / UI | 42 / 15 |
| Browser / touch | 20 / 19 |
| USB connection / logger gate | 16 / 12 |

Run from the project root: `Get-ChildItem tools/tests/*.test.cjs | ForEach-Object { node $_.FullName; if ($LASTEXITCODE) { throw $_.Name } }`.

The MQTT suite executes adapted real function bodies for epoch invalidation, phase
admission, DNS completion/tombstones/ERR_MEM, lease arbitration and expiry, RX/TX limits,
failure budget, delayed CONNACK and trickling/oversized packet input. Static assertions
cover ownership, native API order, no plaintext/hidden connect, stack/time constants,
callback boundary, timestamp/record capacity and classified polling loops. It catches
new unclassified polling loops. Existing host tests still cover media/handover/retrieval
and USB behavior. `git diff --check` passes.

Limits: JavaScript source-body simulations do not compile C++ or prove native API ABI,
FreeRTOS scheduling, TLS from a PSRAM stack, stack margin or real heap peaks. Full per-
attempt UI/IMU service-gap instrumentation and hardware success/failure/flap validation
are increment 2. Minimal safety/phase reporting is present now; this is not a claim that
the responsiveness or memory gates have passed.

## Requested Claude review

Review this increment against approved design revision 2 and current installed 3.3.11
sources, concentrating on:

1. Sole ownership and every moved application call; callback generation/queue bounds,
   publication completion versus enqueue, setup and bench retry policies.
2. DNS context lifetime, immediate errors, cancellation, and the final READY versus
   epoch-change race. Check cleanup acknowledgement before replacement attempts.
3. Split TLS hostname/CA verification, Client facade, every local PubSub polling loop,
   absolute packet bounds and watchdog opportunities. Check C++ include/API correctness.
4. Lease admission in both directions, late-grant rollback, held fault and nonwaiting
   power hooks. Confirm existing media/retrieval assertions were not weakened.
5. Original completion stamps, bounded records, stack/heap reports and host coverage.

Append a dated review here or in a linked review file; report concrete blockers and
nonblocking findings. Do not build or flash. JP builds only after review clearance.
The selected generated `build/build_amoled-1-8-core-3-3-11/sketch/companion.ino.cpp` is
removed because companion.ino changed. No hardware case is issued by this handoff.
After clearance, the first gate is one real TLS/CONNACK on the worker, checking PSRAM
placement, >=2048 stack margin and >=20480 internal-largest, before failure cases.
