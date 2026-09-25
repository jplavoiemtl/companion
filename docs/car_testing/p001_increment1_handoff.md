# P001 increment 1 â€” code-review handoff

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
provenance. Its CONNACK/empty-byte waits and optional chunked write loop use
`vTaskDelay(1)` plus an absolute operation guard. Buffered bodies yield every 64 bytes;
the four-byte remaining-length parser waits through readByte only. Ordinary oversized
packets are counted and drained inside the unchanged absolute 5 s deadline, keeping
the session. Malformed lengths and remaining lengths above 16 KiB close the session.
Deadline/cancellation still closes rather than leaving a partly drained packet. CONNACK type/length/flags are checked. Finite local buffer copies
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

Driver association/disconnection/LOST_IP/STOP events invalidate epochs even for
same-IP recovery; GOT_IP invalidates only when the link was previously down. A repeat
GOT_IP while already up preserves the session. Each new connection also gets a distinct message epoch. Only the
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

All **341 checks in 14 suites pass** after the focused review fixes below. The initial
implementation had 329; the MQTT-owner suite now has 51 checks (12 added). Existing
suites still pass; polling and oversize assertions now express the reviewed correction.

| Suite | Checks |
|---|---:|
| HTTP lifecycle / transfer | 43 / 35 |
| Log time / media admission | 20 / 16 |
| MQTT owner / network diagnostics | 51 / 16 |
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

## Claude code review - September 25, 2026 (commit d7752d9)

**Verdict: not yet cleared for build.** Two blockers (B1, B2), both small. Everything else
matches revision 2. I re-ran the 14 host suites: 329 pass. Per the handoff, I did not
compile; JP's build is the compile check.

### Verified

- **Ownership:** no PubSubClient include or direct client use remains outside
  `src/net/mqtt_client` and the worker. companion.ino, net_module.cpp and imu_module.cpp
  are migrated. Callbacks run on main from copied, epoch-checked slots.
- **Boot ordering:** `netInit()` (companion.ino:2422) registers WiFi events before
  `initWiFi()` (:2436), so the first GOT_IP sets the link and requests are admitted.
- **DNS:** slot allocation, the ERR_OK / ERR_INPROGRESS / immediate-error paths,
  submission ERR_MEM and resolver ERR_MEM (both uncounted), and tombstones released only
  by `completeDns`, even after abandonment. Hostname storage outlives submission.
- **READY versus epoch race:** READY is published atomically with epoch validation.
  `acknowledgeReady()` re-validates. A revoked success is cleaned by the worker with a
  loss acknowledgement, and `request()` refuses until main has consumed both result and
  loss.
- **Split TLS:** `setPlainStart()` on every attempt; `connect(IPv4, port, hostname, CA)`
  then `startTLS()`. The facade refuses implicit connect and all I/O until `ready`, which
  is set only after TLS succeeds.
- **Timing:** constants 15000/35000/40000/5000; per-phase admission against the attempt
  clock; one 5 s absolute deadline per ONLINE `loop()` call.
- **Lease:** requested only after DNS. `arbitrate()` grants only while `requestedLease`
  and phase Lease hold, so a late grant is impossible after the worker's 100 ms expiry.
  The lease is released at ack or cleanup and retained on Fault. Media refusal covers
  Latest, Back, Live, the prepare backstop and notifications. Retrieval refusal sits in
  the shared `entryRefusal()` (panel and USB). The handover cannot meet a held lease,
  because arbitration excludes pending display.
- **Patch versus upstream 2.8** (line-ending-insensitive diff): the only changes are the
  guard, `vTaskDelay(1)` waits, CONNACK validation, packet bounds and counters. No yield-only
  wait remains.
- **Tests:** existing media and retrieval assertions were kept, and lease cases added.
  Replaced network assertions now pin the same calls on the owner. `recordAt` keeps
  queue and drop behaviour. Shutdown hooks are non-waiting.
- **APIs:** the core 3.3.11 `Client` has exactly the 12 pure virtuals the facade
  overrides; `std::min` is exported by Arduino.h; the lwIP and heap APIs exist.

### B1 - an oversized incoming message now drops the whole connection (blocker)

`readPacket` closes the session when `length + len > bufferSize`
(OwnedPubSubClient.cpp:355-358). Upstream 2.8 read and discarded such a packet and kept
the session. Design section 4 says oversized incoming packets are "discarded and counted".
Any message over about 505 bytes on the image, power or energy topics (retained or
periodic) would now cause disconnect, 15 s backoff, a new TLS handshake holding the
lease, and repeat. The old client ignored those messages silently, so their existence
today cannot be ruled out.
**Fix:** discard the remainder inside the existing absolute 5 s operation deadline, count
it, keep the session. Close only on malformed length encoding or a hard cap (for example
a remaining length above 16 KiB). Add a host case.

### B2 - the worker polls TLS about 1000 times a second on every screen (blocker)

`CONFIG_FREERTOS_HZ=1000`, so `vTaskDelay(1)` is 1 ms. While ONLINE, each worker turn
calls `client.loop()`, which runs `NetworkClientSecure::connected()`/`available()`: an
mbedTLS read plus lwIP socket calls under the TCP/IP core lock. That is about 1000 times
a second, against about 40 per second when main called `loop()`. Each turn also scans
the PSRAM stack (`uxTaskGetStackHighWaterMark`), and heap walks run every 20 ms
permanently, on both the worker and main. This competes with Live/image HTTPS for the
core lock and PSRAM bandwidth all the time, not only during reconnects, and the first
hardware gate would not reveal a Live FPS regression.
**Fix:**
- Idle and ONLINE turns use `vTaskDelay(pdMS_TO_TICKS(10))`. Keep 1-tick delays only
  inside active waits.
- Take the stack high-water mark at phase boundaries and at most about once a second.
- Sample heap, on worker and main, only while an attempt is busy, as the design's
  "while worker is active" intends.

### Nonblocking

- **N1 - per-byte delay.** The remaining-length and body loops of `readPacket`
  (:340, :377) delay 1 ms per byte even when data is already waiting, so a 500-byte
  message takes about 0.5 s and B1's discard would crawl. `readByte()` already delays
  when nothing is available. Drop the unconditional per-iteration delay (or take it every
  N bytes) and classify these loops in the static guard as waiting only via `readByte`.
- **N2 - GOT_IP while the link is already up.** A same-IP DHCP renewal would bump the
  epoch and drop a healthy ONLINE session. Ignore `linkEvent(true)` when `status.link`
  is already true. A genuine same-IP recovery still passes through CONNECTED or
  DISCONNECTED (link false) first.
- **N3 - uncounted drops.** The stale or not-connected path in `receive()` and the stale
  discard in `takeRx()` drop messages without counting them. Add them to `rxDrops` or a
  separate stale counter.
- **N4 - dispatch point moved.** MQTT callbacks now run at the top of
  `runBackgroundTick()`, which also runs inside setup keep-alive loops, instead of in
  `loop()` after the tick. MQTT only connects at the end of setup, so no new path is
  exercised today. Note it in the handoff for future setup changes.
- **N5 - compile.** Not compiled, per the handoff. The spot checks above passed, but
  JP's first build is the real compile check.

After B1 and B2, with N1 recommended alongside B1, I only need to re-check those diffs.
The first hardware gate stays as planned: one worker TLS handshake and CONNACK, checking
PSRAM stack placement, a stack margin of at least 2048 bytes and an internal largest
block of at least 20480 bytes.

## Codex focused fixes for review e51ef5a — September 25, 2026

Status: implemented and host-checked; awaiting Claude's focused re-check. No firmware
compile or flash. The Claude review above is preserved as the pre-fix assessment.

- **B1 / N1:** packets above the 512-byte buffer and with remaining length at most
  16384 bytes are counted once, drained without callback/stream delivery, and leave
  the connection open. The next packet starts at the correct byte. A malformed
  remaining-length encoding, invalid PUBLISH topic/message-ID length, or remaining
  length above 16384 closes. Incomplete discard still obeys the original absolute
  operation deadline/cancellation; it cannot keep a partially consumed session alive.
  There is no per-byte sleep for buffered data: empty waits delay one tick and body
  processing delays one tick per 64 bytes. Length decoding is bounded to four bytes.
- **B2:** outer idle/ONLINE turns delay `pdMS_TO_TICKS(10)`. DNS, lease, CONNACK and
  empty-input active waits keep one-tick delays. Stack scans happen at phase boundaries
  (including successful subscription completion and failed-attempt cleanup) and at
  most once a second otherwise. Both worker and main heap sampling require busy state;
  the shared sampler also rechecks busy state and attempt ID before storing minima.
  A cancelled ONLINE cleanup can temporarily be busy under the existing state model;
  normal idle and healthy ONLINE operation do not walk the heap.
- **N2:** an already-up GOT_IP does not invalidate the epoch or clear queues. An actual
  down/up sequence still invalidates, including recovery with the same IP address.
- **N3:** rejected stale/disconnected/stopped receives and dequeues increment `rxDrops`.
  Invalidation also counts queued RX messages before clearing them, once per message.
  The separate parser packet-drop counter remains separate from application RX drops.
- **N4 noted:** callbacks dispatch at the top of `runBackgroundTick`, including setup
  keep-alive loops. MQTT currently connects at the end of setup. Future setup reordering
  must preserve callback readiness before connecting; no dispatch change in this fix.
- **N5 unchanged:** these are source-body simulations/static guards. JP's eventual build
  remains the C++ compilation check, after Claude clears this diff.

Validation: **341 checks / 14 suites**, including **51 MQTT-owner checks**. New simulations
cover 600-byte and 16 KiB discard with following-packet alignment, deadline/cancellation
under slow input, hard cap and malformed lengths, bounded buffered-body yields, repeated
GOT_IP versus down/up, stale RX accounting, stack sampling cadence, idle heap suppression,
and an attempt ending/changing while heap measurements are in progress. No hardware
performance or timing result is claimed.

### Focused re-check request

Review this commit against e51ef5a, concentrating on B1/B2 and N1–N3: packet framing and
absolute deadline during discard, IDLE0 opportunities without per-byte latency, sampling
scope/cadence, epoch preservation, and once-only RX counting. Check the updated host
simulations and C++ correctness. Append the verdict here. No build or flash. After
clearance, the first hardware gate remains one worker TLS handshake/CONNACK with the
already specified stack-placement, 2048-byte stack-margin and 20480-byte largest-block
gates. No hardware case is issued before that review.
