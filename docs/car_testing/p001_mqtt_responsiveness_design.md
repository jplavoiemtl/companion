# P001 - Responsive UI and IMU during MQTT recovery

Revision 2, September 25, 2026. Author: Codex. Status: DESIGN FOR CLAUDE REVIEW AND
JP APPROVAL. No firmware edits, build, flash or hardware case is authorized by this
text. Source baseline is the current iphone-log-retrieval checkout. Related evidence:
[field journal](field_journal.md), F001/F002 and I001; bounded phase timing requested
by JP is part of this design, resolving the need to choose whether to include P002.

## 1. Problem, goal and scope

F001 successful reconnects blocked the main loop for 7985/8018 ms. F002 failed
reconnects blocked it for 8771/5115 ms during a flapping link. The same loop services
LVGL and IMU acquisition, explaining both G-meter and spinner freezes. Reconnection
must be asynchronous to main, for success, failure, partial MQTT replies and WiFi churn.
Do not change WiFi radio/retry settings, IMU sampling/core placement, SD behavior,
retrieval mode rules, calibration math or image/Live pacing in this work.

Acceptance target during a reconnect-only window (no user-requested image/Live):
maximum UI service and IMU service gaps <=100 ms; no >=1000 ms main-loop gap; touch,
spinner and dot remain usable. Record measured maxima, not just visual impressions.
This is a bench target, not a hard-real-time guarantee. An unrelated image GET can still
block main; log that operation and do not claim P001 makes all networking nonblocking.

## 2. Selected architecture and alternatives

Recommend a persistent MQTT owner worker. Retain PubSubClient 2.8 protocol behavior,
but remove its instance and both MQTT transports from application access. ONLY the
worker calls connect, connected, loop, publish, subscribe, disconnect, stop, configuration
setters or TLS error APIs. Main uses bounded snapshots, nonblocking mailboxes and queues.
WiFi remains managed by the existing main/driver paths. Image and Live retain their own
single shared HTTPS transport; they do not share the MQTT secureClient object today.
Their resource competition is TLS heap usage, not an identical client instance.

Why own ALL MQTT operations: moving only connect leaves races with connected() (which
can mutate state), direct publishes, bench off/on, and loop reads, which can themselves
wait. A connection-only handoff is smaller initially but spreads ownership gates across
every caller and leaves receive stalls on main. A permanently owned client is clearer.

Alternative considered: esp-mqtt. Its dedicated event task is attractive, but changes
client, callback, retry, outbox and TLS configuration at once and does not by itself
expose all requested connect-phase boundaries. Reserve that option if review shows
retaining PubSubClient needs more invasive changes than the bounded patch below.
Merely lowering timeouts is not sufficient; successful recovery must be preserved.

Worker recommendation: lazy, boot-retained 12288-byte PSRAM stack, internal static TCB,
core 0, priority 1. Main/IMU/SD writer remain on their existing core. Core 0 choice keeps
TLS CPU work off main's core and below WiFi/lwIP task priorities; hardware verifies
contention and watchdog behavior. Use vTaskDelay of at least one tick on every continuing polling turn. The already validated
HTTP worker establishes PSRAM/lwIP viability, not MQTT/TLS stack sufficiency. Measure
worker high-water; require >=2048 bytes in retained gates. No automatic internal-stack
fallback or larger allocation on failure: stay offline, report worker_alloc, preserve UI.

## 3. Installed-source basis (review these contracts)

Verified locally, Arduino ESP32 3.3.11 internal package and PubSubClient 2.8:
- NetworkClientSecure.h exposes setPlainStart(), startTLS(), and
  connect(IPAddress, port, originalHostname, CA, clientCert, privateKey).
- NetworkClientSecure.cpp:147-164 calls start_ssl_client and omits the handshake when
  plain-start is set; :167-181 performs that handshake and clears plain-start on success.
- ssl_client.cpp uses the supplied hostname in mbedtls_ssl_set_hostname. The pre-handshake
  operation includes socket connect AND TLS configuration/CA setup; its duration is not
  pure TCP time. This distinction must appear in records and analysis.
- PubSubClient.cpp:181-196 reuses an already connected transport; :255-273 waits for
  CONNACK; :279-280 reports -2 when underlying transport connect fails. Missing CONNACK
  is -4. F002 -2 narrows failures to network establishment, not MQTT exchange.
- PubSubClient readByte/readPacket use repeated per-byte waits. A five-second socket
  timeout alone does not enforce an absolute packet or connection deadline.
- Network.hostByName uses blocking getaddrinfo without a caller deadline. Use the
  installed lwIP dns_gethostbyname_addrtype via tcpip_try_callback instead (see below).
- diagnet's application counters/span IDs and diagnostic probe state are main-owned.
  Do not invoke their current Span/probe methods from the new task without redesign.

Package paths for reproducibility: Arduino15/internal/
esp32_esp32_3.3.11_b0d8b7bad2896d0b/libraries/{NetworkClientSecure,Network};
PubSubClient_2.8_48867b22d3bf7501/PubSubClient/src; installed esp32s3-libs/3.3.11
lwip headers for DNS and tcpip callback APIs. No edits to global installed libraries.
Primary target remains the accepted 3.3.11 profile. Claude verified that 3.1.3 exposes
the same split APIs; use one implementation, with no legacy capability branch. Preserve
certificate validation on both. This source verification is not a new hardware acceptance
claim for the older profile.

## 4. Ownership and application integration

| Resource/action | Owner and contract |
|---|---|
| PubSubClient, MQTT secure/plain transports, CA setup, sockets | Worker exclusively, lifetime retained; no raw pointer returned |
| WiFi events | Existing event callback updates a small link epoch snapshot; no MQTT/UI calls |
| UI, calibration, IMU, image notification processing | Main only; incoming MQTT data copied before callback return |
| Connection/retry policy commands | Main supplies desired target/profile/shutdown generation, worker applies |
| State/status | Worker publishes fixed-size value snapshot; main reads without network access |
| Logging | Worker records timestamps/counters into bounded mailboxes; main formats SD/USB records |
| HTTPS media transport | Existing main owner; coordinated resource admission, never borrowed by worker |

Required call-site migration (source search is a completion check):
- companion.ino globals/NetConfig: replace supplied client pointers with owned net module.
- callbackMqtt: dispatch copied image/power/energy messages on main; preserve current
  parsing and notification suppression. No LVGL or requestLatestImage from worker.
- updateConnectionStatusUI, health capture, setup, loop: snapshot accessors only;
  remove direct mqttClient.loop()/connected()/state().
- myCalibMqttSender, IMU telemetry, immediate and periodic motion publishes: bounded
  enqueue API, not direct publish. Calibration report after reconnection runs on main
  via a generation-checked READY notification and queues its data.
- Serial off/on/status: update desired target, invalidate epoch, return immediately;
  worker cleans up and applies it. Never send credentials to the documentation endpoint.
- initMQTT and setup's keep-alive path: start requests and service runBackgroundTick
  while waiting for state changes, not call a blocking client. Existing three setup
  attempts/three-second inter-attempt delay, five initial failures total and 15-second
  normal retry interval remain policy, with asynchronous completions as the counters.

Queues proposed: RX four fixed 512-byte payload slots plus bounded topic-category and
length metadata; TX four 768-byte payload slots with fixed topic enum; PSRAM allocation
once before worker starts. No unbounded String/heap queue; checked size caps and allocation
failure keep MQTT offline. Validate real topic/payload sizes and PubSubClient's existing
512-byte wire buffer at implementation. A larger TX slot does NOT permit an oversized
MQTT packet; reject any topic+payload exceeding the configured wire limit, with a counter.
Unknown topics/oversized incoming packets are discarded and counted. Four-slot RX full
never blocks worker: drop newest and report rx_overflow; do not silently replay later.

All messages carry connection generation and monotonic receipt/enqueue times. Main
handles <=2 inbound messages per loop, within a 2 ms dispatch budget excluding existing
image operations that are separately attributed. Discard old-generation RX. Discard TX
on loss/reconfiguration; no stale motion/calibration replay across recovery. When full,
reject new publication with a counted reason (no blocking wait). Preserve existing
connected-only production semantics. Initial subscriptions are issued by worker before
READY. Subscribe return remains local acceptance, not SUBACK proof, as today.

Publish logging must distinguish enqueued from transport acceptance: enqueue true is
NOT existing accepted=1. Worker returns bounded completion records with accepted and
ack=unobserved. Main updates diagnostic counters accordingly. On lost completion capacity,
count it explicitly; never fabricate success. Critical READY/result/stop acknowledgements
have dedicated slots and cannot be displaced by telemetry. No serial printing or SD
formatting while holding cross-task locks. Locks protect fixed copies only, never I/O.

## 5. State model, link flapping and retries

Worker states: IDLE, WAIT_LINK, WAIT_BUDGET, DNS, TCP_SETUP, TLS, MQTT_CONNECT,
SUBSCRIBING, ONLINE, CLEANUP, BACKOFF, STOPPED; FAULT_HELD if progress cannot finish.
Only one attempt/socket set exists at a time. Each attempt has a unique ID and copies
an immutable target descriptor plus link/command epoch. Bump link epoch on every driver
STA_DISCONNECTED, new association/IP identity, and station shutdown. It must detect
loss/reconnect to the same IP/profile; comparing IP alone is insufficient.

Main invalidates visible connected immediately on a link epoch change, even if the
worker is blocked. A stale successful attempt cannot become ONLINE or publish callbacks.
After every phase and before subscriptions/READY, worker rechecks desired epoch and
shutdown. On mismatch it stops its own transport and enters CLEANUP. Never reuse the
socket/client for the new epoch until owner cleanup is acknowledged. Driver callbacks
and main NEVER call stop/shutdown on the worker's fd or delete its task.

Retain 15 s backoff measured AFTER owner cleanup/completion, not start, on failures and
flap cancellation. Do not restart a backoff on each repeated no_ap_found event; newest
valid configuration replaces desired state but does not launch overlapping attempts.
Link down pauses eligibility. Initial budget counts genuine real-target failures, not
cancelled stale attempts, resource deferrals or test-endpoint attempts. Once a real
connection has succeeded, continue retries without the initial give-up ceiling, as today.
No new WiFi stability delay is introduced in this change.

A timeout or cancel has two meanings: logical invalidation is immediate; resource release
occurs only when worker returns and cleans up. Main must not report release merely because
it reached a deadline. After 40 s without attempt completion, report one worker_stuck
notice and counter, remain disconnected/FAULT_HELD, retain buffers/task and any acquired resource lease,
refuse new connects and media requiring a held lease. A fault before lease acquisition
does not reserve TLS resources or exclude media. Late cleanup clears the fault explicitly;
otherwise restart is the recovery path. Main/UI/IMU continue. No unsafe forced deletion.

## 6. Phase timing and bounded transport behavior

Attempt wall budget: 35 s nominal from dispatch through subscriptions/READY, with DNS
wait 15 s, TCP socket connect 5 s, TLS handshake 5 s, MQTT exchange absolute 5 s,
and subscription setup 2 s (3 s remaining nominal scheduling/setup allowance).
worker_stuck threshold: 40 s from the same dispatch timestamp. Neither clock resets
on phase changes. This accommodates the native DNS first-server retry/failover schedule
without the old 5 s cap. A resolver needing more than 15 s can still time out; its late
callback remains safe and retryable. Timing will establish actual field behavior.

setConnectionTimeout is ALWAYS 5000 ms on both secure and plain MQTT transports.
It also sets persistent socket/read/write timeout behavior; never replace it with a
shrinking attempt remainder. Before TCP, TLS, MQTT or subscriptions, require enough
remaining attempt budget for that phase's full nominal allowance, otherwise clean up
with attempt_budget without starting it. TLS handshake limit remains 5 s; MQTT/packet
absolute deadlines stay 5 s. No stage shrinks ONLINE socket timeouts. User-code checks
also invalidate progress at the overall budget; they cannot preempt a native call.
Allocation/CA parsing, scheduling and cleanup are not hard-real-time bounded. Record
phase/attempt overruns honestly and use FAULT_HELD policy, not forced resource release.

### DNS

Use two boot-retained request slots containing hostname storage (max 253 chars plus NUL),
attempt ID, epoch, callback result and state. Submit with tcpip_try_callback to TCP/IP
thread; perform dns_gethostbyname_addrtype there, IPv4 for the currently IPv4 STA/hotspot
scope. Literal configured addresses bypass DNS and report skipped_literal. Cached success,
asynchronous completion and immediate error all have explicit paths. Poll/notify worker
in <=20 ms intervals using vTaskDelay of at least one tick, enforcing a 15 s wait
from dispatch (including submission delay) even if resolver still owns the request.

Submission failure: if tcpip_try_callback returns ERR_MEM, no callback was queued;
release that matching slot immediately, report dns_submit_busy and defer for the normal
15 s retry interval without consuming the initial-failure budget. Other immediate
submission errors follow the same no-retained-callback cleanup with their error code.
Inside the TCP/IP callback, dns_gethostbyname_addrtype returns ERR_OK (copy result and
release), ERR_INPROGRESS (retain until DNS callback), or an immediate error including
ERR_MEM from resolver-table exhaustion (release, report dns_resolver_busy for ERR_MEM,
defer without counting a broker failure). Never wait for a callback on an immediate
error path. Distinguish submission mailbox saturation from resolver table saturation.

A timed-out/cancelled slot is a tombstone until queued submission and any DNS callback
finish. lwIP copies the hostname into its table when the lookup is submitted; hostname
storage only needs to survive that submission. Keep the complete fixed slot nonetheless:
its callback argument, ID and epoch must survive until completion. Completion releases
its own matching slot even after invalidation but cannot advance a stale attempt.

Installed schedule reviewed by Claude: roughly 7 s per configured DNS server, at most
three servers, hence about 21 s until a callback (NULL on failure), plus TCP/IP mailbox
and timer scheduling delay. This is the normal resolver lifetime bound, NOT permission
to free a slot at 21 s: application scheduling stalls can extend it. Callback/submission
acknowledgement is the only release authority. With 15 s DNS wait followed by 15 s
backoff, two retained slots cover ordinary overlap, including cancellation. If either
submission or callback is delayed abnormally, bounded slots still prevent unsafe reuse.
If both are retained, report dns_slots_busy and defer; do not allocate a third or reset
DNS globally. Repeated saturation/stuck resolver is visible in status; restart is the
recovery path if callbacks never arrive. No TLS lease is held for these DNS tombstones.
No application DNS cache, prewarming lookup, address/hostname logging, or
second hidden hostname connect. Native resolver caching is permitted; report cache state
unknown, not inferred. IPv6-only connectivity is outside the current approved hotspot
scope and must be explicitly refused/marked unsupported rather than silently accepted.

### TCP setup and TLS

For secure ports: setPlainStart(), setConnectionTimeout(5000), configure CA,
then connect(resolvedIPv4, port, ORIGINAL_HOSTNAME, CA, nullptr, nullptr). Time that as
`tcp_setup_ms`: includes socket establishment, CA parsing and TLS context setup, but no
DNS or TLS handshake. Then time startTLS() as `tls_ms` with the unchanged 5 s handshake timeout.
Keep hostname/SNI verification; never use setInsecure or connect by address without the
original name. No MQTT bytes may be written between TCP setup and successful startTLS.
Every retry must set plain-start anew; verify error cleanup leaves no stale TLS state.
For plain ports, TCP uses resolved IP and tls_ms is not applicable.

Use a worker-local Client facade between PubSubClient and the preconnected transport.
Its connect overloads fail if invoked: PubSubClient must not silently start another
unmeasured connect after a link drop between TLS and CONNECT. connected/read/write proxy
only from worker, and require TLS-ready for secure paths. Record unexpected_connect as
failure and cleanup. Capture TLS errors immediately within the failed phase; startTLS
failure may not update the same last_error field as connect, so do not call that fresh
unless source verification supports it. Phase failure is authoritative; TLS code may
remain unavailable/unknown freshness.

### MQTT/CONNACK and packet reads

Time PubSubClient connect against the preconnected facade as `mqtt_exchange_ms`, covering
CONNECT write and CONNACK receipt/parse, not pure server latency. Record result and state;
not-entered phases get an explicit validity bit, not a misleading zero duration.

Use a project-local, renamed PubSubClient 2.8 derivative with a MINIMAL reviewed patch:
absolute operation deadline/cancel predicate checked in connect's CONNACK wait,
readByte, readPacket (including oversize discard), and outgoing write boundaries;
vTaskDelay of at least one RTOS tick on EVERY continuing polling loop path, including
CONNACK wait, readByte, packet/discard loops and facade polling. yield() alone is forbidden.
Core 0 IDLE is watchdog-monitored; a priority-1 task must actually block to let it run.
The local patch ships in increment 1 with the worker, never in a later increment. No
intermediate build may run unpatched PubSubClient on that worker. Enforce remaining-length/packet bounds before
unbounded discard. For ONLINE partial packet receive, use an absolute 5 s packet budget
from first byte, not a fresh budget per byte. No packet or callback can bypass generation
cancellation. This patch is part of P001, with upstream version/license retained and an
explicit diff; do not patch the installed library or silently accept trickle-fed unlimited
waits. Worker transport writes still have underlying bounds/overrun reporting; deadline
checks cannot interrupt an already-running library call.

No reentrant runBackgroundTick from TLS/MQTT hooks. UI progress is obtained by ownership
separation, not by calling UI code from the network stack.

## 7. TLS memory and media/retrieval coordination

Existing MQTT secure transport is separate from the image/Live HTTPS transport. Steady
MQTT+HTTPS coexistence already exists. New work must not introduce concurrent MQTT TLS
establishment and media startup, nor two MQTT TLS contexts. Worker owns at most one.

A main-owned resource arbiter issues a generation-tagged MQTT establishment lease only
when imageFetcherIsBusy, videoStreamActive and pending-display/handover predicates are
clear. DNS runs WITHOUT this lease. After DNS succeeds and immediately before TCP
setup/CA parsing, request the lease through the main arbiter; worker cannot allocate TLS
until main grants the matching epoch. Acquire atomically against main media admission.
All Latest,
Back/history, MQTT-triggered still and Live paths check the same arbiter before side
effects. If media/retrieval already won admission, defer this attempt, discard its
resolved address, and retry after the normal backoff when eligible. No DNS result is
held indefinitely waiting for media; each retry resolves anew (native cache permitted).
Bound lease-response wait to 100 ms; on expiry invalidate the request and defer. A late
grant must be revoked by epoch/request-ID check without starting TCP or leaving a lease
held. DNS, lease wait and backoff do not exclude media. While lease held, refuse local media with a short "Reconnecting. Try again."
notice; suppress/deduplicate remote refused notifications by existing convention. No
unbounded deferred request. Release only after worker cleanup or READY; FAULT_HELD keeps
exclusion. Once ONLINE, existing media behavior resumes. Worker MQTT loop/publishes may
coexist with HTTPS as an established connection, but a loss cannot start a new handshake
until media ends. This temporarily refused-media behavior is part of JP's design approval.

Retrieval HTTP uses memory too. DNS may run while retrieval is
STARTING/ACTIVE/STOPPING, but do not grant the TCP/TLS establishment lease then; defer
that attempt and keep retry pending. Conversely refuse new retrieval entry with
mqtt_reconnecting while establishment owns the lease, before server allocation. This
is the shared entryRefusal(): BOTH panel entry and USB `log mode on` are refused. Panel
shows "Reconnecting. Try again."; USB reports reason=mqtt_reconnecting with the existing
response format. DNS alone refuses neither entry path. An
already established MQTT connection continues servicing messages during retrieval. This
changes recovery timing during download mode (maximum existing five-minute idle window),
not the existing WiFi recovery logic; make the deferral visible in status/logs. No abort
of a healthy download just to reconnect MQTT. JP must approve this explicit trade-off.

Memory allocation plan: 12 KiB worker stack + <=8 KiB bounded mailboxes in PSRAM; static
TCB, locks and <=2 KiB control/DNS metadata internal, measured sizeof and compile-time
caps. MQTT wire buffer remains 512 bytes internal; no second client/outbox/TLS stack.
Persistent footprint and TLS transient allocations must be measured, not inferred from
PSRAM labels. Admission requires current internal largest block >20480; this is only a
precheck, not a guarantee that handshake will stay above the gate. Sample internal free,
largest and DMA free/largest at phase boundaries and every 20 ms from main while worker
is active; include stack margin after TLS and CONNACK. Report minima with sampling scope.

Bench acceptance: observed internal largest >=20480 throughout retained cases, no
allocation failure, stack margin >=2048. If violated, do not lower the gate or disable
certificate checks: stop rollout and revise memory/ownership design. Tests include a
previous media cycle before reconnect to exercise retained heap fragmentation, and media
restoration after READY. Power loss during TLS must not create a replacement connection.

## 8. Shutdown, boot and state visibility

Main power-down hook atomically sets desired STOP and advances epoch, cancels media
admission, disables new publishes and drains no network work. It does not wait for
MQTT sockets/worker, add to diagnosticsClose's caller budget, or invoke client methods.
Worker closes when its bounded phase returns. Normal PMIC-off/deep-sleep proceeds under
existing policy even if it precedes cleanup; no attempt to free live task resources.
Snapshots mark stopping immediately. Worker completion cannot touch LVGL/NVS or revive
network activity after shutdown. Retained resources live for the boot only.

Expose snapshot: desired/current generation, state, connected, attempt ID, phase and age,
backoff remaining, last result, cancelled/stale counters, queue overflow, memory/stack
minima. Existing connected UI means ONLINE after subscriptions, not TCP established.
No status accessor probes socket state. An UNKNOWN worker condition renders offline,
not connected using stale data. Preserve authoritative driver WiFi events separately.

## 9. Bounded diagnostics and validation of the fix

Main assigns attempt IDs. Worker publishes a phase enum/time snapshot and one retained
final result with monotonic timestamps/durations. Main emits at most BEGIN, END, MEM and
SERVICE per attempt (plus existing bounded connection/subscription records). END includes
validity mask, dns_ms, tcp_setup_ms, tls_ms, mqtt_exchange_ms, total_ms, result,
failed_phase, epoch, cancelled, MQTT state and transport error freshness. Split records
if needed; host format checks must keep each field string <456 bytes. No credentials,
broker names, IPs from DNS, SSIDs, certificates or payload contents in these records.

While an attempt is pending, include phase/age in normal health/status. Record no per-poll
lines. Completion slot must be acknowledged by main before reuse; if main is busy, worker
waits with vTaskDelay of at least one tick and does not overwrite evidence. Queue/status counters report
any lost optional records. Main emits original worker timestamps with the correct stamp
rather than pretending delayed delivery happened at emission time; align both monotonic
and wall time, retaining quality if SNTP changes during a phase.

Existing diagnet Span and probe global counters stay main-only. Do not label worker
latency as a main-loop blocking span; keep legacy MQTT event names for analysis continuity
with new execution=worker context. Actual LOOP_GAP records remain measured on main.
Add per-attempt UI service interval and IMU service interval maxima (before/after actual
service calls, not sensor Hz estimates), main-loop max and counts over100ms. Reset these
at request and finish after result adoption; include cancel/failed attempts. A main
operation running concurrently is identified, so unrelated image latency is not hidden.

## 10. Implementation increments and focused review gates

No implementation yet. After the focused review and JP approval, use three increments:
1. A safe end-to-end worker path: ownership facade, fixed queues, event snapshots and
   lifecycle; migrate every direct MQTT access and callback/thread boundary. The local
   PubSubClient delay/deadline patch MUST ship here. Include all prerequisites needed
   to safely connect once: bounded DNS, split transport, generation cancellation,
   cleanup, media/retrieval admission and minimal phase/memory/stack result reporting.
   There is no intermediate unpatched worker or concurrent-client build. Host checks
   and Claude code review precede JP's first build/flash.
2. Complete the planned service-gap/field telemetry and failure/flap validation, plus
   any refinements from the first handshake. Increment 1 already has safety bounds;
   this increment must not be the first delivery of watchdog protection, cancellation,
   DNS lifetime or TLS admission. Re-review changes before JP builds/flashes.
3. Only corrections justified by retained bench evidence; then field rollout.

FIRST hardware check after increment 1: exactly one real-broker TLS handshake and
CONNACK on the PSRAM-stacked worker, before any case A or forced failure. Confirm
ONLINE/subscriptions, hostname-verified TLS success, responsive UI, worker stack margin,
internal-largest gate and absence of reset. This specifically validates mbedTLS/hardware
crypto from the worker's PSRAM stack; the HTTP worker proved lwIP, not this property.
Worker must never write NVS/flash (including Preferences/calibration); those stay on main.
If handshake crashes or fails this placement check, stop and revise the stack design for
review; do not automatically switch to an internal stack or continue other cases.

Required host checks (real implementation logic, not merely regex where practicable):
- every worker-reachable polling loop and continuing wait path includes vTaskDelay
  (>=1 tick), not yield(): audit actual local PubSub patch plus facade/worker loops,
  and run delayed-CONNACK/partial-packet simulations proving tick-delay calls and
  deadlines. A static guard rejects bare-yield waits or newly unguarded polling loops;
- zero direct client access outside owner; no callbacks/UI on worker; publish admission
  versus completion; RX/TX overflow, size bounds, stale-generation discard;
- same-IP link flap, config/bench switch during each phase, cancelled-success suppression,
  retained DNS tombstones beyond 21 s, late completion release, slots exhausted,
  tcpip submission ERR_MEM and resolver ERR_MEM without false failure-budget debit;
- DNS fail, TCP fail, TLS fail, CONNACK fail/trickle/oversize and absolute deadlines;
  hostname/SNI preserved, no plaintext MQTT before TLS, no implicit reconnect; pin
  DNS wait=15000, attempt=35000, stuck=40000 and lifetime connection timeout=5000;
- bidirectional media/retrieval lease admission only after DNS, late grant rollback,
  both panel and USB refusal, pending handover, memory refusal;
- shutdown returns without waiting; held worker fault and late recovery do not free
  live resources; failure budget/backoff reset after completion, setup asynchronous;
- phase timestamp validity, bounded record sizes, main service metrics, no secret fields.
All existing relevant host suites remain required; adapt locations/APIs without weakening
behavioral assertions, and explicitly revise tests for approved admission semantics.

Retained hardware cases, issued ONE AT A TIME after review/build clearance:
A. Successful reconnect after a prior Latest/Live cycle, watching spinner/G-meter/touch;
   phases and service gaps, memory/stack, subscriptions and subsequent image verified.
B. Failed broker connection while WiFi stays associated, two attempts then recovery;
   verify backoff, phase failure, main responsiveness, stack/memory, no log drops. A
   documentation-address fast rejection is insufficient to claim a long-wait bound;
   if it rejects quickly, use a controlled endpoint that delays/drops the intended phase.
C. One controlled hotspot off/on flap while an attempt is pending, including same-IP
   reconnection; prove stale completion cannot resurrect state and next attempt recovers.
Include one media/retrieval refusal observation during the relevant case, not a separate
large matrix. Rare packet/DNS races use host tests. Power/cancel paths require review;
add hardware only if checks or observed behavior expose uncertainty. This design does
not claim existing power tests validate new worker ownership.
Then one normal car ride with a saved full export and comparison against F001/F002.
If no outage occurs, it is normal-use evidence, not proof of reconnect responsiveness.

## 11. Claude review and JP decisions

Claude: review one design, identify blockers against installed source, especially the
split secure API, bounded local PubSub patch, DNS context lifetime, TLS concurrency,
mailbox thread safety and media admission coverage. Append a dated review below or link
a review document; Codex integrates revisions before JP approval.

### Decisions still requiring JP approval

JP has directed the revision-2 corrections; the following wider choices are still
recommendations, not implementation authorization:

1. **Worker and memory:** persistent 12288-byte PSRAM stack, internal static TCB, core 0
   priority 1, exclusive MQTT ownership and bounded queues. Failure stays offline;
   no automatic internal-stack fallback. Recommend approve, subject to first-handshake gate.
2. **Temporary admission behavior:** after DNS, an active establishment lease refuses
   Latest/Back/Live and BOTH panel/USB retrieval entry with a retry notice. If media or
   retrieval is already active, MQTT waits instead; no forced abort. Recommend approve.
3. **Initial failure policy:** retain five genuine failures before first successful MQTT
   connection, then give up until reboot; once connected, retry indefinitely with 15 s
   backoff. Recommend preserve for P001 rather than combine a policy redesign.
4. **Acceptance and rollout:** <=100 ms UI/IMU service-gap target in reconnect-only cases,
   >=20480 internal-largest gate, >=2048 worker-stack margin, first single handshake then
   cases A/B/C one at a time. Recommend approve the focused sequence, not an expanded suite.

DNS=15 s, attempt=35 s and stuck=40 s are revision-2 proposed concrete values implementing
JP's direction to raise the bounds. Phase timing is mandatory; no instrumentation-only
flash. No WiFi-loss cure is promised. JP's approval of revision 2 must explicitly authorize
increment 1 implementation; approving this documentation edit does not do so.

### Revision 2 integration and review status

Codex integrated review 8daed34 at JP's request: B1 tick-delay patch and host check move
to increment 1; B2 DNS is 15 s, attempt 35 s, stuck 40 s; constant socket timeout 5000;
lease only before TCP; explicit DNS lifetime/ERR_MEM paths; no 3.1.3 capability branch;
first hardware check is worker TLS/CONNACK; panel and USB refusal both specified.
The historical revision-1 review below is preserved unchanged. Its quoted old values
are not revision-2 requirements. Claude requested only a focused B1/B2 integration check,
not another full architecture review. JP implementation approval remains pending.

### Claude review - September 25, 2026 (revision 1, commit 16cda62)

**Verdict.** The architecture is right: one worker that owns every MQTT operation, retained
PubSubClient behind a minimal local patch, split secure connect, and main-side snapshots
and queues. Reject a connect-only handoff and esp-mqtt for now, as proposed. Two
blockers (B1, B2) must be fixed in revision 2; the rest are trade-offs for JP or
clarifications.

**Verified against installed sources (3.3.11 unless stated):**
- `setPlainStart()`, `startTLS()` and `connect(IPAddress, port, host, CA, cert, key)` exist
  (NetworkClientSecure.h:59, 89, 95). **3.1.3 has the same three** (same header lines in
  its package), so the legacy capability branch in section 3 is unnecessary.
- connect() skips the handshake in plain-start and sets `last_error`; startTLS() failure
  calls `stop()` and does NOT set `last_error` (NetworkClientSecure.cpp:147-181). Codex's
  "unknown freshness" rule for TLS errors after startTLS is correct.
- TCP connect is nonblocking with `select(timeout)` (ssl_client.cpp:91, 125-143). The
  handshake loop yields with `vTaskDelay(2)` and honours `handshake_timeout` (:331-338).
  The write loop yields too (:463).
- PubSubClient reuses an already connected transport (:186-188); -2 means transport
  connect failed and -4 means no CONNACK (:258-280), as stated.
- `CONFIG_MBEDTLS_INTERNAL_MEM_ALLOC=y`: TLS buffers come from internal heap whatever
  the worker stack location, so section 7's memory gate is the right concern.
- `tcpip_try_callback`, `dns_gethostbyname_addrtype`, `DNS_TABLE_SIZE 4`,
  `CONFIG_LWIP_TCPIP_CORE_LOCKING=y`, tcpip thread pinned to core 0.
- Direct MQTT call sites: companion.ino (9), net_module.cpp (5), imu_module.cpp (IMU
  telemetry :304-305, motion :498-499). image_fetcher.cpp mentions MQTT only in a comment.
  Section 4's migration list covers all of them.

**B1 - watchdog reset on core 0 (blocker).** The CONNACK wait
`while (!_client->available())` (PubSubClient.cpp:257-265) never yields, and `readByte()`
(:290-296) only calls `yield()`, which never lets the priority-0 IDLE task run. The
installed config has `CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0=y`, `TIMEOUT_S=5` and
`PANIC=y`. A priority-1 worker on core 0 waiting 5 s for a CONNACK that never comes would
starve IDLE0 for about 5 s and reset the board. Today the same spin runs on core 1, where
IDLE is not watched, which is why it only freezes the UI.
- Required: every PubSubClient polling loop in the local patch uses `vTaskDelay(>=1 tick)`,
  not `yield()`.
- The patch ships in **increment 1** with the worker. No build may run unpatched
  PubSubClient on the worker, even briefly. Add a host check that no worker-reachable
  loop polls without a tick delay.

**B2 - the 5 s DNS deadline would turn today's successes into failures (blocker).** lwIP
2.x retries on a 1 s timer (`DNS_TMR_INTERVAL 1000`, `DNS_MAX_RETRIES 4`). Queries go out
at about 0, 1, 2 and 4 s, and a server is abandoned at about 7 s before the next server
is tried. F001's two reconnects took 7855 and 7893 ms. That fits "the first DNS server
stays silent for about 7 s, then an answer arrives, then about 0.9 s of TCP/TLS/MQTT"
(hypothesis; the new phase timing will test it). A 5 s DNS cap would abandon exactly
those attempts and add a 15 s backoff each time, lengthening recovery from about 8 s to
about 21 s, against section 2's own rule that successful recovery must be preserved.
With the worker, phase deadlines bound resources, not UI responsiveness, so they must
not be shorter than the native resolver schedule.
- Recommended: DNS wait of at least 8 s (or lwIP's own completion, capped around 15 s).
  Raise the attempt budget and the 25 s `worker_stuck` threshold accordingly.
- Keep TCP and TLS at 5 s initially. F002's 5004 ms failure cannot show whether a longer
  wait would have succeeded; the phase timing will.

**Minor correction.** `setConnectionTimeout()` is not only the TCP connect limit. It
becomes the connection's lifetime `socket_timeout`, `SO_RCVTIMEO` / `SO_SNDTIMEO` and the
write-progress timeout (ssl_client.cpp:116, 171-172, 453). Use a constant 5000 ms, not
"remaining <= 5000", so a late-phase remainder can never shorten ONLINE write timeouts.

**Trade-offs and clarifications:**
- **Lease timing (JP decision).** Acquire the media/retrieval lease just before TCP setup
  (CA parsing and TLS allocation), not before DNS. DNS and link waits use no TLS memory.
  Acquiring earlier, combined with B2's longer DNS wait, would lengthen "Reconnecting. Try
  again." refusals needlessly. On a flapping link (F002: three losses in 70 s) these
  refusals can recur. They are acceptable only if short, and media would usually fail on
  such a link anyway.
- **DNS lifetime.** lwIP copies the hostname into its table at enqueue, so the hostname
  buffer only needs to survive submission. The callback argument must survive until the
  callback, which lwIP always makes, with NULL on failure, after at most about 7 s per
  configured server (up to 3 here, so about 21 s). With the 15 s backoff, two slots are
  enough; state that bound in the design.
- **DNS submission failure path.** `tcpip_try_callback` returns `ERR_MEM` when the tcpip
  mailbox is full. That path must free the slot and defer without counting a failure.
  Alternative: with core locking enabled, call `dns_gethostbyname_addrtype` under
  `LOCK_TCPIP_CORE()` from the worker, which removes that failure mode. Either is
  acceptable.
- **TLS on a PSRAM stack is unproven.** The HTTP worker proves lwIP, not an mbedTLS
  handshake with hardware crypto on a PSRAM stack. The design forbids an automatic
  internal-stack fallback, so make "one handshake and CONNACK on the worker" the first
  hardware observation after increment 1, before case A. The worker must never write NVS
  or flash (Preferences, calibration saves stay on main).
- **Retrieval refusal applies to USB entry too.** Adding `mqtt_reconnecting` to the shared
  `entryRefusal()` refuses USB entry as well. That is acceptable but should be stated,
  with a readable panel message added.

Once B1 and B2 are integrated, the ownership, cancellation, shutdown and diagnostics
sections are ready for JP's decision. No further review round is needed beyond checking
those two changes.

### Claude focused integration check - September 25, 2026 (revision 2, commit 3351cde)

**Verdict: B1 and B2 are fully integrated; no remaining blockers.** Revision 2 is ready for
JP's increment 1 implementation approval. Checked only the revision-1 findings and their
knock-on rules, not a new architecture review.

- **B1 resolved.** vTaskDelay (>=1 tick) is required on every continuing wait: worker turns,
  CONNACK wait, readByte, packet/discard loops, facade polling, DNS polling and the
  completion-slot wait. `yield()` alone is forbidden. The patch ships in increment 1 with
  no intermediate unpatched build, and the host checks add both simulations and a static
  bare-yield guard. No stale "yield" wording remains in the design body.
- **B2 resolved.** DNS 15 s (above lwIP's ~7 s per-server give-up), attempt 35 s
  (15 + 5 + 5 + 5 + 2 + 3 allowance, arithmetic consistent) and stuck 40 s share one
  dispatch clock that never resets. The per-phase admission rule holds: after the maximum
  15 s DNS, 20 s remain against the 17 s TCP/TLS/MQTT/subscription allowance. No stale
  20 s, 25 s or 5 s DNS values remain outside the preserved revision-1 review.
- **Constant 5000 ms connection timeout** is stated on both transports, with the reason.
- **Sequencing and lifetime are consistent.** The lease is requested only after DNS and
  before CA parsing. Denial, the 100 ms wait expiry and late grants all roll back by
  epoch/request ID without starting TCP, and none of them counts toward the initial
  failure budget. A fault before the lease reserves nothing. DNS tombstones hold no lease.
  Two slots cover the ~21 s callback bound against the 15 s wait plus 15 s backoff.
  Callback acknowledgement stays the only release authority. Retrieval allows DNS but
  withholds the lease, and entry refusal (panel and USB) applies only while the lease is
  held.

Non-blocking notes for implementation and field reading:
- The worst-case media/retrieval refusal per attempt is now about 17 s (TCP to READY),
  down from about 20 s in revision 1, and none of it during DNS.
- With three DNS servers, a 15 s cut falls inside the third server's window. The hotspot
  normally supplies one server, so this only matters if field timing shows otherwise.
- The 100 ms lease wait can expire during an unrelated main-loop stall. That is a cheap,
  uncounted deferral, but count it in status so field logs can distinguish it from media
  contention.
