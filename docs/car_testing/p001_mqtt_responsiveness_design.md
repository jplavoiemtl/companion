# P001 - Responsive UI and IMU during MQTT recovery

Revision 1, September 25, 2026. Author: Codex. Status: DESIGN FOR CLAUDE REVIEW AND
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
contention and watchdog behavior. Yield on every polling turn. The already validated
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
Primary target remains the accepted 3.3.11 profile. Before merging code, verify 3.1.3
has the same split API; if not, keep the legacy profile compiling behind an explicit
capability branch using worker-owned combined transport timing, marked combined rather
than fabricated separate fields. No silent change to its certificate validation.

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
it reached a deadline. After 25 s without attempt completion, report one worker_stuck
notice and counter, remain disconnected/FAULT_HELD, retain buffers/task and resource lease,
refuse new connects/media that need that lease. Late cleanup clears the fault explicitly;
otherwise restart is the recovery path. Main/UI/IMU continue. No unsafe forced deletion.

## 6. Phase timing and bounded transport behavior

Attempt wall budget: 20 s nominal, with DNS 5 s, TCP socket connect 5 s, TLS handshake
5 s, MQTT exchange absolute 5 s, each limited by remaining attempt time. Subscription
setup is a separate bounded 2 s budget before READY. Allocation/CA parsing and underlying
library cleanup are not hard-real-time bounded; record actual phase overrun and use
FAULT_HELD policy. Do not advertise these as guaranteed resource-release deadlines.

### DNS

Use two boot-retained request slots containing hostname storage (max 253 chars plus NUL),
attempt ID, epoch, callback result and state. Submit with tcpip_try_callback to TCP/IP
thread; perform dns_gethostbyname_addrtype there, IPv4 for the currently IPv4 STA/hotspot
scope. Literal configured addresses bypass DNS and report skipped_literal. Cached success,
asynchronous completion and immediate error all have explicit paths. Poll/notify worker
in <=20 ms intervals, enforcing a 5 s wait even if resolver still owns the request.

A timed-out/cancelled slot is a tombstone until the queued submission and any DNS callback
finish. Never reuse/free its hostname or callback context first. Completion releases its
own matching slot even after invalidation, but cannot advance a stale attempt. If both
slots are retained, return dns_slots_busy and defer; do not allocate a third or globally
reset DNS. No application DNS cache, prewarming lookup, address/hostname logging, or
second hidden hostname connect. Native resolver caching is permitted; report cache state
unknown, not inferred. IPv6-only connectivity is outside the current approved hotspot
scope and must be explicitly refused/marked unsupported rather than silently accepted.

### TCP setup and TLS

For secure ports: setPlainStart(), setConnectionTimeout(remaining <=5000), configure CA,
then connect(resolvedIPv4, port, ORIGINAL_HOSTNAME, CA, nullptr, nullptr). Time that as
`tcp_setup_ms`: includes socket establishment, CA parsing and TLS context setup, but no
DNS or TLS handshake. Then time startTLS() as `tls_ms` with <=5 s handshake timeout.
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
cooperative delay/yield in polling loops. Enforce remaining-length/packet bounds before
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
clear. Acquire atomically against main media admission, before DNS starts. All Latest,
Back/history, MQTT-triggered still and Live paths check the same arbiter before side
effects. While lease held, refuse local media with a short "Reconnecting. Try again."
notice; suppress/deduplicate remote refused notifications by existing convention. No
unbounded deferred request. Release only after worker cleanup or READY; FAULT_HELD keeps
exclusion. Once ONLINE, existing media behavior resumes. Worker MQTT loop/publishes may
coexist with HTTPS as an established connection, but a loss cannot start a new handshake
until media ends. This temporarily refused-media behavior is part of JP's design approval.

Retrieval HTTP uses memory too. Do not start a new MQTT establishment while retrieval is
STARTING/ACTIVE/STOPPING; keep retry pending. Conversely refuse new retrieval entry with
mqtt_reconnecting while establishment owns the lease, before server allocation. An
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
stays in a yielding state and does not overwrite evidence. Queue/status counters report
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

No implementation yet. After Claude review and JP approval, use three reviewable increments:
1. Ownership facade, fixed queues, event snapshots, worker state and lifecycle; migrate
   every direct MQTT access and preserve callback/thread boundaries. Host review before
   any flash. Do not ship a temporary concurrent-client implementation.
2. Bounded resolver, split TLS phases, minimal local PubSub patch, media/retrieval lease,
   cancellation and telemetry. Host tests and Claude review before JP build/flash.
3. Only corrections justified by the retained bench evidence; then field rollout.

Required host checks (real implementation logic, not merely regex where practicable):
- zero direct client access outside owner; no callbacks/UI on worker; publish admission
  versus completion; RX/TX overflow, size bounds, stale-generation discard;
- same-IP link flap, config/bench switch during each phase, cancelled-success suppression,
  retained DNS tombstones, late completion release and slots exhausted;
- DNS fail, TCP fail, TLS fail, CONNACK fail/trickle/oversize and absolute deadlines;
  hostname/SNI preserved, no plaintext MQTT before TLS, no implicit reconnect;
- bidirectional media/retrieval lease admission, pending handover, memory refusal;
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

JP approval must cover the persistent PSRAM worker/core placement, explicit media refusal
while reconnecting, MQTT recovery deferral during retrieval, and retained initial give-up
policy. These are recommended choices, not already approved implementation. Phase timing
is explicitly requested by JP and is mandatory; no separate instrumentation-only flash
is proposed. No WiFi-loss cure is promised by P001.

### Review status

Claude review pending. JP implementation approval pending.
