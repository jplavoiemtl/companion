# P005/P006 combined increment - code review handoff

September 25, 2026. Author: Codex. Branch: codex/car-improvements-p001.
Base: 2d93c24, Claude's focused design clearance. JP approved design revision 2 and
explicitly authorized this implementation, host checks, commit and push. No build or
flash performed. Awaiting Claude's code review before JP builds or runs a bench case.

Contract: [approved design revision 2](p005_p006_recovery_design.md).
Field rationale: [F003 and review integration](field_journal.md).

## Implemented behavior

- **P005 A:** net_module.cpp stamps a main-owned monotonic ONLINE origin only after
  real READY adoption and connected confirmation. Loss earns one prompt retry only
  in normal bench-idle mode after >=60000 ms. Loss consumes the origin; short sessions
  cannot accumulate credit. Failure/cancellation keeps 15 s, and explicit bench restore
  and first-test delay retain exclusive precedence. Existing shutdown/request gates
  still prevent dispatch. No new timer, task, lease bypass or GOT_IP trigger.
- **P005 B:** net_worker.h names TLS/MQTT ten-second and subscription two-second
  allowances, attempt 45 s and stuck 50 s. net_worker.cpp uses ConnectTimeoutScope
  inside connectExchange: constructor selects PubSub 10 s, destructor restores 5 s
  before the caller handles subscriptions, READY or failure. Both plaintext and TLS
  MQTT use it. Native TLS handshake is 10 s; both setConnectionTimeout(5000) calls
  remain literal and unchanged. ONLINE operations remain 5 s. No PubSub library edits.
- **P006:** companion.ino uses joinedMqttNetwork and configureMqttFromJoinedWifi in
  initial and late paths. Primary/secondary roles return their configured network
  numbers; primary wins equal names, including priority 2. Empty/unknown/disconnected
  snapshots defer without configuring 0. One successful-configuration flag prevents
  repeated invalidation. Deferred initial selection can retry through the late path
  even if WiFi was already up at boot. MQTT_PROFILE_SELECT records numeric selection
  and source only, coalescing repeated outcomes. No new credential/SSID logging.

Existing ownership, memory admission (20480 bytes), stack placement, DNS slots,
packet discard, media/retrieval guards and phase telemetry are retained. P004 is not
implemented. Automatic broker-profile changes after later roaming remain out of scope.

## Review focus

1. Check >=60 s timing at the two main adoption sites; observedConnected is the validity
   flag, so adoption at timestamp zero works. Verify loss/intentional disconnect clearing,
   bench precedence, and that a cancelled attempt cannot acquire prompt eligibility.
2. Check RAII scope exits before every subscription/READY/result path, including failed
   or cancelled CONNECT. The absolute MQTT operation deadline still prevents a trickled
   CONNACK from extending the total wait. Native deadlines remain cooperative, not
   promises of hard wall-clock completion.
3. Check actual network-number selection, WIFI_PRIORITY=2, equal names, unknown/deferred
   recovery, and the initial/late flag. The SSID snapshot and link recheck are not an
   atomic association guarantee; existing owner epochs still invalidate link changes.
4. Check tests changed only for approved policy differences. Endpoint credentials,
   WiFi scan/retry policy and media transport calls retain existing baseline assertions.

## Host validation

All **379 checks in 16 suites pass**. Full suite sweep passed with 378, followed by the
new combined link-event/policy check and rerun of the recovery suite (22 checks).
These are JavaScript source adaptations and source-contract assertions, not C++/ESP32
compilation, ABI checks or physical network timing measurements. git diff --check passes.

| Suite (tools/tests/) | Checks |
| --- | ---: |
| http_lifecycle.test.cjs | 43 |
| http_transfer.test.cjs | 35 |
| log_time.test.cjs | 20 |
| media_admission.test.cjs | 16 |
| mqtt_owner.test.cjs | 53 |
| mqtt_recovery.test.cjs (new) | 22 |
| mqtt_service.test.cjs | 14 |
| network_diagnostics.test.cjs | 16 |
| operation_diagnostics.test.cjs | 8 |
| reader_session.test.cjs | 28 |
| retrieval_mode.test.cjs | 42 |
| retrieval_ui.test.cjs | 15 |
| sd_log_browser.test.cjs | 20 |
| touch_contact.test.cjs | 19 |
| usb_connection_guard.test.cjs | 16 |
| usb_logger_gate.test.cjs | 12 |

New recovery checks execute the modified loss block and READY stamp, result-backoff
statements, real link-event/invalidation bodies, admission expression, connect-timeout
constructor/destructor and selection functions with deterministic mocks. C++ RAII is
represented as try/finally in the JS adapter; call-site ordering is separately asserted.
The owner suite additionally executes real PubSub wait/readPacket/readByte bodies with
CONNACK beyond 5 s, fragmented success at 9 s, timeout at 10 s and early cancellation.
Existing five-second ONLINE/oversized-discard checks stay unchanged.

Commands: run each tools/tests/*.test.cjs with node (PowerShell Get-ChildItem loop),
checking each process exit code. No Arduino compiler, build script or flash tool invoked.

## Hardware handoff after clearance

Do not issue a bench case before Claude clears this implementation. JP then builds
amoled-1-8-core-3-3-11. The selected profile's stale generated companion.ino.cpp is
removed if present because the sketch changed; no other build artifacts are touched.

Only the two approved essential cases remain: stable real connection followed by normal
hotspot recovery, then pending-attempt flap/failed retry with the retained serial fixture.
Issue one at a time. The first needs >=60 s ONLINE (65 s preparation); a slow IP return
may prove recovery without discriminating prompt eligibility. No ten-second successful
TLS handshake or secondary-SSID race is forced on hardware. Benefit on slow real links
remains a field question; longer failures may instead hold the lease longer.

Capture the existing status, END/MEM/SERVICE and WiFi/MQTT timeline through the usual
log export. Retain UI/IMU <=100 ms, stack >=2048 bytes and internal-largest >=20480
bench gates, no new resets/drops/errors. After review, build and accepted results, P004
image responsiveness is the next separate design; it is not authorized by this increment.

## Claude code review - September 25, 2026 (734ad92 against 2d93c24)

**Verdict: cleared for JP's build and the first approved bench case.** No blockers.
Host checks re-run: **379 pass in 16 suites**. Not compiled, per the workflow.

- **P005 A.** `onlineAdoptedAtMs` is stamped only in the real READY branch, after
  `acknowledgeReady` and `netIsMqttConnected()` (net_module.cpp:289-292), and cleared on
  loss and on `intentionalDisconnect`. Prompt eligibility requires Idle bench, a valid
  `observedConnected` and >= 60000 ms between the two main adoptions. The precedence is
  now an exclusive chain (restore, then first test attempt, then prompt), so no double
  subtraction is possible. Cancelled, revoked or test attempts never set
  `observedConnected`, so they cannot earn a prompt retry. With the link down, a backdated
  stamp only makes `request()` refuse until the link returns, with no side effects.
- **P005 B.** `ConnectTimeoutScope` sets 10 s around `connect()` only, and its destructor
  restores 5 s when `connectExchange` returns, before subscriptions, READY or failure
  handling. That covers every exit, including the operation guard failing. The MQTT phase
  allowance (`MQTT_MS`) sets the absolute `operationDeadline`, which still bounds a
  trickled CONNACK. `setHandshakeTimeout(TLS_MS/1000)` = 10. Both
  `setConnectionTimeout(5000)` literals are unchanged, and the TCP allowance uses
  `SOCKET_MS` = 5000. The ONLINE deadlines use `SOCKET_MS`. The constants 45/50/10/10/2
  match the design, and the budget test moved to the 40000/45000 boundary.
- **P006.** `joinedMqttNetwork` returns `primaryNetworkNum` or `secondaryNetworkNum`, with
  primary first, and never a role index. It returns 0 for empty or unknown names, and a
  status recheck after the SSID read defers a disconnected snapshot. Both call sites use
  the helper. The initial path only dispatches when configured. The late path now runs
  whenever the profile is unconfigured and WiFi is connected (no longer only after a
  failed boot), which covers deferred initial selection. Outcome events are coalesced and
  carry only numbers.
- **Tests.** The only existing assertions changed are the timing constants and boundaries
  and the `setSocketTimeout`/`setHandshakeTimeout` call lists. New cases cover a delayed
  or fragmented CONNACK inside 10 s, a timeout at 10 s, and early cancellation.
  Credential, endpoint and media assertions are untouched.

Nonblocking:
- **Profile set once per boot.** `g_mqttConfiguredFromWifi` is never reset, so the
  profile stays fixed for the boot after the first successful selection. That matches the
  design's "no automatic re-selection after roaming", and the existing
  `MQTT_WIFI_PROFILE mismatch` telemetry would expose a later change.
- **Deferred-selection cost.** While selection is deferred on an unknown SSID, the late
  path reads `WiFi.SSID()` (a String allocation) every loop turn. That is cheap, bounded
  by the event coalescing, and only occurs in that uncommon state.


## P005/P006 retained case 1 - September 25, 2026, Codex

Result: reconnect timing, responsiveness, profile and resource gates pass. JP reports
G-meter stayed responsive. Boot 136, build stamp Sep 25 2026 22:37:00; no exact Git
identity embedded. Reviewed code checkpoint is 271f94b (implementation 734ad92).
Evidence preserved in evidence/2026-09-25-p005-p006-case1/: 136-current.log and console.txt.
Log: 225030 bytes, CRC32 4895D185 (console CRC OK), SHA256
564b20bca79e0040ce41ad6fe0c7fe63e2e8194d319ad6c2e7fd01bcdfdc1d1d. Raw evidence remains ignored.

Use same-boot uptime: startup clock is approximate and later synchronized, so wall-clock
subtraction across startup is inappropriate. READY/CONNECTED up_ms=9181, loss=105900:
96.719 s ONLINE, satisfying the >=60 s rule. GOT_IP=116861, BEGIN=116872: **11 ms**.
BEGIN occurs 10.972 s after loss, proving it did not wait the former 15 s. Approximately
4.028 s of the former scheduling wait were avoided in this case. CONNECTED=117611:
750 ms after GOT_IP, total MQTT outage 11.711 s. Worker reports total_ms=730, DNS=2,
TCP=126, TLS=508, MQTT exchange=77 ms, result=ok. BEGIN/END stamps and worker total
use slightly different capture points; preserve the reported total rather than equating them.

Reconnect SERVICE: window=741 ms, UI/IMU/loop gaps=19/20/20 ms, all over100 counts=0.
MQTT stack margin=7228 bytes, internal/DMA largest minima=47092, lease=0 after success,
no dropped messages, lease timeouts, cancellations or stuck faults. Logger queue high=7,
drops=0, truncated=0, error=none; writer margin=3128. P006 startup selection requested=1,
actual=1, mismatch=0; both MQTT attempts use connection=wifi_connection=1. This does
not hardware-exercise reversed priority or unknown-SSID selection; host coverage stands.
Healthy TLS/CONNACK complete well below 5 s, so benefit from the longer allowance
remains unmeasured, as planned. No reset occurs within the captured case.

Boot 136 itself records reset=task_watchdog before the test. JP confirmed this happened
during the serial-monitor switch, the known pre-existing limitation. It is not a reset
during this recovery case. Startup SERVICE loop_gap=687 with loop_n=0 is the setup window,
not the reconnect window; UI/IMU there are 27/32 ms.

Next retained gate: case 2, pending-attempt hotspot flap with failed/cancelled cleanup
and 15-second retry spacing, then explicit on restoring the real broker. No rebuild
required. Overall P005/P006 acceptance remains pending case 2 and JP's acceptance.
