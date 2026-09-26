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
