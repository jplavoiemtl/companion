# Stage 2 network evidence - first bench handoff

## Commit handoff authorized by JP

JP requested committing and pushing this completed implementation/evidence batch
to make diffs easier to follow. Stage 2 is not accepted: the RSSI/suppression
correction still needs JP's rebuild/flash and hotspot repeat. Earlier references
to remaining uncommitted describe the state before this handoff. No firmware
build or flash was performed by Codex.


JP explicitly accepted Stage 1B and requested commit/push, then Stage 2.
The acceptance/evidence checkpoint was committed and pushed as `f899e3c` on
`sd-diagnostics`. This Stage 2 implementation is local and uncommitted, pending
remaining hardware validation. JP's compiled/flashed boot 84 passes startup-record, health and download inspection. JP confirms the prior watchdog followed closing the VS Code monitor, the known limitation. The controlled MQTT off/on case also passes on boot 84 (22:10-22:11); hotspot recovery plus Latest also works. Two diagnostic fixes (RSSI validity and alternating-reason suppression) await rebuild and repeat of that case; see the checkpoint. Stages 3-4 have not started.

## Prepared implementation

- `diagnostics_network` uses the existing bounded event queue and writer. No new
  task, queue, filesystem owner, network request, endpoint or reconnect policy.
- Wi-Fi task callbacks copy association, scan completion, GOT_IP/change,
  LOST_IP and disconnection evidence. Records have `source=driver`; their normal
  `up_ms` stamp is callback-observation time, not a later UI-poll timestamp or
  a claim of the radio's exact physical-event time. Only profile aliases and
  numeric connection IDs are stored, never SSID/password/BSSID.
- Disconnect records keep numeric reason, a label for common values (`other`
  otherwise), and current-disconnect or last-valid RSSI. Unknown is explicitly
  marked; raw -128 and nonnegative values are rejected. Four bounded reason/profile
  slots suppress repetitions within five seconds even when reasons alternate;
  the next event and cumulative health counter report suppression. An association
  resets this guard so a fresh loss after recovery is always recorded.
- Setup, scans, association waits, HTTPS requests and new Live connections have
  paired `NET_BEGIN`/`NET_END` records. Every MQTT connect, real or test, gets
  `MQTT_CONNECT_BEGIN`/`MQTT_CONNECT_END` with a per-boot 64-bit operation ID,
  configured connection, associated Wi-Fi connection, target alias, port/TLS,
  duration, result and actual PubSubClient state. IDs are shared across spans;
  gaps in MQTT attempt IDs are expected. No per-frame success records.
- TLS errors are captured before application `disconnect`/`end`/client reuse,
  only on secure failures. `lastError()` returns a numeric code and bounded
  mbedTLS description, sanitized for a quoted field. `tls_fresh=unknown` is
  deliberate: DNS failures can leave a stale value. A zero code is not proof
  of no TLS problem. Library-internal close may already have happened before
  control returns; these hooks do not instrument inside the SDK.
- Span timing means the actual blocking API boundary: MQTT includes DNS/TCP/TLS/
  CONNACK, image GET includes response headers, Live connect includes DNS/TCP/TLS.
  These are not isolated handshake measurements. Main breadcrumbs preserve and
  restore enclosing phase/operation; new phase IDs append after old IDs.
- First observed MQTT loss is recorded before application cleanup, retaining
  state such as -3 or -4 as actually observed. Records distinguish requested
  bench disconnects, initial/recovered connection, budget exhaustion and media
  retry deferral/release. Release is observed before the next connect attempt.
  Wi-Fi/MQTT profile mismatch is observational; it never reconfigures a broker.
- `off`, `on`, `status` requests and off/on outcomes are recorded using bounded
  aliases. Production credentials still never go to the test endpoint.
- Subscriptions and motion/calibration publishes record the actual library
  return value as `accepted`, with `ack=unobserved`. Image notifications record
  accepted or ignored-live/echo/screen/payload decisions where made. Repeated
  identical ignored notifications are bounded, with suppression counts.
- Main-task callback counters aggregate inbound/power/energy messages without
  payloads or topic strings. The writer appends `NET_HEALTH` beside each HEALTH,
  using the same locked main-task snapshot, cumulative counts and last inbound
  age (-1 until known). Inbound means application callbacks, not TCP/PINGRESP.
  Snapshot age is provided; the writer makes no Wi-Fi or MQTT calls.
- Queue/retention/USB limits, writer placement/priority, all network timeouts,
  retries, endpoint selection, TLS hostname verification and media guards remain.
  Conditional TLS-aware writer deferral was not added without measurements.
  Stage 0 probes remain; firmware tag is `stage2-network`.

## Validation and limits

Passed 16 network checks (source contracts and translated policy replay) against accepted `f899e3c`, plus
19 existing browser protocol checks, 16 connection/pacing source simulations,
and 12 queue/pruning source simulations: 63 host checks total (47 existing checks passed before the diagnostic correction). Run:

```text
node tools/tests/network_diagnostics.test.cjs
node tools/tests/sd_log_browser.test.cjs
node tools/tests/usb_connection_guard.test.cjs
node tools/tests/usb_logger_gate.test.cjs
```

Source-contract checks inspect code/order and compare selected network calls;
they are not C++ compilation or executable tests of the firmware. Reviewed
installed 3.3.11 event structures, NetworkClientSecure error behavior and
PubSubClient 2.8 state/cleanup behavior. No firmware build or flash by Codex.
Callback registration and added records have real memory/CPU cost; the unchanged
20480-byte gate, zero-drop requirement and paired performance checks still apply.

## First case only: normal startup and log inspection

1. Keep the card installed and hotspot on. Build/flash in VS Code using
   `amoled-1-8-core-3-3-11`. Flags are already enabled=1, hooks=0, fixture=0,
   PSRAM writer=1. The assistant removed
   `build/build_amoled-1-8-core-3-3-11/sketch/companion.ino.cpp` before handoff.
   If compilation fails, stop and send the first error with surrounding lines.
2. Connect the web console with DTR=true, RTS=false, all test switches off.
   Wait for normal Wi-Fi/MQTT connection, then send `status` once. Expect logger
   ready, valid PSRAM placement and zero drops/errors. Retain the full console.
3. Stay on the dashboard for about 70 seconds, allowing one new HEALTH and
   NET_HEALTH interval. Keep normal hotspot/USB connection; no media or test
   commands in this initial case.
4. Download current.log once and send `status` again after completion. Expect
   CRC OK, ready, active=0, paused=0 and zero drops/errors. Send the console,
   saved file path (or attach the log), and any visual/startup anomalies.

Inspect the new boot's `build=stage2-network`, Wi-Fi driver/setup records,
MQTT attempt BEGIN/END, configuration/subscription/publish observations,
SETUP_COMPLETE and NET_HEALTH counters. Unknown inbound age is valid if no
application message has arrived. Old boots in current.log remain historical.
No deliberate outage yet. After review, give only the next case: retained
MQTT off/on controls, then later hotspot loss/recovery and media checks, each
with separate instructions. Stage 2 is not accepted by this preparation.
