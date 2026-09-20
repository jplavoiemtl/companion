# Stage 2 network evidence - first bench handoff

JP accepted Stage 2 on September 20: "Yes I accept let's proceed".
The pending-acceptance statements below are historical. Stage 3 handoff: [STAGE3.md](STAGE3.md).

## 2026-09-20 08:10-08:11: normal Live comparison passes; Stage 2 ready for owner review

JP reports video and test ran fine. Same boot87/build as outage case, Wi-Fi/MQTT
connected throughout. Normal Live178/60.183s=2.9576 FPS; outage Live
176/60.326s=2.9175 FPS. Outage is 1.36% below normal, inside about5%
target. Comparable average images18.0 versus18.2KB; HTTP329 versus334ms,
TTFB153ms both. This is an MQTT connected/outage comparison, not a new
logging-off/on measurement or proof that logging has no performance cost.
Reuse accepted Stage1B logging/USB comparisons within their documented scope.
Normal first frame1219ms, max gap1065ms; JP observed normal video, no new stall.

New Live connect spans id11/12 succeed662/734ms. No MQTT loss/retry during
normal Live. Application counters progress4/6/8 (power2/3/4, energy2/3/4),
showing inbound traffic after the previous recovery. Sampled TLS largest31732
(66/73 samples), full Live28660 (6018 samples), above unchanged20480 gate.
Ready/synced; queuehigh6; drops/errors/suppressed/truncated/slow writes0;
writer margin3144 stable. USB idle/unpaused/resultok, no link losses/reset.

Download 262208 bytes, CRC32 84D63CFE, SHA256
28205c58bcdf0894dd3e85074e43965069b467244b19b79285f3f17adc8968b7.
Browser CRC OK,2.57s; previous258232-byte snapshot exact prefix. Records1-74
contiguous for boot87, all12 spans paired, no NET_FORMAT_ERROR. Sources:
[console](../../docs/bench_data/sd_stage2_2026-09-20_boot87_live_normal/console.txt) and
[current log](../../docs/bench_data/sd_stage2_2026-09-20_boot87_live_normal/87-current.log.txt).

Stage2 planned startup/health, MQTT off/on, Wi-Fi loss/recovery, post-recovery
Latest, corrected RSSI/suppression and Live outage/normal comparison now pass.
Present Stage2 for JP's explicit acceptance before Stage3 operation-context
logging. No further bench case assigned now; no firmware changes/build/flash.
This is bounded bench acceptance evidence, not exhaustive event coverage:
LOST_IP, secondary-profile failover and every error/notification branch were
not separately induced. Existing monitor-close limitation and unsupported-card
deferral remain. Stage3-4 unstarted. Commit/push completed results per JP request.

Earlier entries below are historical and superseded by this result.


## 2026-09-20 08:06-08:08: Live during MQTT outage passes, boot 87

JP reports video and test ran fine. Same corrected Stage2 build; boot87 is
power_on, no reset during case. Intentional off precedes test attempt id6,
5003ms/state-2/TLS-1 generic, freshness unknown. Live starts after that attempt.
Paired Live connects id7/id8 succeed in693/625ms. MQTT_RETRY_POLICY deferred
at uptime50499 and released/media_clear at110161. No MQTT attempt within
Live; next test id9 starts110262, 101ms after release, fails in5003ms.
The on command waits behind this already-running call, then restores real
broker. Real id10 succeeds603ms/state0/recovery1; subscriptions/calibration
accepted. This is expected blocking-command behavior, not a new stall.

176 frames /60.326s = 2.9175 FPS. First frame1250ms, max gap939ms,
average18.2KB/frame, HTTP334ms (TTFB153+xfer181), decode61ms, blit80ms.
No direct regression percentage against last night's smaller-image runs:
scene/image size and network conditions differ. Next normal connected run
provides a same-session comparison; this is not a logging-off/on comparison.
Live sampled largest minimum34804 (6033 samples); both TLS windows42996
(69/63 samples). Test attempts63476 each (500 samples), real recovery49140
(61 samples). All above20480. Queuehigh6; drops/errors/suppressed/truncated/
slow writes0; writer margin3144 after download. USB idle/unpaused/resultok,
no link loss. NET_HEALTH correctly shows mqtt0 during Live and mqtt1 afterward;
inbound remains2, age advances: application counters, not keepalive evidence.

Download 258232 bytes, CRC32 C5BA48A1, SHA256 2aa3eafb32f37b070cc61f4fe801a7adf3d2724aef41b9923dc35fdf3746a9ef.
CRC OK,2.47s; prior boot86 snapshot exact prefix. Boot87 records1-59 contiguous,
all ten spans paired; no NET_FORMAT_ERROR. Sources:
[console](../../docs/bench_data/sd_stage2_2026-09-20_boot87_live_outage/console.txt) and
[current log](../../docs/bench_data/sd_stage2_2026-09-20_boot87_live_outage/87-current.log.txt).

Next single case: same build/session, hotspot on and normal MQTT connected,
DTR=true/RTS=false, test switches off. Send status, run one full Live cycle
with no off/on commands or downloads, then status, download current, status.
Send console/log and visual observations. Compare frame count/duration and
network/image metrics with this outage run; target no more than about5% loss,
interpret marginal differences with network variability. No rebuild. Stage2
remains unaccepted; Stages3-4 unstarted. Completed result commit/push authorized
by JP. iPhone plan untouched; no firmware changes or new tests required.

Earlier entries below are historical and superseded by this result.


## 2026-09-19 22:27-22:29: corrected hotspot case passes, boot 86

JP reports normal operation. Both diagnostic corrections pass on hardware:
raw_rssi=-128/raw_valid=0 retains rssi=-31/rssi_source=last_valid. Initial
beacon_timeout reason200 has raw_valid=1. Alternating reasons201 and36 are
reported about every7.245 seconds per reason, with four suppressed events
between pairs. NET_HEALTH wifi_suppressed=16 confirms cumulative suppression;
logger queue suppression/drops remain zero. Association resets pending count;
reason36 is sta_leaving. Recovered valid health RSSI is -18.

MQTT_LOST state=-3 precedes recovery; TLS48 freshness unknown is not a proven
new TLS failure. Reconnection id6 succeeds in486ms, state0/recovery1, all three
subscriptions/calibration accepted. Latest id7 headers714ms/HTTP200, total1135ms,
19665bytes; motion publish accepted. Sampled recovery minimum55284 (49 samples),
HTTPS28660 (71 samples), both above20480. Writer margin3640 before download,
3144 after; queuehigh7, zero drops/errors/truncation/slow writes. USB ends idle,
unpaused/resultok with no link losses. No reset during captured test. Boot86
itself reports a watchdog before capture with prior boot85 idle breadcrumbs;
this capture does not establish its cause. The earlier monitor-close limitation
remains unresolved and is not newly diagnosed here.

Download 241825 bytes, CRC32 9CC35ADA, SHA256 2f408372b9feafd5e48af8822702236aaa407b1629b0703294e4c0b337a97c8d.
Prior boot84 snapshot is an exact byte prefix; 62 contiguous boot86 records,
seven paired spans, no format errors. CRC OK;2.36s. Sources:
[console](../../docs/bench_data/sd_stage2_2026-09-19_boot86_hotspot/console.txt) and
[current log](../../docs/bench_data/sd_stage2_2026-09-19_boot86_hotspot/86-current.log.txt).

Next single case, same build: keep hotspot on, dashboard, browser DTR=true/
RTS=false and test switches off. Status; send off once. After first TEST MQTT
attempt END, promptly start one full Live cycle (before next15-second retry).
Keep MQTT off throughout Live; no downloads during it. After normal Live finish,
send on once, wait for real MQTT recovery, then status/download current/status.
Expect media retry deferral, paired Live connect records, no MQTT attempts while
Live is active, release after media ends, normal video and recovery, CRC OK,
zero drops/errors and memory>=20480. Capture full console, log and visual report.
No rebuild. Stage2 remains unaccepted; Stage3-4 unstarted. No firmware changes.

Earlier entries below are historical and superseded by this result.


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
