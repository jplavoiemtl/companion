# P005 / P006 - faster MQTT recovery and correct broker profile (proposal)

September 25, 2026. Author: Claude, at JP's request after the F003 review. **Status:
proposal for Codex review and design.** This deviates from the journal's usual
direction (Codex designs, Claude reviews) because JP asked Claude to write it up.
Codex owns the resulting design and implementation; this note is input, not an approved
design. No firmware change, build or flash accompanies it.

JP has agreed to the direction: P005 first, P006 alongside it as a small separate change,
then the existing P004 (image-request responsiveness). Evidence: F003 (`field_journal.md`,
Codex analysis and Claude review, b2fda36 / f299059).

## P005 - reconnect promptly and stop cutting off slow-but-working handshakes

### Evidence (F003, boots 45-49, P001 firmware)

- **The fixed wait dominates recovery.** In 7 of the 9 beacon-loss episodes, IP returned
  within 3.5-5.4 s, but the first MQTT attempt always started **exactly 15.0 s** after the
  loss. That is about 10-11.5 s of idle wait per episode. At 20:26, 45 of the 59.2 s
  outage were three 15.0 s waits.
- **The 5 s caps crowd the observed distribution.**
  - Successful phases reached TLS 4975 ms, MQTT exchange 4986 ms and TCP 3903 ms.
  - Failures sit at the caps: TLS 5003 and 5004 ms (twice), TCP 5003 ms.
  - After the 15:28:56 TLS cap failure, the next attempt succeeded in 1.9 s.
- **The 15 s interval protected LVGL from blocking attempts** (old `netCheckMqtt`
  comment: a long attempt would otherwise be "starving LVGL"). P001 removed that cost:
  F003 recovery windows show UI <= 23 ms and IMU/loop <= 25 ms.

### Proposed change A - one prompt attempt after a loss

Current: `netMainTick()` stamps `lastMqttAttempt = millis()` when it adopts a loss
(net_module.cpp:305), so the next attempt waits the full `MQTT_RECONNECT_INTERVAL`
(15 s). Attempts need the link, so after a WiFi drop the attempt starts at the first loop
turn at least 15 s after the loss and after GOT_IP.

Proposal:
- On adopting a loss of a previously **established** connection, make the owner eligible
  immediately: `lastMqttAttempt = millis() - MQTT_RECONNECT_INTERVAL`, the same idiom
  `restoreBenchMqtt` already uses. With WiFi still associated (the 20:26 case) the attempt
  starts right away. After a WiFi drop it starts as soon as the owner sees the link up
  again.
- **Only one prompt attempt per loss.** If it fails, keep the existing 15 s stamp after
  completion (net_module.cpp:279). Do not re-arm on every GOT_IP. A flapping link (F002:
  three drops in 70 s) must not produce back-to-back attempts or repeated lease holds.
- Leave everything else unchanged: bench off/on adjustments, initial five-failure budget,
  the cleanup-before-new-attempt rule, and the lease/admission rules.
- Optional, for Codex to decide: a short settle delay (about 1 s) after GOT_IP before the
  prompt attempt. F001/F003 do not show whether attempts started immediately after GOT_IP
  are slower. Start without it and let phase timing show.

Expected effect, from the F003 episodes: typical beacon-loss recovery from about 20-26 s
to about 5-15 s (IP return plus attempt time). The 20:26 case from 59 s to roughly 30 s
with the same failures, or less with change B.

### Proposed change B - raise the TLS and MQTT-exchange caps to 10 s, keep TCP at 5 s

- **TLS handshake 5 -> 10 s.** `secure.setHandshakeTimeout(10)` and the TLS phase
  allowance 5000 -> 10000 (net_worker.cpp:216, 262). This only affects the handshake loop
  (`ssl_starttls_handshake` checks `handshake_timeout`).
- **MQTT exchange 5 -> 10 s.** Phase allowance 5000 -> 10000 (net_worker.cpp:267). The
  CONNACK wait uses the client socket timeout (`setSocketTimeout(5)`, :214). Either raise
  it to 10 for the connect exchange only, or rely on the phase deadline. Keep the ONLINE
  per-`loop()` 5 s operation deadline unchanged.
- **TCP stays at 5 s, deliberately.** `setConnectionTimeout()` also becomes the
  connection's lifetime `socket_timeout` / `SO_RCVTIMEO` / `SO_SNDTIMEO` and write-progress
  timeout (ssl_client.cpp:116, 171-172, 453). Raising it would lengthen ONLINE write stalls.
  F003 has only one TCP cap failure and a 3.9 s maximum success, so the evidence does not
  justify that coupling.
- Budgets: attempt = DNS 15 + TCP 5 + TLS 10 + MQTT 10 + subscriptions 2 + 3 allowance =
  **45 s** (from 35); `worker_stuck` **50 s** (from 40). Same single dispatch clock and
  per-phase admission rule.

**Trade-off for JP.** The media/retrieval lease covers TCP setup through READY. Its worst
case per attempt grows from about 17 s to about 27 s, during which Latest/Live/download
entry are refused with "Reconnecting. Try again." Typical attempts in F003 held it for
0.6-11 s. On a link this slow, media requests usually fail anyway (20:26: image requests
hit their own 5 s bound).

### Watchdog and ownership

No new polling loop, no change to the tick-delay rules, the worker cadence, the DNS slot
lifetime or the lease protocol. Longer handshakes are still cooperative:
`ssl_starttls_handshake` yields `vTaskDelay(2)` between steps.

### Validation (bounded)

- Host checks: prompt eligibility only after an established-connection loss, one prompt
  attempt per loss, 15 s after failures, and no re-arm on repeated GOT_IP. Pin the new
  constants (TLS 10 s, MQTT 10 s, TCP 5 s, attempt 45 s, stuck 50 s). Bench and budget
  semantics unchanged.
- Bench: reuse retained cases 1 and 3 (successful reconnect, one hotspot flap). Check that
  the first attempt starts at link-up, not 15 s after the loss, and that SERVICE stays
  within the P001 figures. No new matrix.
- Field: compare loss-to-recovery against the F003 table. Expect fewer cap-edge failures.

## P006 - configure the MQTT profile from the network actually joined

### Evidence (F003 boot 46)

`WIFI_BEGIN profile=secondary` at 8.4 s, but the driver associated with the **primary**
network at 19.8 s (`WIFI_ASSOC profile=primary connection=1`). MQTT was then configured
as connection 2 (`MQTT_CONFIG connection=2`), with `MQTT_WIFI_PROFILE mismatch=1` three
times in that boot.

### Cause

`connectToWiFi(connection)` calls `netConfigureMqttClient(connection)` with the
**requested** index (companion.ino:790-792). WiFi can end up on the other network, for
example a still-pending earlier association. The late-WiFi path in `loop()` already
derives the profile from the actual network: `(WiFi.SSID() == ssid1) ? 1 : 2`
(companion.ino:2526).

### Impact

Harmless in the CAR build: both MQTT profiles use the same host and port (checked by
comparing the configuration, without recording it). In the HOME build, profile 1 is port
1883 for home WiFi and profile 2 is 9735 for the iPhone, so a mismatch would pair the
wrong broker profile and fail or misroute MQTT.

### Proposed change

In `connectToWiFi`, configure MQTT from the joined network using the same expression as
the late-WiFi path. Record one event when requested and actual differ; the existing
`MQTT_WIFI_PROFILE` record can serve. No change to WiFi retry or scan policy. Host check:
requested 2 but joined `ssid1` configures profile 1.

## Order and scope

1. P005 A + B with P006, one small increment, Claude code review, one bench session
   (cases 1 and 3).
2. Then P004 design (image-request responsiveness), as already listed in the journal.

Out of scope: WiFi radio/power-save/retry changes (P003 is still investigating hotspot
loss), timeout changes to TCP/DNS, and image/Live behaviour.
