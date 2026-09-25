# First car ride - September 25, 2026

JP installed the car firmware around September 24 21:30, tested parked, and drove
about 20 minutes the next morning. He noticed two roughly three-second G-meter freezes
around 07:04, without noticing connection loss. Analysis below uses local -04:00 times.

## Evidence and retrieval

Downloads/start-unknown_41-1-current-98264.log: 98264 bytes, independently computed
CRC32 F1E81AFB; SHA256
4aa2f1a6da3baceb0c466bcd4bf8202ef8839bbe09b6772ef70a991e82b57e43.
Both listing.PNG and last-result.PNG were visually inspected. The latter shows expected,
transport and writer bytes all 98264, both CRCs F1E81AFB, match, result ok, appends resumed.
Pause 1214814 -> resume 1216325 ms = 1511 ms; listing grows to 99138 bytes afterward.
Actual exported file agrees with the device size and CRC, proving phone export integrity
at that level. No independent physical-card byte comparison is claimed.

The file spans boots 39/40 (evening) and 41 (morning). Its first header has unknown clock,
so start-unknown is correct. Boot 41 reports power_on and clock sync at 06:49:10.938;
the snapshot ends at HTTP_GET_BEGIN 07:09:07.722. The transfer's own END cannot be in
its frozen snapshot; screenshot supplies final result. No morning reboot is recorded.
Panel Stop after this download is not in supplied evidence; do not claim verified exit.

## Two G-meter freezes: strong correlation with blocking reconnects

| Event | First interruption | Second interruption |
|---|---|---|
| WiFi beacon_timeout | 07:04:12.232 | 07:04:36.159 |
| WiFi got IP again | 07:04:15.849 | 07:04:49.692 |
| MQTT connect began | 07:04:15.952 | 07:04:49.796 |
| MQTT recovered | 07:04:23.807 | 07:04:57.689 |
| MQTT call duration | 7855 ms | 7893 ms |
| Measured main-loop gap | 7985 ms | 8018 ms |

Screen 3 (G-meter) is active through both reconnects. Between them JP navigated home
and back (07:04:28.977-31.149). Both reconnects succeed and resubscribe; green UI
status follows. Total time from first WiFi loss to MQTT recovery is 11.575 s and 21.530 s.
Driver also emits no_ap_found/sta_leaving during retries, some suppressed. These are
retry events within two outages, not independent additional complete outages.

Source inspection: companion.ino loop runs UI/IMU background work on the main task;
net_module.cpp calls synchronous PubSubClient connect inside the mqtt_connect span.
Thus the roughly 7.9 s reconnect calls account for nearly all of each 8 s loop gap and
strongly explain the two observed dot freezes. JP's three-second estimate is visual;
the logged loop duration is about eight seconds. This is not evidence of an SD stall.

WiFi's underlying cause is NOT established. Beacon-timeout RSSI was -35 and -41 dBm;
these last reported levels do not establish what happened during the lost beacons.
No inference that cellular handover, weak signal or the iPhone itself caused it.
MQTT_LOST tls_code=48 has tls_fresh=unknown and is not proof of a fresh TLS fault.
The combined reconnect span does not separate DNS/TCP/TLS/CONNACK time. Per-stage
5-second limits are not a five-second total-connect budget.

## Other observations

- Morning records show zero logger drops/truncated and slow writes=0. Write max 7199 us,
  flush max 10515 us, SD max 165686 us, unchanged around the freezes. Logging continued
  during a main-loop stall: 07:04:55 health snapshot_age_ms=6349, so wifi=0 there is
  stale main-task state, not a contradiction of the earlier driver got-IP event.
- Writer stack minimum 3704; retained DMA largest 24564 and sampled internal blocks
  remain above the 20480 gate. No evidence here of exhaustion or logging corruption.
- Other timing events: manual history image at 07:00:16 caused a 1220 ms loop gap;
  Live had a 2.688 s frame gap at 07:01:09.463. Separate from G-meter incidents.
- MQTT-triggered image at 07:08:30 and subsequent Live handover succeeded after recovery.

## Disposition

First real-car fresh-card logging and phone retrieval are demonstrated. User-observed
G-meter freezes are a field responsiveness issue, with measured blocking MQTT reconnects
as the supported mechanism; hotspot loss trigger remains unresolved. No firmware edit,
build or flash for this analysis. Do not reopen accepted IMU average-rate investigations.
Continue field observations with approximate time/screen/symptom and a later log export.
Proposed next code work, if JP wants to address responsiveness: review a bounded design
for keeping UI/IMU responsive during MQTT recovery, preserving network/TLS ownership;
do not simply move the existing shared client to another task without that review.
