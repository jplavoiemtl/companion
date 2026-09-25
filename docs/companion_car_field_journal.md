# Companion in-car field analysis and improvements

Living journal for JP's car module. Keep new field results, open findings and improvement
proposals here rather than creating a separate analysis document for every ride.
Last updated: September 25, 2026.

## Current position

- SD diagnostics and iPhone retrieval completed the accepted bench scope, including
  increment 11, UI polish, no-card refusal and blank FAT32 initialization.
- JP installed the car firmware around September 24 at 21:30. The first morning ride
  demonstrated logging and successful iPhone export from the real car module.
- Two G-meter freezes correlate strongly with measured blocking MQTT reconnects.
  Both reconnects succeeded. The initiating WiFi loss remains unexplained.
- Field observation continues. No firmware improvement is approved or implemented by
  this journal. Do not treat one successful ride as long-term reliability proof.
- Reference: [bench results](sd_iphone_log_download_bench.md),
  [retrieval spec](sd_iphone_log_download_spec.md),
  [accepted UI polish](sd_iphone_log_download_ui_polish.md).

## How to add a field session

Append dated sessions below with a stable ID (F001, F002, ...). Preserve earlier
observations; put corrections in a dated note rather than silently rewriting evidence.
Update the findings and improvement tables when the new evidence changes a conclusion.

For each session record:
- JP's approximate local time, screen, symptom and estimated duration; distinguish
  those observations from device measurements. Note parked/driving and unusual power
  or phone events only when known. No interaction while driving is required.
- Firmware commit/build label if known, boot IDs and log coverage, including time-zone
  offset and clock quality. Never infer an exact flashed commit from current Git HEAD.
- Evidence filenames, byte lengths, CRC/SHA where checked, and screenshots used.
- Measured event timeline, logging/connection/recovery results, and evidence gaps.
- Which finding the session supports or contradicts, and the next decision.

Suggested evidence handling: keep each export and its screenshots together in a dated
folder outside Git, such as 2026-09-25-first-ride. Generic screenshot names get replaced
on later exports. The first session has now been archived by copying its three files; originals remain
in Downloads. Future sessions should use their own dated folder. Reference
original filenames and hashes here rather than committing whole operational logs.
A current.log snapshot normally excludes its own transfer END; the last-result view or
later log can supply that completion evidence. Avoid requesting another transfer when
existing evidence is sufficient.

## Open findings

| ID | Finding | Evidence / confidence | Status |
|---|---|---|---|
| I001 | G-meter pauses during MQTT recovery | F001: two ~8 s loop gaps, ~7.9 s synchronous connects, G-meter active; strong explanation of JP's observations | Open: design review proposed |
| I002 | Hotspot link loses beacons | F001: two beacon_timeout episodes followed by successful recovery; cause unknown | Observe future field events |
| I003 | Other image/Live latency | F001: 1.220 s main-loop image gap and 2.688 s Live frame gap; distinct from the G-meter freezes | Monitor; no separate fix justified yet |

Do not count retry no_ap_found/sta_leaving records as separate full outages without
checking the timeline. Do not attribute unknown-freshness TLS errors to a current TLS
failure. Health snapshots can be stale while the main loop is blocked.

## Possible improvements and decision gates

### P001 - Keep UI and IMU responsive during MQTT reconnection

Priority: first proposed improvement, linked to I001. Goal: reconnect without freezing
the G-meter or other main-loop UI work. The current synchronous call is measured to
block for about eight seconds; reducing the normal IMU rate is not the issue.

Before code, review connection ownership and call paths with Claude. Compare a genuinely
nonblocking connection approach with an isolated connection worker only if client/TLS
ownership and handoff can be made explicit. No architecture is selected here. Moving
the existing shared client to another task without an ownership design is not acceptable.
Simply lowering timeouts could trade successful recovery for repeated failures and is
not an established solution. Existing per-stage timeouts do not bound total connect time.

The design must cover MQTT callbacks/subscriptions, WiFi changes, image/Live interaction,
cancellation/power-down, bounded memory and the existing 20480-byte memory gate.
Set a measurable UI responsiveness target during design review. Keep validation focused:
one controlled bench reconnect case at a time, then ordinary car observation. Preserve
successful recovery, subscriptions, logging and image readiness. JP approves implementation;
Claude reviews the code before JP builds/flashes. No new case is issued by this proposal.

### P002 - Add targeted timing detail only if needed

The current log already identifies the blocking MQTT span. DNS/TCP/TLS/CONNACK timing is
not separated. Consider bounded phase timing only if it is needed to choose or validate
P001; avoid high-volume per-frame or per-sample records. Do not claim such measurements
exist today. A second useful measure could be UI/IMU service gaps, with a defined metric
and bounded reporting, if loop gaps alone cannot validate the selected design.

### P003 - Investigate recurring hotspot loss from field patterns

Collect time, recovery duration and known phone/power circumstances across sessions.
The first ride does not identify weak signal, cellular handover or iPhone behavior as
the cause. Change WiFi policy only when repeatable evidence supports it; do not make
speculative power-save, retry or radio changes while diagnosing UI blocking.

## Working agreement

Keep the accepted reduced bench scope. Reopen a waived case only for relevant evidence
or a code change. JP performs builds/flashes; firmware changes go through Claude review.
No new IMU average-rate or writer-core investigation without new relevant symptoms.
The next useful field input is a timestamped symptom plus a later log export, not an
expanded test suite. Deployment and successful retrieval do not close I001 or I002.

## Field sessions

### F001 - September 24 evening installation and September 25 first ride

Firmware identity: log reports compiled "Sep 24 2026 21:02:06", build label
stage3-context; CAR selection was confirmed in the workspace before deployment.
Exact flashed Git commit is not encoded in this evidence and is not assumed.
Analysis first recorded at e3951ea; consolidated here without changing its conclusions.

JP installed the car firmware around September 24 21:30, tested parked, and drove
about 20 minutes the next morning. He noticed two roughly three-second G-meter freezes
around 07:04, without noticing connection loss. Analysis below uses local -04:00 times.

#### Evidence and retrieval

Preserved evidence folder (September 25, copied at JP's request):
`C:\Users\photo\Downloads\Companion-car-evidence\2026-09-25-first-ride`.
All three copies were verified SHA256-identical to their originals; originals retained.
- `start-unknown_41-1-current-98264.log`: hash below.
- `listing.PNG`: SHA256 58bb086e36e5f45c545d8ff933a6a023a0af89995ef1a24d315bc511407d8e2e.
- `last-result.PNG`: SHA256 7b723808354a3a15702570dfb29815ef900e5468a152f1e0e25dd4485f8aed85.


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

#### Two G-meter freezes: strong correlation with blocking reconnects

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

#### Other observations

- Morning records show zero logger drops/truncated and slow writes=0. Write max 7199 us,
  flush max 10515 us, SD max 165686 us, unchanged around the freezes. Logging continued
  during a main-loop stall: 07:04:55 health snapshot_age_ms=6349, so wifi=0 there is
  stale main-task state, not a contradiction of the earlier driver got-IP event.
- Writer stack minimum 3704; retained DMA largest 24564 and sampled internal blocks
  remain above the 20480 gate. No evidence here of exhaustion or logging corruption.
- Other timing events: manual history image at 07:00:16 caused a 1220 ms loop gap;
  Live had a 2.688 s frame gap at 07:01:09.463. Separate from G-meter incidents.
- MQTT-triggered image at 07:08:30 and subsequent Live handover succeeded after recovery.

#### Disposition

First real-car fresh-card logging and phone retrieval are demonstrated. User-observed
G-meter freezes are a field responsiveness issue, with measured blocking MQTT reconnects
as the supported mechanism; hotspot loss trigger remains unresolved. No firmware edit,
build or flash for this analysis. Do not reopen accepted IMU average-rate investigations.
Continue field observations with approximate time/screen/symptom and a later log export.
Proposed next code work, if JP wants to address responsiveness: review a bounded design
for keeping UI/IMU responsive during MQTT recovery, preserving network/TLS ownership;
do not simply move the existing shared client to another task without that review.
