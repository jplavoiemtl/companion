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
- Reference: [bench results](../sd_iphone_log_download_bench.md),
  [retrieval spec](../sd_iphone_log_download_spec.md),
  [accepted UI polish](../sd_iphone_log_download_ui_polish.md).

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

Evidence lives beside this journal in `evidence/YYYY-MM-DD-description/`, ignored by
Git. Preserve original names and verified hashes. Each session gets its own folder so
generic screenshot names cannot overwrite another session. The first session's three
files have been copied here and verified; Downloads copies remain as a separate copy.
Git does not back up this evidence directory: include it in normal project backups.
Do not force-add raw evidence without JP's approval. Both assistants read the same local
files and keep their analysis in this tracked journal.
A current.log snapshot normally excludes its own transfer END; the last-result view or
later log can supply that completion evidence. Avoid requesting another transfer when
existing evidence is sufficient.

## Shared editing protocol

- Edit sequentially: JP hands the journal to one assistant at a time. Check Git status
  and read the latest journal before editing; preserve another assistant's work.
- Each session has JP observations, attributed Codex and Claude analyses, and an agreed
  findings / unresolved questions section. Do not overwrite another author's conclusions.
- Claude should inspect raw evidence first, then compare with Codex's analysis. Add
  concrete agreements, disagreements and evidence gaps, not a duplicate event transcript.
- Record author and date on analyses or corrections. Cross-review consensus is separate
  from JP's approval to implement. Proposed improvements remain unapproved until JP agrees.
- Maintain the one shared findings table and improvement list. Annotate disputed items
  rather than silently replacing them. Record implementation and validation evidence
  when a proposal eventually progresses.

## Open findings

| ID | Finding | Evidence / confidence | Status |
|---|---|---|---|
| I001 | G-meter pauses during MQTT recovery | F001: two ~8 s loop gaps, ~7.9 s synchronous connects, G-meter active; strong explanation of JP's observations | Open: design review proposed. Claude (Sep 25): agreed; source suggests WiFi-up/no-internet failures could block repeatedly, not yet observed |
| I002 | Hotspot link loses beacons | F001: two beacon_timeout episodes followed by successful recovery; cause unknown | Observe future field events |
| I003 | Other image/Live latency | F001: 1.220 s main-loop image gap and 2.688 s Live frame gap; distinct from the G-meter freezes | Monitor; no separate fix justified yet |

Do not count retry no_ap_found/sta_leaving records as separate full outages without
checking the timeline. Do not attribute unknown-freshness TLS errors to a current TLS
failure. Health snapshots can be stale while the main loop is blocked.

## Possible improvements and decision gates

### P001 - Keep UI and IMU responsive during MQTT reconnection

Status: proposed; not approved or implemented.
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

Status: conditional proposal; not approved or implemented.
Claude (September 25): disputes the priority, recommending bounded DNS/connect-phase
timing before the P001 design is chosen. See F001 agreed findings; JP decides.

The current log already identifies the blocking MQTT span. DNS/TCP/TLS/CONNACK timing is
not separated. Consider bounded phase timing only if it is needed to choose or validate
P001; avoid high-volume per-frame or per-sample records. Do not claim such measurements
exist today. A second useful measure could be UI/IMU service gaps, with a defined metric
and bounded reporting, if loop gaps alone cannot validate the selected design.

### P003 - Investigate recurring hotspot loss from field patterns

Status: observing field data; no WiFi-policy change approved.

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

#### JP observations

JP installed the car firmware around September 24 21:30, tested parked, and drove
about 20 minutes the next morning. He noticed two roughly three-second G-meter freezes
around 07:04, without noticing connection loss. Analysis below uses local -04:00 times.

#### Codex analysis - September 25, 2026

##### Evidence and retrieval

Preserved evidence folder (September 25, copied at JP's request):
`docs/car_testing/evidence/2026-09-25-first-ride/` (relative to the project root).
Originally preserved in Downloads; project copies verified against those copies.
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

##### Two G-meter freezes: strong correlation with blocking reconnects

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

##### Other observations

- Morning records show zero logger drops/truncated and slow writes=0. Write max 7199 us,
  flush max 10515 us, SD max 165686 us, unchanged around the freezes. Logging continued
  during a main-loop stall: 07:04:55 health snapshot_age_ms=6349, so wifi=0 there is
  stale main-task state, not a contradiction of the earlier driver got-IP event.
- Writer stack minimum 3704; retained DMA largest 24564 and sampled internal blocks
  remain above the 20480 gate. No evidence here of exhaustion or logging corruption.
- Other timing events: manual history image at 07:00:16 caused a 1220 ms loop gap;
  Live had a 2.688 s frame gap at 07:01:09.463. Separate from G-meter incidents.
- MQTT-triggered image at 07:08:30 and subsequent Live handover succeeded after recovery.

##### Disposition

First real-car fresh-card logging and phone retrieval are demonstrated. User-observed
G-meter freezes are a field responsiveness issue, with measured blocking MQTT reconnects
as the supported mechanism; hotspot loss trigger remains unresolved. No firmware edit,
build or flash for this analysis. Do not reopen accepted IMU average-rate investigations.
Continue field observations with approximate time/screen/symptom and a later log export.
Proposed next code work, if JP wants to address responsiveness: review a bounded design
for keeping UI/IMU responsive during MQTT recovery, preserving network/TLS ownership;
do not simply move the existing shared client to another task without that review.

#### Claude analysis - September 25, 2026

Raw evidence inspected before reading the Codex analysis, at head b4464cf.

##### Evidence integrity

- All three files are SHA256-identical to the hashes recorded above and to the Downloads
  copies. The log is 98264 bytes, CRC32 F1E81AFB (independently recomputed), pure ASCII,
  LF-only, and ends on a complete line. This agrees with the last-result screenshot
  (expected, transport and writer 98264, both CRCs F1E81AFB, match, appends resumed).
- Sequence numbers are gap-free within each boot: 39 (1-97), 40 (1-109), 41 (1-308).
  No record was lost or truncated in the exported file.
- Boots 39 and 40 each end with POWER_DECISION shutdown (usb=0, idle 60 s) followed by
  SESSION_END pending=0. The WiFi loss just before each shutdown (auth_expired, then
  no_ap_found) is consistent with the car being switched off, not a field fault. Both
  evening panel downloads were complete (10931 and 31363 bytes, CRC match, paused 186
  and 456 ms), and both ended with panel_stop result=ok.
- PREVIOUS_BREADCRUMB valid=0 on each boot is expected after a PMIC power-off.

##### G-meter freezes

I agree with the Codex timeline and mechanism; every value in its table checks against
the raw records. Each loop gap is the connect call plus about 130 ms of other work, which
includes the fixed `delay(100)` before `connect()` in `netCheckMqtt()`. The health record
at 07:04:55.293 was written during the second block (snapshot_age_ms=6349), which confirms
Codex's point that health snapshots can be stale while the main loop is blocked.

Two additions:

1. **The constant reconnect duration is a lead.** The two reconnects took 7855 and 7893 ms,
   38 ms apart, while the three boot-time connects in this file took 455-1081 ms. Two
   independent reconnects landing within 0.5% of each other point to a fixed wait inside
   the connect path rather than cellular variance. Source check: in the CAR configuration
   the broker is a hostname (only the configuration structure was checked, not the value),
   so DNS resolution runs inside the timed span. `netCheckMqtt()` sets TCP (5000 ms), TLS
   handshake (5 s) and CONNACK (5 s) limits, but no DNS limit. Which phase holds the
   ~7.9 s is **not measured**; this is a hypothesis, not a finding.
2. **The observed case is the mild one.** Both reconnects succeeded. Reconnects are only
   attempted while WiFi is connected, so the likely bad case in a car is WiFi up with no
   internet behind it, for example the iPhone keeping its hotspot through a cellular dead
   zone. From source, each failed attempt would then block for up to the unmeasured DNS
   time plus the 5 s stage limits it reaches, and repeat 15 s after each return
   (`MQTT_RECONNECT_INTERVAL`, stamped after the call). That would give repeated freezes,
   not two. Derived from source only; not yet observed in the field.

##### Storage and memory

Agree with Codex: zero drops, truncation and slow writes; write/flush maxima unchanged;
writer stack minimum 3704; internal minimum 34296 bytes, stable since boot; sampled
internal and DMA largest blocks stay above the 20480 gate. `sd_max_us` (~165 ms) is set
in the first minute of every boot and never rises during driving, so it is a
startup/mount cost, not a field stall.

Addition: SD queue high-water rose from 6 to 8 at the 07:04 event burst. Eight of 16 is
exactly the transfer queue-pressure early-abort threshold (spec: 50 %, 8 of 16). It is
harmless outside a transfer and nothing was dropped. However, a download running during a
WiFi flap would probably have early-aborted by design, even before the link loss itself.
No change proposed; note it when interpreting a future aborted download.

##### Other observations

Agree that the 07:00:16 image gap (1220 ms) and the 07:01:09 Live gap (2.688 s) are
separate from the freezes. The 07:00 Live feed averaged 2.1 fps (128 frames in 60 s,
transfer-dominated) and the 07:08 feed about 2.9 fps (34 frames in 11.5 s). Both are
below the 3.2-4.1 fps bench range, which is expected on a moving cellular link. The
07:08:30 MQTT image and motion handover to Live succeeded, as Codex states.

##### Assessment of proposals

- **P001:** agree it is the right first improvement, and that moving the shared client to
  another task without an ownership design is unacceptable. Additional input for the
  design review: PubSubClient's `connect()` and `WiFiClientSecure` are synchronous, so a
  genuinely nonblocking approach in practice means an MQTT client with its own event-driven
  task (for example ESP-IDF's esp-mqtt), not a change of timeouts. That option must be
  weighed against the TLS memory constraint and the 20480-byte gate. The design must
  bound the WiFi-up/no-internet case above, not only the observed successful reconnect.
- **P002:** **disagreement on priority.** Codex makes phase timing conditional. I recommend
  a small version before the P001 design is chosen: time DNS separately (one explicit
  lookup before connect, logging its duration) and record when `connect()` returns in
  relation to CONNACK. Reconnects are rare, so the volume is bounded. If the 7.9 s turns
  out to be a single fixed wait, such as a DNS retry, a narrow fix could remove most of
  the observed freeze independently of P001, while P001 remains necessary for the
  no-internet case. Still subject to JP approval; no implementation here.
- **P003:** agree. Both drops happened with strong last-reported RSSI, 24 s apart, and the
  link was stable for the rest of the ride. JP's note of phone circumstances (call,
  navigation, screen lock) at the time of a future drop is the most useful extra input.

#### Agreed findings and unresolved questions

Updated September 25 by Claude after cross-review of the Codex analysis.

Agreed (Codex and Claude):
- Evidence integrity and phone export are verified (size, CRC, gap-free sequence).
- The two G-meter freezes are explained by two ~7.9 s synchronous MQTT reconnects on the
  main loop after two short WiFi beacon-timeout outages. Both reconnects succeeded.
- Logging, storage and memory were healthy throughout. No SD stall contributed.
- The image and Live gaps are separate from the freezes. No IMU-rate or writer
  investigation is reopened.
- The cause of the WiFi outages is unknown. RSSI does not explain it, and TLS codes
  with unknown freshness are not evidence of a TLS fault.

Explicit disagreement:
- P002 priority. Codex: phase timing only if needed for P001. Claude: add bounded DNS and
  connect-phase timing first, because the constant 7.9 s suggests a fixed wait with a
  possibly narrow fix. JP decides.

Unresolved:
- Which connect phase holds the ~7.9 s (DNS, TCP, TLS or CONNACK)?
- How long the main loop blocks when WiFi is up but the broker is unreachable (derived
  from source, not yet observed).
- What initiated the two WiFi outages.
- Which P001 architecture preserves network ownership while servicing the UI.

JP has authorized shared documentation and review, not a firmware implementation.
