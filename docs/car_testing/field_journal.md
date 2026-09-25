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
- F002 repeats the UI stall with failed MQTT attempts (8.771 s and 5.115 s loop gaps),
  then automatic recovery. Two ~0.12 s storage operations were also recorded without loss.
- Field observation continues. JP approved P001 revision 2 and increment 1 on September 25;
  Claude cleared c9629df and the first worker TLS/CONNACK bench gate has technically
  passed on boot 130. JP acceptance / increment 2 go-ahead is pending.
  The car firmware has not been changed by this work. One successful ride is not long-term
  reliability proof.
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

Rule (agreed September 25, 2026): **one author, one reviewer.** Every piece of work has a
single owner who writes it and one reviewer who checks it. The two assistants do not
both produce full analyses or designs of the same thing. F001 predates this rule and
contains two full analyses; later sessions follow the workflow below.

- Edit sequentially: JP hands the journal to one assistant at a time. Check Git status
  and read the latest journal before editing; preserve another assistant's work.
- Record author and date on analyses, reviews and corrections. Cross-review consensus is
  separate from JP's approval to implement. Proposals remain unapproved until JP agrees.
- Annotate disputed items rather than silently replacing them. Record implementation and
  validation evidence when a proposal eventually progresses.

### Ride analysis workflow

1. **Codex is the primary analyst.** Codex inspects the raw evidence, writes the session
   entry (JP observations, evidence and hashes, timeline, conclusions) and updates the
   findings table, improvement list and the session's agreed findings / unresolved section.
2. **Claude reviews, when JP asks.** Claude spot-checks the key claims against the raw
   evidence, scans it for anything the entry missed and adds one dated "Claude review"
   block of about 10-15 lines: agreements, disagreements with evidence, additions. Claude
   does not re-transcribe the timeline and does not edit the shared tables or sections.
3. **Codex integrates the review.** Codex updates the findings table and agreed /
   unresolved section, stating any disagreement explicitly for JP to decide.
4. **Review is optional for routine rides.** Request it when JP saw a symptom, a new event
   type appears, or a conclusion would change a finding or proposal. Otherwise Codex's
   entry stands alone. Several rides may be batched into one export and one analysis.

### Design and code workflow

1. **Design:** Codex owns the design document. Claude reviews it and the review is
   recorded, with Codex incorporating agreed changes. If JP wants Claude's view before a
   design exists, Claude writes a short options note that Codex builds on; Claude does
   not write a competing design. JP approves the design before implementation.
2. **Code:** Codex implements with host checks. Claude reviews the commit against the
   approved design and reports blockers before JP builds. Codex applies fixes; Claude
   re-reviews only the changed parts.
3. **Hardware:** JP builds, flashes and runs one case at a time. Codex records results.
   Claude reviews results only when a case fails or looks unusual.

### Defaults so JP's prompts can stay short

- **New evidence:** any folder under `docs/car_testing/evidence/` that no session
  references yet. If there are several, handle them in date order as one session each,
  unless JP says they belong together.
- **Session ID:** the next number after the highest existing session (F001, F002, ...).
- **JP's observations:** whatever JP writes in the prompt. If the prompt has none,
  record "no symptoms reported" rather than asking.
- **Review target:** the newest session that has no "Claude review" block yet.
- **Always:** follow the workflow above, verify and record evidence hashes, make no
  firmware changes, builds or flashes, then commit and push the journal.

### Prompt templates for JP

Analysis request to Codex:

```text
New car evidence added. Analyze it following the field journal workflow.
My observations: <time, screen, symptom, duration, what the phone was doing>
```

Review request to Claude:

```text
Review the newest field session following the field journal workflow.
```

## Open findings

| ID | Finding | Evidence / confidence | Status |
|---|---|---|---|
| I001 | UI pauses during MQTT recovery | F001: two ~8 s G-meter gaps; F002: 8.771 s and 5.115 s gaps on screen 1 during failed connects, matching spinner symptom | Open: P001. Repeated failed attempts while associated now observed; internet outage itself not established |
| I002 | Hotspot link loses beacons | F001: two episodes; F002: three beacon-timeout losses in a 73.735 s MQTT recovery episode | Observe; underlying cause unknown. Later power-associated interruption tracked separately |
| I003 | Other image/Live latency | F001 image/Live gaps; F002 1.514 s Live-connect loop gap. Later Live disconnect was JP's intervention, not a field fault | Monitor separately from MQTT stalls |
| I004 | Occasional slow storage operations | F002: write 114.893 ms, flush 122.353 ms; slow counter 2, zero drops/truncation | Monitor; no evidence these explain multi-second UI freezes |

Do not count retry no_ap_found/sta_leaving records as separate full outages without
checking the timeline. Do not attribute unknown-freshness TLS errors to a current TLS
failure. Health snapshots can be stale while the main loop is blocked.

## Possible improvements and decision gates

### P001 - Keep UI and IMU responsive during MQTT reconnection

Status: JP approved [revision 2](p001_mqtt_responsiveness_design.md) and increment 1
after Claude focused check e3ae528. Codex implemented the owned worker on
`codex/car-improvements-p001`; [code review is pending](p001_increment1_handoff.md).
F002 requires failed reconnects and UI animation as well as successful recovery.
Priority: first proposed improvement, linked to I001. Goal: reconnect without freezing
the G-meter or other main-loop UI work. The current synchronous call is measured to
block for about eight seconds; reducing the normal IMU rate is not the issue.

The reviewed design selects permanent worker ownership of the MQTT client and transports,
with bounded snapshots/queues at the main-task boundary. A connect-only handoff or
sharing the old client across tasks is not used.
Simply lowering timeouts could trade successful recovery for repeated failures and is
not an established solution. Existing per-stage timeouts do not bound total connect time.

The design must cover MQTT callbacks/subscriptions, WiFi changes, image/Live interaction,
cancellation/power-down, bounded memory and the existing 20480-byte memory gate.
Set a measurable UI responsiveness target during design review. Keep validation focused:
one controlled bench reconnect case at a time, then ordinary car observation. Preserve
successful recovery, subscriptions, logging and image readiness. JP approves implementation;
Claude reviews the code before JP builds/flashes. No new case is issued by this proposal.

### P002 - Add targeted timing detail only if needed

Status: included in the P001 design and authorized increment 1 at JP's explicit request
on September 25; phase timing implemented, hardware measurements pending. Historical priority disagreement below is retained for provenance.
The new design includes bounded DNS, TCP setup, TLS handshake and MQTT-exchange timing;
no separate instrumentation-only flash is proposed.

F002 update (Codex, September 25): failed connect durations vary (8659 and 5004 ms),
then recovery takes 870 ms. A universal fixed 7.9-second wait is not supported across
rides. Phase attribution remains unknown; this neither proves nor rules out DNS waits.
The existing P002 priority disagreement is preserved pending review/JP decision.

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

### F002 - September 25 second ride, spinner pause around 08:51

#### JP observations

JP saw a WiFi disconnect around 08:51. The LVGL spinner continued turning initially,
then stopped for a few seconds. JP confirmed he trimmed earlier boots from the supplied
log. No answer yet about phone/power circumstances at 08:58:49; no circumstance assumed.

#### Codex analysis - September 25, 2026

##### Evidence and limits

Folder: `docs/car_testing/evidence/2026-09-25-second-ride/`.
Only `start-unknown_42-1-current-157410.log` was supplied: actual size 49755 bytes,
CRC32 of this excerpt 46050290; SHA256
881535583f14c1055d49017449f1906d973c4b68373b4adc39ac01953419922f.
JP confirmed trimming, so filename size 157410 is not the size of this evidence file
and the discrepancy is not evidence of transfer corruption. Boot 42 has all sequence
numbers 1-250 without gaps and a complete final LF-terminated record. Do not compare
the excerpt CRC to a CRC of the original full download. No screenshots or transfer END
were supplied; this session's download integrity/resume/Stop cannot be independently
validated from the excerpt. No repeat transfer requested for the symptom analysis.

Boot 42: power_on, context append; build stage3-context, compiled Sep 24 2026 21:02:06.
Exact flashed commit is not encoded. Clock sync 08:46:19.584, up_ms=17505; last record
HTTP_GET_BEGIN at 08:59:23.910. All times below are synced local -04:00. No reboot within
the supplied session. Earlier historical boots are deliberately absent.

##### Disconnect and spinner timeline

| Local time | Measured event |
|---|---|
| 08:51:21.365 | WiFi beacon_timeout, last RSSI -29 dBm; MQTT loss follows 10 ms later |
| 08:51:21.581 | Connection UI red; screen 1 remains active |
| 08:51:45.672 / 46.795 | Association restored / IP acquired after no_ap_found retry events |
| 08:51:46.898-55.557 | MQTT attempt 11 fails, state=-2, elapsed 8659 ms |
| 08:51:55.559 | Main-loop gap 8771 ms: connect 8659 + other 112 ms |
| 08:51:55.573 | Connection UI orange (WiFi up, MQTT down) |
| 08:52:10.660-15.665 | Attempt 12 fails, state=-2, elapsed 5004 ms |
| 08:52:15.667 | Main-loop gap 5115 ms: connect 5004 + other 111 ms |
| 08:52:17.686 / 22.377 | Another beacon timeout / IP restored |
| 08:52:30.517 / 34.123 | Third beacon timeout / IP restored |
| 08:52:34.229-35.100 | Attempt 13 succeeds in 870 ms; subscriptions and motion publish resume |
| 08:52:35.125 | Connection UI green |

Initial WiFi loss to MQTT recovery: 73.735 seconds. This is not a continuous 74-second
UI freeze: it contains retry time with loop service, plus the two measured blocking calls.
Screen 1 is recorded before, during and after the episode. Source still services LVGL
from the main loop and performs synchronous MQTT connect there; this strongly explains
why the spinner can animate while disconnected but freeze during reconnect calls.
The log measures main-loop gaps, not individual spinner frames or a physical observation
of the exact freeze boundaries. No G-meter/IMU average-rate investigation is implied.

Unlike F001, the long attempts FAIL. Repeated blocking MQTT attempts while WiFi is
associated are now observed, rather than only predicted from source. This does not prove
cellular internet was absent: DNS/TCP/TLS/CONNACK are not separately timed. state=-2 and
tls_code=-1, tls_fresh=unknown do not identify the failed phase. Last RSSI -29/-30 and
beacon_timeout do not establish the hotspot outage cause. Retry disconnect records,
including suppressed duplicates, are not counted as additional distinct full outages.

##### Storage, memory, and later events

- New finding I004: slow counter rises to 1 at 08:51:47.024 and 2 at 08:51:55.569.
  Next HEALTH records write_max_us=114893 and flush_max_us=122353; both exceed the
  100000 us threshold. Source counts BOTH writes and flushes in slowWrites. These are
  two slow operations, not two write failures. Wall time can include scheduling delay;
  the log does not isolate physical card latency. They are much shorter than the loop
  gaps, whose measured MQTT spans already explain nearly all elapsed time.
- Across all 13 HEALTH records: drops=0, truncated=0, queue_high=7/16; slow stays 2
  afterward. SD max stays 165519 us. No persistent storage error or record loss shown.
  Writer stack minimum 3592, internal minimum 34148, sampled internal largest minimum
  31732, DMA largest minimum 26612: all relevant largest-block samples exceed 20480.
- Earlier Live completes 145 frames in 60350 ms (~2.40 FPS), with a separate 1514 ms
  main-loop gap attributed to live_connect (1340 ms) at 08:47:54.847. No paired bench
  comparison or regression conclusion is justified by this ride alone.
- A later, distinct episode has POWER_USB present=0 at 08:58:41.632, WiFi auth_expired
  at 08:58:49.763 and Live connection_closed at 08:58:49.767 after 25 frames. MQTT retry
  is deferred until media clears, then an attempt fails in 1 ms with wifi_connection=0.
  This can reflect a state transition; not enough evidence to call it a separate bug.
  USB returns at 08:58:59.377, association changes from channel 4 to 6, IP returns
  08:59:01.657 and MQTT reconnects in 595 ms at 08:59:05.701. Last-result retrieval mode
  starts at 08:59:14.955 and reaches ACTIVE. User circumstances remain unconfirmed;
  do not assume ignition-off or hotspot toggling caused this sequence.
- An image succeeds at 08:58:37.609 after the main outage, before the later Live failure.

#### Claude review - September 25, 2026

Spot-checked against the raw excerpt: SHA256 and CRC32 46050290 match; the timeline, both
loop gaps, the 73.735 s recovery and the 08:58 sequence are as stated. Agree with the
disposition and with not treating the excerpt as download-integrity evidence.

Additions and one refinement:
1. **state=-2 narrows the failed phase.** In PubSubClient, -2 (MQTT_CONNECT_FAILED) means
   the network `connect()` itself failed; a missing CONNACK returns -4. So attempts 11 and
   12 failed in DNS, TCP or TLS, not in the MQTT exchange. Attempt 12's 5004 ms matches a
   single 5 s stage limit (TCP connect or TLS handshake). Attempt 11's 8659 ms exceeds any
   single limit, so at least two phases consumed time, as did F001's 7.9 s successes.
   Unmeasured DNS time remains the leading candidate for the extra ~3-4 s (hypothesis).
2. **The failures were on a flapping link, not a stable link without internet.** Attempt
   11 began 1.2 s after re-association, and attempt 12 ended 2 s before the next beacon
   timeout, within three losses in 70 s. This is not yet the "WiFi stable, no internet"
   case derived in F001, although the UI effect is the same kind of repeated block.
3. **The two slow storage operations bracket connect attempt 11.** slow_total=1 is logged
   126 ms after the connect began and slow_total=2 12 ms after it ended. Given this, CPU
   contention on core 1 during the connect is at least as likely as card latency. The
   loop task and SD writer share core 1 at priority 1. F001's slower successful
   connects produced no slow operations, so this is weak evidence. Hypothesis; I004 stays
   "monitor".
4. **The 08:58 episode was JP's own intervention** (correction, September 25, after JP
   confirmed it). JP switched the hotspot off manually and turned the car off by mistake,
   then restarted it to download the log. That explains the USB loss (08:58:41.6 to
   08:58:59.4), the auth_expired disconnect and the channel change (4 -> 6) when the
   hotspot came back. My earlier start-stop / shared-USB hypothesis is withdrawn. This
   episode is not a field fault and should not count toward I002 or I003. The Live
   connection_closed and the 1 ms failed MQTT attempt are expected consequences of it.

No disagreement with Codex's conclusions. P002 priority position unchanged (see F001).

#### Agreed findings and unresolved questions

Codex integrated Claude's review September 25 while preparing P001. F001 cross-review
remains intact. Accepted refinements: F002 state=-2 excludes the CONNACK phase; failure
was during a flapping link, not demonstrated stable WiFi without internet. The 08:58
power/hotspot sequence was JP's confirmed intervention and is excluded from field-fault
counts. SD scheduling contention remains a hypothesis, not a card-latency diagnosis.
A 5004 ms duration is consistent with a five-second transport limit, but wall duration
alone does not prove how many phases consumed time; requested phase timing will resolve it.
F002 supports I001/P001 with failed as well as successful reconnect blocking. I002 remains
unexplained. I004 is a bounded storage-latency observation to monitor, not a diagnosed
cause of the UI freeze. P002's priority disagreement is unchanged; no code is approved.

Review warranted because JP observed a symptom and failed long attempts/slow storage
are new field evidence. Suggested Claude focus: timeline and 73.735 s recovery duration,
failed-connect versus no-internet distinction, storage counter interpretation, and the
excerpt's integrity limits. Append one concise Claude review per workflow, not a second
full analysis. Next field input can be another normal ride log; no extra bench case,
build or flash requested. Keep original untrimmed exports in future if available, and
label any excerpts separately, so phone-download CRC verification remains possible.

## September 25 - P001 design handoff

JP requested the design now, covering successful/failed recovery, F002 link flapping,
TLS memory and the 20480-byte gate, with bounded DNS separate from TCP/TLS/CONNACK timing.
Codex authored [P001 revision 1](p001_mqtt_responsiveness_design.md). It recommends an
exclusive MQTT worker, fixed cross-task messaging, split measured connection phases,
absolute MQTT packet deadlines and resource admission. Main-thread UI/IMU responsiveness
is measured independently from worker duration. Several behavior trade-offs are explicit
approval items. Claude reviews this design before JP approves implementation. No firmware
changes, tests, builds or flashes accompany this documentation. Historical analyses above
retain their original uncertainty; the integrated corrections and latest status govern.

### P001 revision 2 - review integration

At JP's request, Codex integrated Claude review 8daed34 into P001 revision 2. Required
vTaskDelay polling protection and its host check ship with increment 1. DNS wait is
15 s, attempt budget 35 s, worker-stuck threshold 40 s; socket timeout remains a constant
5000 ms. Media/retrieval lease starts only before TCP setup; shared admission includes
USB as well as panel refusal. DNS callback lifetime and both ERR_MEM paths are explicit.
The first hardware check after increment 1 is one TLS handshake/CONNACK on the worker.
The obsolete 3.1.3 capability branch is removed. Remaining JP decisions are enumerated
in design section 11. Design only; focused B1/B2 check and implementation approval remain
pending. No code, build, flash or host execution accompanied this documentation change.

### P001 increment 1 — September 25, 2026, Codex

JP approved all four revision-2 decisions and increment 1 following Claude focused
review e3ae528. At JP's request, development moved to `codex/car-improvements-p001`,
branched at that checkpoint. `iphone-log-retrieval` stays at the return point.

Implemented the permanent MQTT owner, local PubSubClient polling/deadline patch,
bounded DNS, split hostname-verified TLS, post-DNS lease, cancellation/cleanup,
main-only callback dispatch and minimal phase/memory/stack reporting. See
[handoff](p001_increment1_handoff.md). All 14 host suites pass: 329 checks. These are
source-body simulations and integration assertions, not compiled firmware or hardware
validation. No build, flash, card access or field-evidence change. Claude code review
is next; the first hardware gate remains one worker TLS handshake/CONNACK. Increment 2
service-gap telemetry and failure/flap validation have not been implemented.


### P001 increment 1 focused review fixes — September 25, 2026, Codex

Addressed Claude review e51ef5a at JP's request: bounded oversized-packet discard that
keeps the session, 10 ms idle/ONLINE cadence, phase/1 Hz stack sampling, busy-only heap
sampling, buffered-body batch yields, duplicate GOT_IP filtering, and counted stale RX
drops. All 14 host suites pass, **341 checks**. See the [focused handoff](p001_increment1_handoff.md)
for changes and review scope. Claude re-check is pending; no build, flash or hardware
results. The car installation and raw field evidence remain unchanged.


### P001 first worker TLS/CONNACK bench result — September 25, 2026, Codex

JP compiled/flashed after Claude clearance c9629df. Boot 130 connected successfully:
DNS 77 ms, TCP setup 91 ms, TLS 445 ms, MQTT exchange 50 ms, total 696 ms.
Worker stack margin 7260 bytes, PSRAM placement and internal TCB confirmed; attempt
internal-largest minimum 53236 bytes exceeds 20480. Subscriptions accepted, green
connection recorded, no reported drops/faults. JP confirms the reset preceding boot 130
was caused by closing/switching the VS Code monitor, the known separate limitation.

The technical placement/connection/resource gate passes. This does not yet prove
reconnect responsiveness or failure/flap behavior. JP acceptance / increment 2 approval
pending. Full evidence, hashes and limitations are in the
[handoff result](p001_increment1_handoff.md#p001-increment-1-first-hardware-gate--september-25-2026-codex).
The original 57,256-byte USB export is preserved under
`evidence/2026-09-25-p001-first-handshake/130-current.log` (ignored by Git).
