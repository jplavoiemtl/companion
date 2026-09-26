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
- **MQTT responsiveness issue I001 addressed by P001:** owned MQTT worker and bounded
  timing/service telemetry implemented, reviewed and bench-validated. JP accepted
  increment 2 and all three retained cases on September 25. F003 now contains car
  worker/SERVICE records from firmware compiled September 25 at 11:28:38; exact flashed
  Git identity is not embedded.
- The accepted bench covers successful reconnect, two five-second TCP failures and
  recovery, and hotspot loss during a pending attempt with same-IP recovery. G-meter
  remained responsive; maximum reconnect UI gap 27 ms, IMU/loop gap 26 ms, no drops
  or resets during the cases. Stack and 20480-byte memory gates passed.
- **F003 supports P001 in the field:** all 14 post-startup MQTT attempts in afternoon/
  evening boots kept UI gaps <=23 ms and IMU/loop gaps <=25 ms, including failures.
- **WiFi loss (I002) remains open:** nine afternoon/evening beacon-timeout episodes.
  A separate 59.209 s MQTT outage near 20:26 occurred without recorded WiFi loss (I005).
  Image requests still block main for up to 5.264 s (I003); P004 is proposed, not approved.
- **P005 A+B with P006 implemented for review:** JP approved design revision 2 after
  Claude clearance 2d93c24. The combined [implementation handoff](p005_p006_implementation_handoff.md)
  records 379 passing host checks, Claude clearance 271f94b, and two passing retained
  bench cases on JP's build (boot 136). JP explicitly accepted P005/P006.
  P004 design is now written for Claude review and JP approval; no implementation
  authorized. No build or flash performed by Codex.
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
| I001 | UI pauses during MQTT recovery | F001: two ~8 s G-meter gaps; F002: 8.771 s and 5.115 s gaps on screen 1 during failed connects, matching spinner symptom | Addressed by P001, accepted by JP. F003 now supports field responsiveness: UI <=23 ms, IMU/loop <=25 ms during post-startup attempts |
| I002 | Hotspot link loses beacons | F001: two episodes; F002: three beacon-timeout losses in a 73.735 s MQTT recovery episode | Open: F003 adds nine afternoon/evening beacon-timeout episodes at reported RSSI -31 to -42 dBm. Cause unproven; separate power-associated interruption |
| I003 | Other image/Live latency | F001 image/Live gaps; F002 1.514 s Live-connect loop gap. Later Live disconnect was JP's intervention, not a field fault | Open: F003 image-request main-loop gaps reach 5.264 s with two failed requests near 20:26. Proposed P004, separate from MQTT recovery |
| I004 | Occasional slow storage operations | F002: write 114.893 ms, flush 122.353 ms; slow counter 2, zero drops/truncation | Monitor: F003 write/flush maxima include 118.747/229.690 ms with zero drops. No evidence these explain multi-second network spans |
| I005 | MQTT/TLS outage while WiFi stays associated | F003 boot 48: 59.209 s observed MQTT outage, two ~5 s TLS failures, no driver WiFi disconnect | Open: path/broker/transport cause unresolved; association is not proof of working internet |

Do not count retry no_ap_found/sta_leaving records as separate full outages without
checking the timeline. Do not attribute unknown-freshness TLS errors to a current TLS
failure. Health snapshots can be stale while the main loop is blocked.

## Possible improvements and decision gates

### P001 - Keep UI and IMU responsive during MQTT reconnection

Status: **implemented, reviewed, bench-validated and accepted by JP on September 25.**
Increment 1 and increment 2 are accepted. [Increment 2 closeout](p001_increment2_handoff.md)
records the three retained passes. Source checkpoint a83c96a is the bench-tested firmware;
subsequent commits through acceptance contain documentation only. Development branch:
`codex/car-improvements-p001`. No additional corrective implementation or bench matrix
is indicated by the MQTT bench results. F003 subsequently establishes P001-era car
deployment and responsive field recovery; the exact flashed Git hash is not embedded.

The owned worker keeps MQTT connection waits off the UI/IMU main task. Failed, cancelled
and successful attempts preserve cleanup, epoch checks, subscriptions and resource
admission. Separate DNS/TCP/TLS/MQTT timing and per-attempt UI/IMU/loop service measurements
now make field validation possible. Existing main-loop gap records remain in place.

Measured bench result: maximum UI service gap 27 ms, IMU/loop gap 26 ms across retained
reconnect windows, no intervals over 100 ms; JP saw no G-meter freezes. Worker stack
minimum 7228 bytes, recovery internal-largest minimum 51188 bytes; retained media/logger
largest minimum 24564, all above the gates. Subsequent Latest and USB export worked.
These observations address I001 on the bench; they do not prove all future field behavior
or explain WiFi losses. The earlier field firmware's synchronous MQTT call produced the
multi-second stalls in F001/F002.

F003 now supplies field recovery evidence: 14 post-startup attempts include long
successful and failed connections with bounded UI/IMU service gaps. Continue full exports
and symptom times, distinguishing media stalls and network outage from MQTT UI blocking.
Increment 3 is evidence-justified corrections if needed and field rollout; no code change
is proposed now. The known VS Code monitor-close reset limitation remains separate.

### P002 - Add targeted timing detail only if needed

Status: delivered with P001 and accepted by JP September 25. Split phase timing and
service telemetry were exercised in the three retained bench cases and F003 field
reconnects. The 20:26 episode now separates DNS, TCP setup and failed TLS. Historical
priority disagreement below is retained for provenance.
The new design includes bounded DNS, TCP setup, TLS handshake and MQTT-exchange timing;
no separate instrumentation-only flash is proposed.

F002 update (Codex, September 25): failed connect durations vary (8659 and 5004 ms),
then recovery takes 870 ms. A universal fixed 7.9-second wait is not supported across
rides. Phase attribution remains unknown; this neither proves nor rules out DNS waits.
The existing P002 priority disagreement is preserved pending review/JP decision.

Historical pre-P001 rationale: the field log identified the blocking MQTT span but did
not separate DNS/TCP/TLS/CONNACK timing. Consider bounded phase timing only if needed to validate
P001; avoid high-volume per-frame or per-sample records. Do not claim such measurements
exist today. A second useful measure could be UI/IMU service gaps, with a defined metric
and bounded reporting, if loop gaps alone cannot validate the selected design.

### P003 - Investigate recurring hotspot loss from field patterns

Status: observing field data; no WiFi-policy change approved.

Collect time, recovery duration and known phone/power circumstances across sessions.
The first ride does not identify weak signal, cellular handover or iPhone behavior as
the cause. Change WiFi policy only when repeatable evidence supports it; do not make
speculative power-save, retry or radio changes while diagnosing UI blocking.

#### Investigation plan - Claude, September 25, 2026 (requested by JP; no changes made)

What F001/F002 already show (5 beacon_timeout drops):

| Ride | Drop, time into ride | RSSI | Channel before -> after | AP invisible (no_ap_found) |
|---|---|---|---|---|
| F001 | 07:04:12, ~15 min | -35 | 6 -> 6 | ~2.5 s |
| F001 | 07:04:36 | -41 | 6 -> 6 | ~12 s |
| F002 | 08:51:21, ~5 min | -29 | 4 -> 4 | ~24 s |
| F002 | 08:52:17 | -30 | 4 -> 4 | ~2.5 s |
| F002 | 08:52:30 | -30 | 4 -> 4 | ~2.5 s |

- Signal was very strong at every drop, so range is not the cause.
- ESP32 power save is already off (`WiFi.setSleep(false)`, companion.ino:790). The
  module is always listening, so missed beacons are not due to modem sleep.
- There was no channel change: each drop recovered on the same channel.
- After each drop the module actively scanned and could not see the hotspot for
  2.5-24 s. Combined with the points above, this points to the iPhone pausing its
  hotspot radio rather than the module losing reception. Strong indication, not proof:
  the module only sees its own side.
- Drops cluster (2 within 24 s; 3 within 70 s), then the link stays stable for the
  rest of the ride.

Plan, in priority order:

1. **Second client on the same hotspot (decisive; no firmware change).** During a ride,
   connect a laptop or iPad to the iPhone hotspot with a continuous ping (for example
   `ping -t 172.20.10.1` on Windows) and compare ping failures with the module's drop
   times.
   - Both drop together: the iPhone is pausing its hotspot. The module cannot prevent
     that; P001 improves the recovery.
   - Only the module drops: investigate the ESP32 side.
2. **Record phone circumstances for each drop,** for P003:
   - leaving home Wi-Fi range (fits "a few minutes into a ride"; the iPhone switching
     off home Wi-Fi may restart the hotspot)
   - wireless CarPlay or Bluetooth audio connecting (they share the iPhone's radio)
   - navigation, calls, screen lock, Low Power Mode
3. **Keep collecting normal ride logs.** Per drop, tabulate time into the ride, time the
   hotspot was invisible, channel and RSSI, to see whether drops always come early (which
   would support the home-Wi-Fi hypothesis) or at random.
4. **Only if 1-3 are inconclusive (a firmware change, needs approval):** sample RSSI
   about once a second (bounded) before drops, to tell a gradual fade from an instant
   disappearance. An instant disappearance would confirm the AP switching off.

This plan changes no WiFi policy. Per the workflow, Codex integrates any resulting
finding into the tables.

**Update, September 25 (JP):** a second client is not available now, so step 1 is
replaced by a stationary test, alongside more ordinary rides:

- **A. Driveway, within home Wi-Fi range, stationary, 20-30 min.** Module on the hotspot
  as usual. Note whether the iPhone's own Wi-Fi shows the home network.
- **B. Parked outside home range, stationary, 20-30 min.**
- **C. Transition.** Start in the driveway, drive away, and note the time you leave home
  range.

Interpretation:
- Drops only in C, shortly after leaving: the home-Wi-Fi hand-off restarting the hotspot
  is the likely cause.
- Drops in A or B while stationary: iPhone hotspot behaviour independent of movement and
  home Wi-Fi. Suspect phone settings next (Low Power Mode, CarPlay/Bluetooth, screen
  lock).
- No drops in A or B, but drops while driving: movement or cellular hand-offs.

Keep the phone state constant within each test. Note start and end times and any phone
activity. Keep the module powered continuously and export the log afterwards; the log
records every drop and its timing. Evidence goes in a new folder under `evidence/` as
usual.

## Working agreement

Keep the accepted reduced bench scope. Reopen a waived case only for relevant evidence
or a code change. JP performs builds/flashes; firmware changes go through Claude review.
No new IMU average-rate or writer-core investigation without new relevant symptoms.
The next useful field input is a timestamped symptom plus a later log export, not an
expanded test suite. Deployment and successful retrieval do not close I001 or I002.

### P004 - Keep UI/IMU responsive during image-request setup

Status: JP authorized design work after accepting P005/P006. Codex's
[P004 revision 1](p004_image_responsiveness_design.md) is ready for Claude review
and JP approval; **implementation remains unapproved**.
F003 records main-loop gaps of 5.192 and 5.264 s around synchronous image requests,
plus shorter image/Live setup gaps. Recommend a focused design review of image HTTP/TLS
setup, preserving shared media ownership, Live/handover, cancellation, memory and logging
constraints. Do not blindly reuse the MQTT worker or move LVGL across threads. This
would preserve responsiveness during slow network work, not necessarily shorten remote
response times or prevent outages. No firmware change or new bench campaign now.

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

### P001 increment 1 � September 25, 2026, Codex

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


### P001 increment 1 focused review fixes � September 25, 2026, Codex

Addressed Claude review e51ef5a at JP's request: bounded oversized-packet discard that
keeps the session, 10 ms idle/ONLINE cadence, phase/1 Hz stack sampling, busy-only heap
sampling, buffered-body batch yields, duplicate GOT_IP filtering, and counted stale RX
drops. All 14 host suites pass, **341 checks**. See the [focused handoff](p001_increment1_handoff.md)
for changes and review scope. Claude re-check is pending; no build, flash or hardware
results. The car installation and raw field evidence remain unchanged.


### P001 first worker TLS/CONNACK bench result � September 25, 2026, Codex

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


### P001 increment 2 � September 25, 2026, Codex

JP accepted increment 1 and requested essential-only testing, then authorized increment 2.
Implemented bounded per-attempt main UI/IMU/loop service records, overlapping-operation
context, DMA-largest memory sampling and normal worker phase/age/backoff health snapshots.
355 host checks / 15 suites pass. See [review handoff](p001_increment2_handoff.md).
No firmware build/flash or additional hardware test. Three retained bench cases only:
successful reconnect, failed attempts/recovery, and a pending-attempt hotspot flap; one
at a time after review clearance. Actual failure/flap evidence remains pending. Car
rollout follows acceptance and only evidence-justified corrections.


### P001 retained case 1 � September 25, 2026, Codex

PASS: JP reports no G-meter freeze. Boot 132 real recovery took 888 ms; measured maximum
UI/IMU/loop service gaps were 23/26/26 ms with zero intervals over100 ms. A preceding
five-second TCP wait was cancelled by serial on; gaps remained 27/23/23 ms. This adds
long-wait/cancellation evidence but does not replace failed-attempt or WiFi-flap cases.
Worker stack 7228, attempt largest 51188; retained media/logger largest 24564, all above
gates. No drops/reset during the case; Latest displayed after recovery. See the
[increment 2 handoff](p001_increment2_handoff.md) for timing, hashes and qualifications.
Evidence is preserved in `evidence/2026-09-25-p001-case1/`. Cases 2 and 3 remain pending.


### P001 retained case 2 � September 25, 2026, Codex

PASS. JP observed no G-meter freeze. Boot 132 attempts 4/5 each failed after 5003 ms TCP
setup, with maximum UI/IMU/loop gaps 21/21/21 ms and no over100 intervals. Retry began
15004 ms after first result adoption. Real recovery took 1087 ms; gaps 21/20/20 ms.
USB retrieval admission correctly refused `mqtt_reconnecting` while the lease was held.
Worker stack 7228 and attempt largest minimum 51188 passed; no new reset, cancellation,
drops or stuck worker. Full evidence/hashes are in the [increment 2 handoff](p001_increment2_handoff.md)
and `evidence/2026-09-25-p001-case2/`. Only the retained hotspot-flap case remains; no
controlled failure endpoint or extra failure tests are needed from this result.


### P001 retained case 3 / bench closeout � September 25, 2026, Codex

PASS: JP observed responsive G-meter. Driver WiFi loss at up_ms=785205 overlapped attempt
7; that epoch ended cancelled. Same-IP GOT_IP (172.20.10.2, changed=0) preceded the next
attempt. A further test attempt was cancelled by serial restore; real epoch 43 then
connected in 613 ms with UI/IMU/loop gaps 16/18/18 ms. No stale epoch reported connected,
no reset/drops/stuck worker; stack and memory gates passed. Full results/hashes are in
[increment 2 handoff](p001_increment2_handoff.md); evidence is retained under
`evidence/2026-09-25-p001-case3/`.

All three agreed essential cases pass. Maximum measured reconnect UI gap 27 ms,
IMU/loop gap 26 ms, no over100 intervals. No additional firmware correction or bench
case is proposed. JP acceptance and approval for car rollout are pending. Next evidence
would be one ordinary car ride with full export; field WiFi-loss causes remain open,
and no-outage driving alone cannot validate field reconnection behavior.


### JP acceptance and car-testing handoff � September 25, 2026, Codex

JP explicitly accepted increment 2 and will conduct car testing and report findings.
I001 is now **addressed: implemented, reviewed, bench-validated and accepted**, with field
confirmation pending. P001/P002 current statuses above are updated; dated prior entries
remain as historical checkpoints. All three retained cases passed; no further bench
case or code change is requested. The next evidence is a full ride export with symptom
and reconnect times if observed. I002 WiFi-loss cause remains unresolved. No claim is
made that the car has already been flashed with P001.


### F003 - September 25 afternoon/evening rides; MQTT outage around 20:26

Author: Codex, September 25, 2026. Analysis complete; Claude review recommended because
this is P001's first field confirmation and stronger evidence of a separate media issue.
No firmware changes, builds or flashes. Raw evidence read in place, unchanged.

#### Observation, integrity and coverage

JP reports additional rides from about 13:00 and an MQTT disconnection lasting many
seconds around 20:26. No explicit UI-freeze observation was supplied for these rides.
Phone location/power actions and which intervals were parked versus driving are unknown.

File: `evidence/2026-09-25-ride-3/start-unknown_49-1-current-524908.log`.
524908 bytes (matches filename), 2491 lines; SHA-256
`3d1e989fbda022250ae3ecd924c6e62de878fdce134dded67297bf7770b759cb`;
computed CRC32 `638048F8`. These are local integrity fingerprints, not a comparison to
an independently supplied device END/phone CRC. No screenshots/last-result supplied.
Last record: boot 49 seq146 HTTP_GET_BEGIN, 20:43:14.378; a current snapshot normally
excludes its own transfer END/CLOSE. No extra export is required just for that reason.

Cumulative log starts at FILE_OPEN boot 39, clock unknown, and includes boots 39-49.
Boots 39-43 embed firmware Sep 24 21:02:06: do not count their older stalls as a P001
regression. Boots 44-49 embed Sep 25 11:28:38 and contain worker/SERVICE records,
establishing P001-era deployment, not an exact Git hash. Every boot 44-49 has contiguous
record sequences. Boot 44 is supplemental pre-ride data. Afternoon/evening analysis
below uses boots 45-49. There is no synchronized 13:00 record; coverage is not invented.
Synced wall times are local -04:00; durations below use same-boot up_ms. Unknown-clock
startup is not assigned an exact wall time by extrapolation.

| Boot | Synced coverage | End |
|---|---|---|
| 44 | 12:05:08-12:08:57 | Clean shutdown; supplemental before reported rides |
| 45 | 15:23:26-15:45:35 | Clean shutdown |
| 46 | 16:05:32-16:25:49 | Clean shutdown |
| 47 | 18:22:17-19:31:49 | Clean shutdown |
| 48 | 20:23:07-20:28:58 | Clean shutdown; reported outage |
| 49 | 20:35:53-20:43:14 | Active at export |

These are powered-session windows, not measured driving durations. Boots 45-49 report
power_on; no watchdog reset boot in this scope. Boot 44 code 11 is labelled other;
its initiating cause is not inferred.

#### Reported outage: boot 48, 20:26

MQTT_LOST at **20:25:59.818**, MQTT_CONNECTED at **20:26:59.027** = **59.209 s**
(up_ms 238376 to 297585). The physical transport may have failed earlier while main
served an image: main observation is not a precise remote-disconnect timestamp.
Orange UI 20:26:00.697; green 20:26:59.884. No WIFI_DISCONNECT during the episode;
WiFi health remains associated, RSSI -29 to -37 dBm around the incident.

| Time | Event and measurement |
|---|---|
| 20:25:32.005-37.092 | Latest fails, 5087 ms image network span; LOOP_GAP 5192 ms |
| 20:25:40.274-42.143 | Latest succeeds, 1867 ms request span; LOOP_GAP 1969 ms |
| 20:25:49.597-51.279 | Back succeeds, 1679 ms request span; LOOP_GAP 1784 ms |
| 20:25:54.516-59.677 | Back fails, 5157 ms request span; LOOP_GAP 5264 ms |
| 20:25:59.818 | MQTT_LOST state=-3; no corresponding WiFi loss |
| 20:26:02.006-03.092 | Back succeeds; temporary media deferral fits inside existing retry wait |
| 20:26:14.822-22.863 | Attempt 2: TLS failure; DNS/TCP/TLS 1/3022/5003 ms, total 8042 ms |
| 20:26:37.871-43.001 | Attempt 3: TLS failure; DNS/TCP/TLS 1/112/5004 ms, total 5131 ms |
| 20:26:58.009-59.020 | Attempt 4 succeeds; DNS/TCP/TLS/MQTT 224/265/458/44 ms, total 1011 ms |

About 45 s is three configured ~15 s waits (initial delay plus post-failure backoffs);
about 14.2 s is connection work/transition overhead. Media clears before the first retry
is due, so that short deferral does not add delay beyond the normal wait. No evidence of
an unbounded/stuck worker. Shortening backoff is an availability/load tradeoff, not an
automatically warranted fix on this evidence.

TLS failures align with the ~5 s bound, but error=0/error_fresh=0 does not diagnose their
cause. Fast DNS rules out DNS as the major delay in these two attempts. TCP success and
WiFi association do not prove working internet/TLS/broker service. State=-3 identifies
lost connection, not which endpoint/path component caused it. Nearby HTTPS failures
suggest broader path or endpoint difficulty (inference only); they do not prove a
cellular outage, common cause, or that MQTT loss was caused by image requests.

P001 worked during the retry windows: SERVICE UI/IMU gaps were **17/17, 17/17, 19/19 ms**;
loop gaps 17/18/18 ms, all over100 counts zero. These windows exclude the earlier image
stalls. Post-recovery subscriptions accepted (SUBACK unobserved); later inbound power/
energy counts advance. The near-minute disconnection is not a near-minute UI freeze.

#### Other network episodes

Nine driver beacon_timeout losses in boots 45-49 had reported RSSI -31 to -42 dBm.
Strong event/last RSSI does not exclude interference/missing beacons or prove the phone
paused its radio. Repeated no_ap_found/sta_leaving during recovery are not new outages.
Observed MQTT loss-to-adopted-recovery intervals:

| Boot | Loss | Recovery | Duration | Context |
|---|---|---|---:|---|
| 45 | 15:27:16.789 | 15:27:41.943 | 25.154 s | Beacon loss; successful attempt 10.141 s |
| 45 | 15:28:34.614 | 15:29:13.140 | 38.526 s | Beacon loss; TLS failure then success |
| 45 | 15:39:16.517 | 15:39:42.479 | 25.962 s | Beacon loss; successful attempt 10.958 s |
| 46 | 16:09:57.598 | 16:10:17.685 | 20.086 s | Beacon loss |
| 46 | 16:11:05.588 | 16:11:21.236 | 15.648 s | Beacon loss |
| 47 | 18:27:32.568 | 18:27:54.800 | 22.232 s | Beacon loss |
| 47 | 19:11:35.880 | 19:12:00.394 | 24.514 s | Beacon loss |
| 47 | 19:12:21.749 | 19:13:09.006 | 47.257 s | Beacon loss; 5003 ms TCP failure then recovery |
| 48 | 20:25:59.818 | 20:26:59.027 | 59.209 s | WiFi stays associated; two TLS failures |
| 49 | 20:41:30.045 | 20:41:48.189 | 18.143 s | Beacon loss |

Separate power-associated event: boot 46 POWER_USB=0 at 16:25:19.362, then Live
connection_closed and WiFi auth_expired/MQTT loss at 16:25:24, shutdown 16:25:49. No
recovery before shutdown. Actual phone/car action not reported; do not group it with
unexplained beacon loss. Boot 44 adds two pre-13:00 beacon losses (12:07:27.998 and
12:08:30.365); the first recovers, the second ends in shutdown. Excluded from count nine.

Boots 45-49 have 19 worker attempts: five startup and **14 post-startup**, comprising
ten successes and four failures (three TLS, one TCP). Across all 14 recovery SERVICE
windows: **UI gap <=23 ms, IMU/loop <=25 ms, all over100 counts zero**. Long successful
attempts span 7-11 s with delays spread across DNS/TCP/TLS/MQTT, not one universal failing
phase. At 15:39 the successful MQTT exchange took 4986 ms; reducing timeout could reject
such legitimate slow success.

Startup differs: boots 46/48 have ~222-223 ms UI/IMU gaps, loop_n=0, with wifi_setup
context. Boot 44 initially returns lease_deferred with similar spacing then succeeds.
Setup's fixed-delay background paths (including 200 ms in initWiFi) are consistent with
this spacing; exact per-gap causality is not measured. Record this limitation, not a
recurring multi-second worker regression or a reason for a new test matrix.

#### Other findings and limits

- Image/Live: 11 explicit >1 s LOOP_GAP records across boots 45-49, all attributed to
  image_request/live_connect; one also carries suppressed=1. Eleven records are not
  all possible gaps. Maximum 5.264 s, plus two code=-1 image failures with TLS error
  freshness unknown. Supports I003/P004 independently of MQTT blocking.
- MQTT MEM minima: stack 7228, internal/DMA largest 47092 bytes, above 2048/20480 gates.
  HEALTH retained DMA-largest reaches 21492 in boot 47, only 1012 above gate; media
  headroom is narrower than MQTT's. These are sampled minima, not guarantees about
  unsampled transients. No allocation-failure event found.
- All 124 afternoon/evening HEALTH records: drops=0, truncated=0, queue high <=9.
  Boot 46 two slow operations: write max 118.747 ms, flush max 122.073 ms. Boot 48 one
  slow flush, 229.690 ms. They do not explain measured multi-second image network spans.
  Writer stack minimum 3480 in these sessions.
- Boot 46 uses MQTT profile 2 with WiFi association 1, mismatch=1 at seq44/90/114, yet
  connects and exchanges data. Profile numbers do not establish endpoint equivalence
  or intent. Flag for reviewer, not a proven defect; unrelated to boot-48 profile-1
  outage at 20:26. Do not expose credentials to investigate.
- Boot 48 initial WiFi setup took 56992 ms including a second scan/retry. Hotspot absence
  at startup is not an in-ride loss. Early wall time was unknown. Filename start-unknown
  is correct for original boot-39 FILE_OPEN, not evidence current time remained unknown.
- No supplied last-result/CRC record for this export; size matches and sequences are
  contiguous. Do not claim byte-for-byte phone/device verification beyond these facts.

#### Conclusions and review handoff

**I001/P001 is now field-supported:** long successful and failed MQTT recovery keeps
UI/IMU servicing responsive. **I002 remains open:** recurring WiFi beacon loss continues.
**I005 is new:** the reported outage is associated-WiFi MQTT/TLS disruption; normal retry
policy recovered it in 59.209 s, cause unresolved. **I003/P004** is the next concrete
responsiveness design candidate: nonblocking image request setup, subject to JP approval.

Claude should spot-check this F003 entry against the raw file, especially the 20:26
timeline, attribution separation, nine beacon episodes, SERVICE scope and memory margin,
and append a short review under the shared workflow. This is Codex analysis, not yet
cross-review consensus. No firmware edits, new bench cases or timeout/backoff/radio
changes proposed now. Continue full exports and approximate symptom times; phone/power
context may narrow causes, but no extra export is needed to establish these findings.

#### Claude review - September 25, 2026 (b2fda36)

**Spot-checked and agreed:**
- SHA-256 and CRC32 `638048F8` match; 2491 lines; sequences are gap-free in every boot
  39-49; boots 45-49 embed "Sep 25 2026 11:28:38".
- The 20:26 timeline is exact, including the three retry waits of exactly 15.0 s each.
- The 14 post-startup SERVICE windows (boots 45:4, 46:2, 47:4, 48:3, 49:1) show UI <= 23
  ms, IMU and loop <= 25 ms, with zero over-100 counts. Startup windows are rightly
  separated.
- Nine beacon timeouts at -31 to -42 dBm, each re-associating on the same channel. Eleven
  LOOP_GAP records, all image/Live. Memory and logging figures as stated.

**Additions and disagreements:**
1. **20:26, what the evidence establishes.**
   - `state=-3` comes from the transport reporting disconnected. A keepalive expiry or
     worker deadline would give -4. So the socket was closed or errored; this was not a
     silent keepalive lapse.
   - Image and MQTT use the **same configured server host**, checked by comparing the
     configuration without recording it. Both independent TLS clients hit the ~5 s bound
     (image 5087/5157 ms, MQTT TLS 5003/5004 ms) within 90 s, while image requests in
     between took 1.7-1.9 s against about 0.7 s normally.
   - That is stronger than "suggests": a degraded common endpoint or path. Cellular versus
     server-side still cannot be separated.
2. **My F001 DNS hypothesis is refuted.** DNS never exceeds 3.5 s here. Slow attempts are
   spread across phases, as Codex states.
3. **Disagreement: the 5 s phase caps are now too tight.**
   - Successful phases reached TLS 4975 ms, MQTT 4986 ms and TCP 3903 ms.
   - Failures sit exactly at the caps: TLS 5003/5004 ms, TCP 5003 ms. After the 15:28:56
     TLS failure, the next attempt succeeded.
   - Cap-edge failures are likely slow-but-working handshakes converted into a failure
     plus a 15 s wait.
   - With the non-blocking worker, raising TCP/TLS/MQTT to about 10 s costs only a
     longer media lease.
4. **Disagreement: backoff, not the network, dominates recovery.**
   - In 7 of the 9 beacon episodes, IP returned within 3.5-5.4 s (13.2 s in two, 25.2 s
     in one), yet the first MQTT attempt always began exactly 15.0 s after the loss:
     about 10-11.5 s of idle wait per episode.
   - At 20:26, 45 of the 59 s were policy waits.
   - The 15 s interval originally protected LVGL from blocking attempts. P001 removed
     that reason.
   - Proposed **P005**, for JP: attempt promptly after GOT_IP or loss, keep 15 s after
     failures, and raise the caps as in point 3.
5. **P004.** Agree it is the right *responsiveness* item: image requests are the only
   remaining multi-second main-loop stalls. But 9 of the 11 gaps are <= 1.4 s, and the
   two 5.2 s gaps are user-initiated failures. P005 is small, evidence-based and gives more
   availability per effort. Suggested order: P005, then P004.
6. **Boot 46 mismatch: cause found (pre-existing, not P001).**
   `connectToWiFi(secondary)` began the secondary profile (8.4 s), but the driver
   associated with **primary** (19.8 s). `connectToWiFi` then configures MQTT from the
   *requested* index (2), not the actual SSID; the late-WiFi path in `loop()` uses
   `WiFi.SSID()`. This is harmless in the CAR build, where both profiles use the same host
   and port (verified by comparison). In HOME builds (1883 versus 9735) it would pair the
   wrong broker profile. Low-priority fix candidate.
7. **Memory context.** The retained DMA-largest low of 21492 was set between 18:46:57 and
   18:48 in a Live session started right after Back images, with MQTT online and no MQTT
   attempt in progress. This confirms media, not P001, as the tightest consumer.


### P005/P006 design and F003 review integration - September 25, 2026, Codex

JP requested P005 A+B together with P006, then P004. Reviewed Claude's proposal
092b32c against current source, installed core 3.3.11 and raw F003 evidence. The
[combined design](p005_p006_recovery_design.md) agrees with the direction and specifies
one implementation increment, host checks and two essential bench cases. It remains
unapproved for implementation pending Claude review and JP approval.

Corrections to the proposal/review, preserving their original text: six of nine beacon
losses recovered IP in 3.5-5.4 s; two took about 13.2 s and one 25.25 s. Eight first
attempts began near 15 s, the last near 25.26 s. Removing only the first wait from
the 20:26 outage predicts about 44.2 s with identical attempt outcomes, not 30 s.
Longer TLS/MQTT allowances might help but success beyond the old caps is unproven.
Seven of eleven explicit image LOOP_GAP records were <=1.4 s, two were 1.784/1.969 s,
and two 5.192/5.264 s. These corrections do not change the agreed improvement order.

P005 retains TCP/ONLINE five-second limits, raises connect-only TLS/MQTT to ten seconds,
and permits one prompt retry after established loss; failed attempts keep 15 s backoff.
P006 selects the recognized actual SSID at initial/late configuration, deferring unknown
selection rather than treating it as secondary. No radio tuning or image changes.
The first bench case must use normal hotspot recovery: serial on already bypasses the
wait, so it cannot validate the new scheduling behavior. No bench case is issued yet.


### P005/P006 revision 2 - September 25, 2026, Codex

Integrated Claude review f5a25b7 at JP's request in the combined design. Prompt retry
now requires at least 60 seconds ONLINE, measured from real READY adoption to loss
adoption; repeated short sessions retain the normal 15-second wait. SSID selection
returns the configured network number, including reversed WIFI_PRIORITY=2 roles.
Added host-case requirements for both corrections and noted the accepted nonblocking
items (cancelled-attempt backoff, shutdown no-dispatch, and nondiscriminating bench
timing). Two bench cases remain planned, with a stable-session preparation for case 1.
Design only; awaiting Claude's focused check and JP implementation approval.


### P005/P006 implementation - September 25, 2026, Codex

JP approved revision 2 after Claude clearance 2d93c24 and authorized the combined
increment. Implemented >=60 s stable-session prompt retry, connect-only TLS/MQTT
ten-second allowances with ONLINE five-second restoration, 45/50-second attempt/stuck
bounds, and joined-SSID network-number selection shared by initial/late configuration.
The design records approval and section 9's editorial correction. All 379 checks in
16 host suites pass; no firmware compilation or hardware validation is claimed.
See the implementation handoff for review focus, counts and retained two-case scope.
Awaiting Claude code review before JP builds; no new bench procedure issued. P004
and changes to WiFi radio policy remain outside this increment.


### P005/P006 retained case 1 - September 25, 2026, Codex

JP reports responsive G-meter. Boot 136 demonstrates prompt recovery: 96.719 s ONLINE
before loss, MQTT BEGIN 11 ms after GOT_IP and 10.972 s after loss (earlier than the
old 15-second eligibility). Reconnected 750 ms after GOT_IP; worker total 730 ms.
UI/IMU/loop gaps 19/20/20 ms, stack 7228, internal/DMA largest 47092, zero drops/errors.
Joined/configured network 1 agrees. Recovery case passes; case 2 and JP acceptance
remain pending. JP confirmed the pre-test boot watchdog occurred during the serial
monitor switch, the known limitation; no reset occurred during this case. Evidence, hashes, clock caveat and
full metrics are in the P005/P006 implementation handoff, with raw copies preserved
in evidence/2026-09-25-p005-p006-case1/. No firmware changes or rebuild required.


### P005/P006 retained case 2 / bench complete - September 25, 2026, Codex

JP reports responsive G-meter. Same boot 136: hotspot loss cancelled pending id 7,
then id 8 began 15.011 s after worker END (about 15.005 s after adoption), despite IP
returning 11.146 s before that retry. JP sent on during id 8; it cancelled and cleaned
up before id 9 restored the real broker in 671 ms. This procedure variation preserves
the intended gate and also verifies restore waits for native TCP cleanup; no repeat
needed. UI/IMU/loop maxima all <=24 ms, worker stack 7228, internal/DMA largest >=51188
for these attempts, zero drops/errors/stuck faults or resets during the case.

The cumulative log also demonstrates the short-session rule: a loss after 16.919 s
ONLINE retained 15.002 s before retry. No physical cause is inferred for those earlier
transitions outside the submitted console window. Raw evidence preserved under
evidence/2026-09-25-p005-p006-case2/; hashes and precise timeline in the handoff.
Both planned cases pass. Awaiting JP's explicit acceptance, then separate P004 design;
no further testing or firmware work is requested now.


### P005/P006 accepted; P004 design - September 25, 2026, Codex

JP explicitly accepted P005/P006 after both retained cases and said to proceed.
The accepted checkpoint includes implementation 734ad92, Claude clearance 271f94b
and bench results through 02bd7bf. No additional P005/P006 bench case is required;
longer-handshake benefit remains a field measurement, not a guaranteed outcome.

Prepared P004 design revision 1 against current image/Live/source ownership. It proposes
one shared media transport worker for still and Live network operations, keeping LVGL
and decode/render on main, explicit single-buffer Live prefetch ownership, asynchronous
cleanup, preserved MQTT/retrieval exclusion and unchanged 20480-byte gate. Residual
decode/blit gaps are explicitly outside the network responsiveness target. The design
lists protocol/admission trade-offs, host checks and at most four focused bench cases.
Claude design review and JP implementation approval are next. No firmware changes,
build, flash, new measurements or bench instructions in this step.
