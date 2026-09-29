# Companion in-car field analysis and improvements

Living journal for JP's companion field modules (car and bike). Keep new field results, open findings and improvement
proposals here rather than creating a separate analysis document for every ride.
Last updated: September 29, 2026.

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
  Image requests still block main for up to 5.264 s (I003); JP deferred P004 pending
  demonstrated practical impact.
- **P005 A+B with P006 accepted:** JP approved design revision 2 after
  Claude clearance 2d93c24. The combined [implementation handoff](p005_p006_implementation_handoff.md)
  records 379 passing host checks, Claude clearance 271f94b, and two passing retained
  bench cases on JP's build (boot 136). JP explicitly accepted P005/P006.
  P004 is deferred: preserve the design, but no implementation, review or bench campaign
  is the automatic next step. Continue ordinary car use and evidence-based improvements.
  No build or flash performed by Codex.
- **F004 (September 26 snapshot):** three completed operating sessions plus the export
  boot up to its snapshot record no WiFi disconnects or MQTT losses. Startup MQTT attempts all succeeded
  in 607-1009 ms. Media delays remain measurable (5.302 s maximum explicit loop
  gap), with no symptom reported by JP; P004 remains deferred. This clean sample
  does not establish a WiFi fix or exercise P005 recovery after an established loss.
- **F005 (September 27):** JP reports stable behavior, matching about 50 minutes
  without WiFi/MQTT loss. Four full Live cycles average 2.87 FPS in aggregate.
  No reconnect occurred, so this does not validate P005 as the cause of fewer drops.
  The cumulative export adds a September 26 post-export loss after USB power removal;
  see the dated F005 addendum rather than treating F004 as a complete-day claim.
- **F006 (September 27, bike):** afternoon boot 145 connected after a startup scan
  miss/retry, then stayed connected through shutdown. One unmeasured 1.102 s loop
  gap and retained DMA-largest 19444 bytes merit review; no media/logging failure.
  Bike evidence includes older bench history, which is not counted as ride faults.
- **F007 (September 27 later car rides):** recurring beacon losses explain JP
  reporting a long outage around 12:50 (logged cluster 12:53-12:55). Eleven recovered
  MQTT outages lasted 8.548-44.329 s. P005 now has field evidence: prompt attempts
  after stable-session losses and four successful TLS phases above the old 5 s cap.
  P001 kept recovery UI gaps <=77 ms and IMU/loop <=74 ms. Radio cause remains open.
- **F008 (September 28-29, car):** JP reports no noticeable driving problems.
  New-firmware boots 68-77 exercise P007: 11 WiFi-led recovered outages, including
  five short-session losses and two cancelled attempts; recovery BEGIN follows each
  usable IP return in 2-19 ms. A separate 20.597 s associated-WiFi MQTT outage recurs
  (I005). All 15 post-startup attempts keep UI <=66 ms and IMU <=65 ms. Beacon cause
  remains open; no evidence justifies a new bench campaign or firmware change now.
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
| I001 | UI pauses during MQTT recovery | F001: two ~8 s G-meter gaps; F002: 8.771 s and 5.115 s gaps on screen 1 during failed connects, matching spinner symptom | Addressed by P001, accepted by JP. F003 now supports field responsiveness: UI <=23 ms, IMU/loop <=25 ms during post-startup attempts. F007 adds 16 recovery attempts with UI <=77 ms, IMU/loop <=74 ms, none >100 ms F008 adds 15 post-startup attempts with UI <=66 ms, IMU <=65 ms, loop <=64 ms; none over 100 ms. |
| I002 | Hotspot link loses beacons | F001: two episodes; F002: three beacon-timeout losses in a 73.735 s MQTT recovery episode | Open: F003 adds nine afternoon/evening beacon-timeout episodes at reported RSSI -31 to -42 dBm. Cause unproven; separate power-associated interruption. F004 snapshot and F005 morning have no beacon losses; F005 adds a September 26 post-export auth_expired loss after power removal. I002 remains open. F007 adds 14 powered beacon-loss events in 11 recovered MQTT outages plus a separate post-power-loss beacon episode F008 new firmware adds 13 powered beacon events in 11 recovered outages; P007 prompt scheduling is field-validated, radio cause remains open. |
| I003 | Other image/Live latency | F001 image/Live gaps; F002 1.514 s Live-connect loop gap. Later Live disconnect was JP's intervention, not a field fault | Measured, not fixed: F003 image-request main-loop gaps reach 5.264 s. JP deferred P004 until practical impact justifies added complexity; separate from accepted MQTT improvements. F004 adds a 5.302 s Live-connect gap with eventual success; no reported symptom. F005 maximum explicit loop gap 1.222 s; JP reports stable behavior F008 adds five media-attributed 1.029-1.092 s loop gaps and two 4.37/4.54 s Live frame gaps; the latter are not main-loop freeze measurements. No symptom reported; P004 stays deferred. |
| I004 | Occasional slow storage operations | F002: write 114.893 ms, flush 122.353 ms; slow counter 2, zero drops/truncation | Monitor: F003 write/flush maxima include 118.747/229.690 ms with zero drops. F004 adds two slow operations (flush maximum 124.554 ms), zero drops. F005 write/flush maxima 235.312/240.719 ms, zero drops. No evidence these explain multi-second network spans F008 new firmware adds five slow-count increments across three boots, max write/flush 118.569/233.910 ms; drops/truncation remain zero. |
| I005 | MQTT/TLS outage while WiFi stays associated | F003 boot 48: 59.209 s observed MQTT outage, two ~5 s TLS failures, no driver WiFi disconnect | Open: path/broker/transport cause unresolved; association is not proof of working internet. No recurrence in F004 snapshot or F005 morning coverage F008 boot 72: September 28 16:10:27, 20.597 s MQTT outage with no WiFi event, failed 5 s TCP setup then successful retry after 15 s. Cause remains unlocalized. |
| I006 | Reduced retained DMA-capable largest block during media | F006 bike boot 145: 19444 bytes first reported after first Live, versus previous car minima 21492; no allocation failure, periodic general-internal largest >=31732 | Review/monitor. DMA capability differs from the formal internal-largest gate; do not declare the full retained gate passed from periodic samples alone F008 reproduces retained DMA-largest 19444 in car boots 68/72/75/76, so this is not bike-specific; no allocation failure. MQTT attempt internal-largest >=47092; periodic general-largest >=28660. |
| I007 | Isolated unattributed main-loop pause | F006 bike at 15:38:00.468: 1102 ms, span=unmeasured; nearby snapshots show inclinometer, online MQTT, no storage spike | Unexplained; no corresponding user symptom reported yet. Review before assigning a cause or adding code No new unmeasured >=1 s loop gap in F008; keep the isolated bike observation open without speculative changes. |

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

Status: **deferred by JP on September 25, 2026**, after reviewing the benefit versus
complexity. [P004 revision 1](p004_image_responsiveness_design.md) is retained for
reference; no implementation, further review or bench testing is currently planned.
Measured blocking alone does not establish sufficient practical benefit. Reopen with
concrete field impact or new evidence and JP's explicit agreement, considering a narrower
solution first.
F003 records main-loop gaps of 5.192 and 5.264 s around synchronous image requests,
plus shorter image/Live setup gaps. The deferred design examines image HTTP/TLS
setup, shared media ownership, Live/handover, cancellation, memory and logging
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


### P004 deferred - September 25, 2026, JP decision recorded by Codex

JP approved deferring P004. The logs establish image-related main-loop blocking, but
JP has not identified the two F003 image waits as a practical issue requiring a fix.
A worker would keep UI/IMU service active during those waits, not necessarily shorten
network response time, and introduces substantial cancellation/buffer/Live complexity.
The expected benefit currently does not justify that complexity.

Preserve the findings and design; do not mark I003 fixed or automatically proceed with
review, implementation or its proposed tests. Continue ordinary car testing with accepted
MQTT improvements. Revisit only for demonstrated practical impact or new concrete
evidence, with JP's agreement, and assess a smaller solution first. This supersedes the
earlier planned P005/P006-to-P004 sequence. Project principle: address real problems
with a clear benefit and the smallest effective change; do not add architecture solely
because a measured delay exists. No firmware changes, build or flash.


## F004 - September 26 car sessions and cumulative export

Author: Codex, September 26, 2026. JP added today's car evidence and requested a
WiFi/MQTT review. **No symptoms reported.** Exact driving versus parked intervals,
route, and phone movements were not supplied; operating sessions below are not a
claim that the car was moving throughout. No firmware changes, build or flash.

### Evidence and coverage

Folder: `evidence/2026-09-26/`. One log, no screenshots in this submission:
`start-unknown_55-1-current-732766.log`.

- Actual length 732766 bytes, matching filename; 3460 records.
- SHA256 `62db8dafb2ab49229f8d1573b5719963c48ca9e1316ec926f3c4a0fd40df3211`.
- Computed CRC32 `09F0623B`. This is a local evidence checksum, not comparison with
  an independently supplied HTTP completion CRC.
- The complete 524908-byte F003 export is an exact prefix. Earlier boots 39-49
  are cumulative history, not new September 26 failures. New boots 50-55 have
  contiguous sequence numbers from 1; no missing sequence numbers within them.
- Boots 51-55 report compiled `Sep 25 2026 22:37:00`, consistent with the accepted
  P005/P006 bench build label. Exact flashed Git SHA is not embedded.
- All times below are synced local time, UTC-04:00. Initial boot/setup records may
  precede clock synchronization. File-open clock was unknown, hence start-unknown;
  this does not make the later synced event times unknown.

| Boot | Synced coverage, September 26 | End state |
|---|---|---|
| 52 | 13:18:47.849-13:35:58.912 | Clean shutdown, pending=0 |
| 53 | 14:17:49.704-14:41:40.440 | Clean shutdown, pending=0 |
| 54 | 16:40:12.671-17:01:46.798 | Clean shutdown, pending=0 |
| 55 | 18:55:56.214-18:56:14.827 | Current-log HTTP export begins |

The three completed sessions cover about 63 minutes of powered uptime in total.
All four September 26 boots report power_on, with no recorded watchdog/brownout.
The snapshot ends at HTTP_GET_BEGIN boot 55 id 1, as expected for current.log;
its own END/CLOSE and phone last-result screenshot are unavailable. No extra export
is required for this connection analysis.

### WiFi and MQTT results

**Zero WIFI_DISCONNECT and zero MQTT_LOST records in boots 52-55.** Each boot has
one successful startup MQTT attempt; no retries or failed attempts. All 62 periodic
HEALTH/NET_HEALTH pairs report WiFi and MQTT connected, worker online. Joined and
configured network 1 agree on all four boots. All 33 NET_END records report ok;
all eight still-image requests complete successfully.

| Boot | MQTT total ms | DNS ms | TCP ms | TLS ms | MQTT exchange ms |
|---|---:|---:|---:|---:|---:|
| 52 | 607 | 35 | 72 | 407 | 53 |
| 53 | 729 | 110 | 66 | 458 | 57 |
| 54 | 622 | 17 | 60 | 462 | 45 |
| 55 | 1009 | 53 | 209 | 653 | 56 |

Totals include work outside the measured subphases, so columns need not sum exactly.
Startup SERVICE windows show UI gaps <=28 ms and IMU gaps <=30 ms. Their loop gaps
of 610-1024 ms occur during setup (contexts=8192, loop_n=0), while UI/IMU are serviced;
these must not be reported as an MQTT-induced runtime freeze.

RSSI samples range from -24 to -86 dBm. The weak sample at 14:41:32 is after USB
power loss at 14:40:52, shortly before shutdown, with both connections still reported
up. Phone separation is plausible but unverified. Unlike F003, there is no beacon
loss today to explain. These observations support stable operation in this sample,
not elimination of the intermittent fault. They do not exercise the stable-session
prompt-retry rule or prove the benefit of the extended handshake allowances.

### Other measured behavior

- **16:43-16:44 still images:** three requests completed in 3914, 6031 and 3022 ms.
  Their header waits were 2883, 2590 and 2108 ms; the 6031 ms request also spent
  3203 ms downloading. All returned HTTP 200 and decoded successfully.
- **16:44:12 Live start:** connection succeeded after 5195 ms, with a 5302 ms main-loop
  gap recorded at 16:44:17.717. First frame arrived after 6.803 s. The cycle completed
  normally: 114 frames / 60.460 s = 1.89 FPS, maximum inter-frame gap 3.318 s.
  The other two completed cycles delivered 151/60.352 = 2.50 and 170/60.312 = 2.82 FPS.
  These are different field conditions, not a controlled performance comparison.
- There are five explicit LOOP_GAP records: three image_request and two live_connect,
  from 1.234 to 5.302 s. No concurrent WiFi/MQTT loss. A slow server or network path
  is possible; the records do not identify which. The known synchronous media path
  accounts for measured blocking, not necessarily the cause of the remote delay.
- Five Live sessions began. Three have successful duration END records. The remaining
  two (boots 53 and 54's final motion handover) run into power-related shutdown with
  no LIVE_END record. Do not classify them as network failures or completed cycles.
  Boot 53 also reports an inter-frame gap of 4.195 s near its end of session.
- Across September 26 HEALTH records: drops=0, truncated=0, queue high=7; logger stack
  minimum 3480 bytes. Two slow operations in boot 53 near 14:38:39, maximum flush
  124.554 ms; maximum write 7.200 ms and mount-inclusive SD maximum 177.205 ms.
- MQTT attempt stack minimum 7228 bytes, internal-largest minimum 51188 bytes.
  Periodic HEALTH internal-largest minimum 31732 bytes; retained DMA-largest minimum
  21492 bytes, only 1012 above the unchanged 20480 gate. Thus no recorded gate breach,
  but the media memory margin remains worth monitoring. Free-byte minima are different
  metrics and must not be substituted for largest contiguous block.

### Additional previous-night history newly available

Boot 50 has 16 unsynced records on the older 11:28:38 build; its date and termination
are not established. Boot 51 is the first record here of the 22:37:00 build and reports
reset=other/code=11, not a recorded watchdog. Its synced coverage is September 25
23:27:54-23:47:45. Its reset cause is not assigned to a user action without evidence.

Boot 51 lost WiFi with auth_expired at 23:47:14.687 and MQTT 15 ms later. USB power
went away at 23:47:15.771; no_ap_found/sta_leaving retries followed until clean shutdown
at 23:47:45.817, without IP recovery. This is a real additional late-evening interruption
close to power removal, not a September 26 mid-ride beacon timeout. The proximity does
not prove a common cause, nor does it validate P005 retry after IP restoration.

### Findings, limits and next decision

Codex findings (not yet cross-reviewed): I002/I005 remain open, with no September 26
recurrence. I001 has no new contrary evidence, but today's attempts are startup only.
I003 remains measured: slower media and main-loop blocking still occur, with successful
results and no practical symptom reported by JP. I004 remains occasional short storage
latency without loss. **P004 remains deferred under JP's explicit decision**; this
entry does not authorize a media worker or another bench campaign.

Continue ordinary car use, retaining full cumulative exports and noting approximate
local times of any noticed connection or display issue. No new firmware change or
special test is justified by this sample. Claude review is optional; if requested,
focus on the 16:43-16:45 media interpretation and the separation of boot 51 from today's
clean connection record. Original evidence remains unchanged and ignored by Git.


## F005 - September 27 morning ride, stable connections and Live FPS

Author: Codex, September 27, 2026. JP reports very stable behavior while observing
his module, no unexplained WiFi/MQTT drops, and asks whether the last software change
explains the improvement. He also requests measured FPS for the Live sequences.
No firmware changes, build or flash; no new bench case proposed.

### Evidence and integrity

One file in `evidence/2026-09-27/`, no screenshots:
`start-unknown_56-1-current-854829.log`, 854829 bytes (filename agrees), 4010 records.
SHA256 `d9332167784fb674ed3a63fa2a118589550c9f15311256929c02c9e053f6ff04`;
computed CRC32 `0FA7EB89`. This export contains the complete 732766-byte F004 file
as an exact prefix. Preserve original evidence; no deletion performed by Codex.

New coverage is boot 55's tail from yesterday and boot 56 this morning. Both boots'
sequences are contiguous (55: 1-88; 56: 1-511). Boot 56 reports power_on and compiled
`Sep 25 2026 22:37:00`, matching the P005/P006-era label but not proving an exact Git
SHA. Synced time is local UTC-04:00, from 09:00:49.003 to 09:51:11.646. Total powered
uptime at the final record is 3040114 ms, about 50 min 40 s; driving/parked boundaries
are not established by these times. Initial setup precedes CLOCK_SYNC.

The snapshot ends at HTTP_GET_BEGIN boot 56 id 1. It cannot contain its own transfer
completion and no last-result screenshot was supplied. Its hash is a local integrity
record, not independent proof of HTTP completion. No repeat download is needed.

### Connections and responsiveness

- Zero WIFI_DISCONNECT, zero MQTT_LOST, zero failed MQTT attempts in boot 56.
  One startup connection succeeds in 616 ms: DNS 14, TCP 71, TLS 450, MQTT exchange
  39 ms (total also includes unassigned overhead). No recovery attempt occurs.
- All 50 periodic HEALTH/NET_HEALTH pairs report WiFi/MQTT connected and worker online;
  no WiFi-event suppression. RSSI samples -42 to -29 dBm. Configured network 1 matches
  joined network 1, with no profile mismatch. All 22 NET_END records report ok.
- Startup UI/IMU service gaps both 26 ms. The 631 ms loop interval with loop_n=0 and
  contexts=8192 is setup, not a runtime MQTT freeze. No later MQTT service window exists
  because there is no reconnect to measure.
- Only one explicit runtime LOOP_GAP: 09:12:32.430, 1222 ms, including 1111 ms in
  live_connect. It succeeds. The retained OP_HEALTH loop maximum is also 1222 ms.
  This is the known synchronous media behavior; JP reports no practical problem.

### Live frame rates

FPS is frames divided by the full recorded elapsed duration, including startup wait;
not a sampled peak rate or a rate computed after removing the first-frame delay.
Times below identify LIVE_BEGIN, with results from the matching LIVE_END.

| Local start | ID | Frames | Duration s | FPS | End reason | First frame s | Maximum frame gap s |
|---|---:|---:|---:|---:|---|---:|---:|
| 09:01:32 | 2 | 167 | 60.245 | 2.77 | duration | 1.077 | 0.995 |
| 09:03:23 | 8 | 82 | 26.611 | 3.08 | screen_left | 1.228 | 0.410 |
| 09:12:31 | 9 | 171 | 61.633 | 2.77 | duration | 1.776 | 1.762 |
| 09:18:25 | 10 | 176 | 60.167 | 2.93 | duration | 1.339 | 1.143 |
| 09:41:36 | 13 | 180 | 60.182 | 2.99 | duration | 1.219 | 0.961 |

All five report failure=none. ID 8 ended when the screen was left, not through a
recorded network failure. The four full-duration cycles delivered 694 frames over
242.227 s, weighted mean **2.87 FPS**. Different durations/content/path conditions
prevent interpreting the short cycle's higher rate as a firmware improvement.

All nine still images succeeded (HTTP 200, expected bytes received), total times
1112-1383 ms. There is no repeat of F004's 3-6 s still responses or 5.302 s Live-connect
loop gap this morning. This is a field comparison, not a controlled benchmark.

### Logger and resources

No dropped/truncated records; queue high=8, writer stack minimum 3592 bytes. Two
slow-operation warnings at 09:35:34 and 09:35:41; maximum write 235.312 ms and flush
240.719 ms, without loss. These extend I004's recorded maxima but do not warrant
changes in the absence of an observed impact.

MQTT attempt stack minimum 7324 bytes; internal/DMA largest minimum 51188 bytes.
Periodic internal-largest samples minimum 26612; retained DMA-largest minimum 21492,
1012 bytes above the 20480-byte gate, unchanged from F004's lowest value. Internal
free minimum 25664 and DMA free minimum 18000 are different metrics from contiguous
largest-block minima; their historical extrema need not occur at the same instant.
No recorded largest-block gate breach, watchdog or reset within this session.

### September 26 post-export addendum (does not describe this morning)

New boot 55 tail confirms F004's export completed: HTTP_GET_END bytes=writer_bytes=
732766, both CRCs=09F0623B, crc_check=match, result=ok, matching the preserved file.
Appends resumed after a 12133 ms pause.

Later, USB power disappeared at 18:56:43.548. At 18:56:58.065, 14.516 s later by
uptime, WiFi reported auth_expired; MQTT loss followed 4 ms later by uptime. A pending
image failed with http_status/code=-1 at the same transition. Subsequent no_ap_found/
sta_leaving records represent retries of this one outage, not separate established
session losses. No IP restoration preceded clean shutdown at 18:57:41.810 (pending=0).

F004's no-loss statement was correct for its captured prefix, but must not be expanded
to the entire day. This newly visible loss is temporally associated with end-session
power removal. Phone departure/hotspot shutdown is plausible, not established, and
there is no basis to call it a new unexplained mid-ride loss. Keep the observation
without assigning a physical cause or asking JP for another test.

### Did the software cause the improvement? Findings and next decision

The measured result agrees with JP's observation: this morning was stable. A causal
claim about fewer WiFi drops is **not supported** by this evidence:

- P001 keeps UI/IMU responsive during MQTT connection work. That benefit was already
  demonstrated in bench cases and F003, but no reconnect happened this morning.
- The latest P005 change permits prompt retry after a >=60 s stable session and gives
  connecting TLS/CONNACK up to 10 s. It reduces avoidable recovery waits and allows
  slower handshakes; it does not alter WiFi radio/beacon behavior. Today's 450 ms TLS
  and 39 ms MQTT exchange never approach even the previous 5 s bounds.
- P006 fixes initial broker-profile selection from the joined network. Today's profile
  agrees, but no mismatch occurred here to demonstrate that fix making a difference.
- Neither change adjusts Live pacing or media throughput. Better network/server
  conditions are a plausible explanation for today's faster media, but are unmeasured.

Thus the known software benefits and this clean ride can coexist without claiming
the changes prevented the radio losses. It is too early to separate natural variation
from any indirect effect. I002/I005 remain open; I001 remains addressed. I003/P004 stay
deferred under JP's decision. Monitor I004 and memory using ordinary exports; no new
implementation or dedicated bench test justified. Review by Claude remains optional.


## F006 - September 27 afternoon BIKE ride

Author: Codex, September 27, 2026. JP installed a companion module on his bike and
requested analysis using this same field-journal workflow. Similar setup to the car;
**this session is a bike ride**, not a car ride. JP supplied no specific symptom.
Keep this module's boot numbering separate from car boots 39-56 in F001-F005.
No firmware changes, build, flash or new bench case performed.

### Evidence and scope

Folder `evidence/bike/`, one file and no screenshots:
`start-unknown_146-1-current-544946.log`, actual 544946 bytes, filename size agrees,
2495 records. SHA256 `63fefbae7ef35dcdb4f98ac524ec20494005eeaa4c61ac2dd4f9e932fa32b89d`;
computed CRC32 `79E78976`. Preserve as received. This is a different cumulative log
from the car export, not its replacement. All boot sequences 128-146 are contiguous
within each boot; sequence order is authoritative when asynchronous timestamps differ.

The afternoon ride is **boot 145**: power_on, compiled `Sep 27 2026 10:48:00`, matching
the day's UI-change build era but not embedding an exact Git SHA. Synced local coverage
UTC-04:00 is 14:55:16.343-15:45:16.685; uptime at clean shutdown is 3063741 ms (51 min
3.741 s including unsynced startup). Boot 146 is the later export session, synced
15:46:54.098-15:47:07.964, also power_on on the same build. It ends at HTTP_GET_BEGIN;
its own END/CRC/close records are not in this snapshot. The computed checksum identifies
the supplied file but is not an independent HTTP completion comparison.

Boots 128-144 are earlier bench/setup/history, not the afternoon ride. In particular,
boot 142 contains morning disconnect/reconnect activity and boots 142/144 report
watchdog resets before the ride. Do not count these as bike-ride failures or assume
all were monitor switches without confirmation. Earlier boot 130/136 monitor-switch
resets were already confirmed in the bench record. No reset occurs within boot 145,
and it ends SESSION_END reason=shutdown pending=0. No unlogged interval between
physical departure/arrival and powered times is inferred.

### Connectivity: slow initial discovery, then stable

**No WIFI_DISCONNECT or MQTT_LOST in ride boot 145 or export boot 146.** All 51 ride
HEALTH/NET_HEALTH pairs report connected/online, with no suppressed WiFi events.
Periodic RSSI -51 to -25 dBm. Configured/joined network 1 agree; no profile mismatch.

Startup details, measured from boot because the clock was not yet synchronized:

| Event | Uptime ms | Interpretation |
|---|---:|---|
| First scan ends | 5305 | 13 networks seen, requested SSID not found |
| Second scan ends | 8331 | 14 networks seen, requested SSID not found |
| Setup retry 2 | 53529 | About 45.2 s after the second scan ended |
| Next scan succeeds | 56839 | Requested SSID found |
| GOT_IP | 58254 | First connection, not recovery of a lost session |
| MQTT worker END | 59067 | Success, total 677 ms |
| MQTT connected adopted | 59262 | Main observes worker completion |
| SETUP_COMPLETE | 61083 | Ready |

The scan 'failed' result means the requested SSID was absent from returned results,
not that the driver scan itself failed. The scan profile field reports primary even
on the helper used to search a secondary name; do not infer both scans targeted the
same SSID from that label alone. Once reachable, MQTT was quick: DNS 116, TCP 61,
TLS 354, MQTT exchange 35 ms. Most startup delay was the existing WiFi retry policy,
not a slow broker handshake. Hotspot discoverability/enable timing is unknown; the log
cannot tell when the phone began advertising relative to those scans.

Startup SERVICE reports UI gap 217 ms and IMU gap 226 ms over an 873 ms window,
loop_n=0, before SETUP_COMPLETE. Source initWiFi() services background work between
200 ms stabilization delays; this supports setup cadence as the explanation. Do not
claim the usual <=30 ms startup service result or a runtime MQTT reconnect freeze.
The P001 worker itself completes successfully; no post-startup reconnect is measured.
Export boot 146 connects successfully in 921 ms (UI/IMU gaps 26/28 ms).

### Media and the isolated pause

Both still requests succeed with HTTP 200 and exact expected byte counts: 14:57:50,
1806 ms total; 15:40:44, 1369 ms total. All three Live sessions report failure=none.
FPS below is frame count divided by full elapsed duration, including startup.

| Local start | ID | Frames | Duration s | FPS | End | First frame s | Max frame gap s |
|---|---:|---:|---:|---:|---|---:|---:|
| 14:57:51 | 2 | 156 | 60.308 | 2.59 | duration | 1.312 | 1.972 |
| 15:31:47 | 3 | 167 | 60.382 | 2.77 | duration | 1.353 | 0.991 |
| 15:40:45 | 5 | 14 | 6.089 | 2.30 | screen_left | 1.096 | 0.452 |

The short final sequence ended on leaving the screen, not a recorded network failure.
Only the two unsuccessful startup scans have failed NET_END results; all media network
operations succeed.

At **15:38:00.468**, one LOOP_GAP records **1102 ms**, observed_span=unmeasured,
span_ms=0. Nearby health snapshots show screen 4 (inclinometer), idle media, WiFi/MQTT
online and no increase in storage maxima. No recorded UI action or network attempt
coincides with it. The log establishes a main-loop scheduling gap, not its cause or
whether JP noticed a freeze. Do not attribute it to MQTT, SD, touch or NVS without
evidence. This becomes I007 for review/monitoring, not automatic implementation.

### Resources: a new low worth reviewing

Zero logger drops, truncations or slow-operation counts; queue high=7, writer stack
minimum 3592 bytes. Maximum write 9.225 ms, flush 12.352 ms, mount-inclusive SD
172.862 ms. Worker stack minimum 7228 bytes and attempt internal/DMA-largest minimum
51188 bytes. No error-level records, media allocation failures or logger errors in
ride boot 145. USB power disappears at 15:44:24.921; orderly shutdown follows at
15:45:16.685 with no pending log records.

A new low: retained **DMA-capable largest block 19444 bytes**, compared with 21492 in
F004/F005. It was 24564 at 14:58:15 during the first Live and 19444 at 14:59:15 after
that cycle, so the minimum occurred in that interval, not necessarily at the latter
sample. It remains 19444 thereafter because this is a boot-retained minimum, not proof
of a persistently small current block or accumulating leak. No MQTT attempt overlaps
this interval. Media activity is a temporal association; it does not identify the
allocation responsible or prove the two added UI labels caused it.

The specification's 20480-byte gate is for **internal largest block**. Periodic general
internal-largest samples stay >=31732, while DMA-largest uses the narrower internal+
DMA capabilities and is retained across samples. Thus 19444 is 1036 below the numerical
reference, but not alone proof that the formal general-internal gate was crossed.
Conversely, periodic samples cannot prove its transient minimum stayed above the gate.
No full log status or per-media general-internal retained minimum was supplied here.
Record I006 honestly as a reduced memory margin requiring focused review, not a crash,
not a proven leak, and not a blanket resource-gate pass.

### Findings and next decision

Connectivity during the actual ride was stable; I002/I005 remain open historically.
P005 retry after established loss was not exercised. Startup discovery wait is separate
from a disconnect. JP says late hotspot availability is possible and asks to focus
on the ride itself; no startup follow-up is requested. I001 remains addressed for the
previously measured MQTT problem; the new unmeasured I007 gap is not attributed to it.
P004 remains deferred. I006/I007 are the new observations for a focused Claude review,
with no authorization for memory tuning, a new worker, or a firmware change.

JP replied that delayed hotspot availability is possible and requested focusing on
the ride; he did not confirm a perceived pause at 15:38. Do not pursue startup testing.
Continue normal use and retain this module's logs separately from car exports. Recommend Claude
spot-check the retained-vs-current memory interpretation and unmeasured pause, following
the shared review workflow, before deciding whether any narrow follow-up is worthwhile.


## F007 - September 27 later CAR rides: repeated link loss and slow recovery

Author: Codex, September 27, 2026. JP reports WiFi drops and a long MQTT disconnection
around 12:50. This is the car module, separate from F006's bike module. No new UI-freeze
symptom was reported. Closest logged prolonged cluster is **12:53-12:55**; no recorded
WiFi/MQTT loss occurs at exactly 12:50. Treat JP's time as approximate.

### Evidence and coverage

`evidence/2026-09-27-car/start-unknown_65-1-current-1271452.log`: 1271452 bytes,
5889 records, filename size agrees. SHA256
`315decdd7e5286d1ceff97405b87c622dcce0fa60718580c7fc1ed6f0e2d9798`;
computed CRC32 `12A4B338`. No screenshots supplied. The entire 854829-byte F005
morning file is an exact prefix, so earlier faults are not new afternoon events.

Boot 56 tail confirms F005 export success, both device CRCs=0FA7EB89 over 854829 bytes,
matching that retained file, with appends resumed after 13703 ms. Boot 57 is a short
unsynced older-build session with no established date/ending. Boot 58 reports other/
reset_code=11 on the newer compiled label; do not infer a spontaneous fault from it.
Boots 59-65 report power_on, with no intervening ride watchdog or brownout recorded.
New analyzed boots 58-65 each have contiguous sequence numbers starting at 1.

All these new boots report compiled `Sep 27 2026 10:48:00`, consistent with the UI-label
build era including accepted P001/P005/P006, not proof of an exact flashed Git SHA.
Synced local times UTC-04:00; initial setup may be unsynced:

| Boot | Synced coverage | End |
|---|---|---|
| 58 | 11:20:09-11:22:36 | shutdown |
| 59 | 11:29:49-11:41:19 | shutdown |
| 60 | 11:48:03-11:53:32 | shutdown |
| 61 | 12:49:22-13:09:19 | shutdown |
| 62 | 14:22:22-14:45:16 | shutdown |
| 63 | 17:37:07-18:25:22 | shutdown |
| 64 | 18:53:56-19:32:45 | shutdown |
| 65 | 19:46:01-19:53:41 | current-log export begins |

All seven completed sessions end pending=0. The last snapshot excludes its own HTTP
completion as expected; no additional export needed for this analysis. Boot 63 also
records an earlier successful 1160268-byte HTTP export, device CRC check match and
result=ok; no separately supplied phone file is assumed for that intermediate transfer.

### The reported long outage: boot 61

The first WiFi beacon loss is 12:53:03.381, MQTT loss adopted at 12:53:03.403. IP
returns 12:53:17.690 after a failed reassociation/auth_expired within the same outage.
MQTT starts 2 ms after GOT_IP and succeeds at 12:53:26.987. Total MQTT outage
**23.583 s**, including about 14.3 s to regain IP and a 9.290 s connect attempt.
It then stays online only **10.196 s** before another loss.

The longest uninterrupted outage is **12:53:37.182-12:54:21.512 = 44.329 s**:

| Step | Local time | Contribution |
|---|---|---|
| MQTT loss after beacon timeout | 12:53:37.182 | Start |
| IP restored | 12:53:40.699 | 3.517 s after loss |
| Retry starts | 12:53:52.187 | 15.004 s after loss; short prior session retains backoff |
| TLS attempt fails | 12:54:04.161 | TCP 1958 ms + TLS 10003 ms; total 11975 ms |
| Next retry starts | 12:54:19.168 | 15.007 s after worker END |
| MQTT restored | 12:54:21.512 | Final successful attempt 2341 ms |

The first 15 s includes WiFi recovery; do not add those 3.517 s again. Approximately
30 s is the specified retry spacing, 14.3 s connection work/adoption. The ten-second
TLS failure has no fresh detailed TLS error, so no certificate/server error is inferred.

Online lasts another **12.301 s**, then a third loss at 12:54:33.813. IP returns
12:54:37.365; the short-session rule again waits until 12:54:48.814, then a 5550 ms
attempt restores MQTT at 12:54:54.368 (**20.555 s** outage). From first loss to this
restoration is about 111 s with two brief online intervals, not one continuous 111 s
MQTT disconnection. This repeated loss explains a prolonged impression of instability.

### All recovered MQTT outages in new coverage

Measured from MQTT_LOST adoption to MQTT_CONNECTED adoption; retry driver events do
not each count as a new outage. There are **11 recovered MQTT outages**, containing
**14 beacon-timeout events while USB power remains present**. Three additional beacon
losses occur during already-disconnected recovery, explaining the different counts.

| Loss time | Boot | MQTT unavailable s | Main recovery complication |
|---|---:|---:|---|
| 11:51:37 | 60 | 24.654 | TCP fails at 5002 ms, then 15 s retry spacing |
| 12:53:03 | 61 | 23.583 | 14.3 s IP recovery and slow successful TCP/TLS |
| 12:53:37 | 61 | 44.329 | Short-session backoff, ten-second TLS failure, retry |
| 12:54:33 | 61 | 20.555 | Short-session backoff plus 5.550 s connect |
| 13:06:06 | 61 | 35.645 | WiFi drops again during TLS; cancellation then retry |
| 13:07:14 | 61 | 24.269 | Short-session backoff, another WiFi loss, slow TLS |
| 14:27:12 | 62 | 8.548 | 3.6 s IP recovery, then 4.969 s connect |
| 14:27:57 | 62 | 38.505 | Short-session backoff, TCP timeout, another WiFi loss |
| 14:37:26 | 62 | 27.250 | DNS fails after 7000 ms, then retry succeeds |
| 19:16:31 | 64 | 16.219 | Successful TLS takes 9741 ms |
| 19:52:13 | 65 | 12.965 | DNS 2369 ms plus successful TLS 6766 ms |

The DNS failure is a resolver failure returned before the 15 s application deadline,
not proof that the configured wait was shortened to 7 s. Causes span DNS, TCP, TLS
and repeated link loss; no single universal DNS explanation fits.

Four further MQTT losses near session endings follow USB power removal: 11:22:13,
13:08:58, 14:44:54 (auth_expired), and 18:25:01 (beacon_timeout then assoc_expired).
They do not recover before clean shutdown and are separate from the 11 recovered
outages. The last has weak reported RSSI -78/-90 dBm. The 14 powered beacon events
report -34 to -47 dBm, so weak sampled signal alone does not explain those episodes.
RSSI at an event does not prove continuous beacon reception or a healthy internet path.
Phone movement, hotspot policy, radio environment and upstream conditions remain
unresolved; do not assert a physical cause from reason labels alone.

### What the accepted changes did and did not fix

P005 now has positive field evidence beyond the earlier clean rides:

- After stable sessions, the first MQTT attempt starts about 2-18 ms after restored
  IP in the measured cases; it does not impose an extra 15 s initial recovery wait.
- Short sessions of 10.196, 12.301, 32.248 and 36.815 s retain the intended backoff.
  These delays are policy operating as approved, not a newly found scheduling defect.
- Four successful TLS phases exceed the old 5000 ms allowance: boot 61 id 7 **8483 ms**,
  boot 61 id 8 **6641 ms**, boot 64 id 2 **9741 ms**, boot 65 id 2 **6766 ms**.
  This establishes that the extended allowance is useful for real slow handshakes;
  exact counterfactual recovery time on old firmware is not measurable here.
- One TLS attempt still fails at 10003 ms. The allowance helps but cannot guarantee
  success or keep a flapping WiFi association alive. No further timeout increase is
  proposed automatically.

There are 16 post-startup attempts: 11 success, two TCP failures, one TLS failure,
one DNS failure, one cancellation on renewed WiFi loss. Across all these windows,
UI gap <=77 ms, IMU/loop <=74 ms, and all over100 counters zero. P001 continues to
prevent multi-second main-task freezes during long MQTT work. These maxima are higher
than F003's <=25 ms but remain under 100 ms. A one-millisecond serial_commands span
in SERVICE is not evidence of actual user commands or the cause of larger UI calls.
No MQTT-only loss without a corresponding WiFi episode occurs in this new coverage.

### Media, logging and memory

All 18 still images succeed; the slowest completes in 1731 ms. Sixteen Live sessions
begin: ten end normally on duration, two on screen_left, three fail connection_closed
at the post-power-removal WiFi losses (13:08:58, 14:44:54, 18:25:01). One in boot 60
runs into shutdown without LIVE_END; do not invent its completion or a network failure.

Full-cycle FPS (frames divided by entire recorded duration):

| Start | Frames | Duration s | FPS |
|---|---:|---:|---:|
| 11:30:25 | 186 | 60.366 | 3.08 |
| 12:50:05 | 154 | 60.339 | 2.55 |
| 13:00:40 | 185 | 60.459 | 3.06 |
| 14:23:21 | 190 | 60.208 | 3.16 |
| 17:38:02 | 138 | 60.341 | 2.29 |
| 18:17:41 | 174 | 60.245 | 2.89 |
| 18:19:57 | 169 | 60.197 | 2.81 |
| 18:54:36 | 196 | 60.282 | 3.25 |
| 19:22:45 | 193 | 60.358 | 3.20 |
| 19:24:15 | 200 | 60.366 | 3.31 |

One successful cycle (19:22:45) includes a 4.557 s frame gap despite its 3.20 average
FPS, so average FPS is not proof of uniformly smooth playback. Only three explicit
main-loop gaps, all media spans, 1018-1140 ms; no F006-style unmeasured gap here.

Logger drops/truncations=0; queue high reaches 9 without loss. Three slow-operation
increments across boots 61/64, maximum write 118.180 ms and flush 232.659 ms. Writer
stack minimum 3016 bytes. Across all new MQTT attempts, worker stack >=7228 and
attempt internal/DMA-largest >=45044 bytes. Periodic general-internal-largest >=31732,
retained DMA-largest >=21492. No recurrence of bike I006's 19444 low. As with F006,
periodic general-memory samples and retained DMA minima are different metrics and
do not by themselves establish every transient general-internal minimum.

### Findings and next decision

I002 remains the principal unresolved issue: powered WiFi losses recur despite good
sampled RSSI. I001 stays addressed; P005 behavior and useful longer TLS allowances are
now demonstrated in the field, while some long recovery comes from intentionally
retained backoff during flapping. I005's earlier MQTT-only outage does not recur as
an independent pattern here. P004 remains deferred; I004 continues without loss;
I006/I007 from the bike are not reproduced by these car sessions.

Recommend Claude's focused review because JP observed a symptom and this evidence
changes P005's field-validation status. Review the 12:53 timing decomposition, stable/
short-session retry distinction, four slow successful TLS phases, and the powered versus
end-session outage count. Continue normal field collection; no code/build/flash or
extra bench campaign is justified solely by this analysis. Do not change retry policy
again without weighing faster flapping recovery against repeated connection load and
JP's preference for essential, evidence-based changes.

### Claude review - September 27, 2026 (acf05d5)

Raw file checked independently: SHA256 and CRC32 `12A4B338` match, with 5889 records.
All 11 recovered outages, their phase timings and the stable/short-session split
reproduce exactly.

**Confirmed findings**
1. **Every recovered outage starts with a driver beacon timeout 3-23 ms before MQTT
   loss,** at RSSI -34 to -47 dBm. None is an MQTT-only loss.
2. **After each beacon timeout the hotspot is back almost at once.** In 13 of 15
   powered beacon timeouts, the rejoin is identical: one failed scan at +2.4 s,
   association at +2.5 s, IP at +3.55 +/- 0.1 s. The exceptions are 12:53:03, which had
   an extra `auth_expired` and IP at +14.3 s, and the post-power 18:25 event. This
   differs from F002, where the AP stayed invisible for up to 24 s. In F007 the
   association breaks and the AP is available again about 2.5 s later.
3. **Losses come in bursts:** 12:53:03, 12:53:37, 12:54:33; 13:06:06, 13:06:18,
   13:07:14, 13:07:25; 14:27:12, 14:27:57, 14:28:31. Intervals are 11-56 s, then the
   link is quiet for 10+ minutes.
4. **P005 works as designed.** After stable sessions, the first attempt starts 2-18 ms
   after IP. Four successful TLS handshakes of 6.6-9.7 s would have failed under the old
   5 s cap. One TLS failure at 10003 ms shows the cap can still be reached.
5. **The 15 s waits in bursts are idle time with the link up.** Waiting with IP restored
   and no attempt: 12:53:37 11.5 s, 12:54:33 11.4 s, 14:27:57 11.4 s. At 13:06:06 an
   attempt was cancelled by a renewed WiFi loss, and the next one still waited 15 s from
   the cancellation: 11.5 s idle after IP returned at +15.2 s.
6. **P001 still holds.** In the 16 post-startup windows the maximum UI gap is 77 ms,
   with over-100 counts all zero.
   - The rise from F003's <= 25 ms comes from single long `lv_timer_handler` calls
     (ui_call_ms 46-67 ms), not from MQTT waits. These occurred on screens 1 and 4,
     about the cost of one full-frame blit.
   - One startup window (boot 65 id 1, unsynced) records `ui_over100=1` (120 ms gap,
     101 ms call). That is outside the post-startup scope, as Codex states, but should
     not be read as "never over 100 ms".

**Hypotheses (not established)**
- The beacon losses look like short interruptions on the phone or radio side, not loss
  of range or a long hotspot shutdown. The AP is back within about 2.5 s, RSSI is
  strong, and module power saving is off. The logs cannot show when beacons actually
  stopped, only when the driver gave up, so the true pause length is unknown.
- The recent rise in UI-call time is likely full-screen redraws on the active screen
  (dashboard or inclinometer), not networking. Look into it only if JP notices
  sluggishness.

**Proposed improvement (one, small; JP decision)**

**P007 - treat a WiFi-caused loss or cancellation as "link returned: retry now",
whatever the session length.** Keep the 60 s gate for losses while WiFi stays
associated (the broker-kick storm case B1 guarded against), and keep 15 s after a
genuinely failed attempt with the link up.
- **Why it is safe:** each prompt attempt needs a full WiFi down-then-up cycle first
  (at least about 3.5 s, and 11+ s between losses here). The attempt rate is therefore
  bounded by the physical link, not by the broker. There is still only one attempt,
  one TLS context and one lease at a time. The memory gate, cancellation and media
  refusal are unchanged.
- **Expected effect on F007:** 4 of the 11 outages about 11.5 s shorter, assuming the
  same attempt outcomes:
  - 12:53:37: 44.3 -> about 32.8 s
  - 12:54:33: 20.6 -> about 9.1 s
  - 13:06:06: 35.6 -> about 24.1 s
  - 14:27:57: 38.5 -> about 27.1 s

  In a burst, a prompt attempt may be cancelled again. That costs one handshake, not
  extra wall time.
- **Complexity:** main needs to know that a link-down event occurred between the last
  adopted connection or attempt and this loss or cancellation. The WiFi event callback
  can increment a counter (`netLinkEvent(false)` already exists). No worker or protocol
  change. Host checks: the link-caused short-session loss is prompt, the link-caused
  cancellation is prompt, and a link-up short-session loss keeps 15 s. Validate in
  ordinary rides; no bench campaign.

**Not recommended now**
- **Shorter spacing after a failed attempt with the link up (15 -> 5 s).** Only three
  cases (11:51 TCP, 12:53:37 TLS, 14:37 DNS). Each next attempt succeeded, but against a
  truly unreachable broker this would roughly triple attempt and lease frequency.
  Revisit with more evidence.
- **Keeping the MQTT socket across a 3.5 s same-IP rejoin, or lengthening the ESP32
  beacon/inactive timeout.** Either could avoid reconnects entirely, but both are
  unproven. We do not know whether the iPhone NAT preserves the TCP session, or how long
  beacons actually stop. They would also change WiFi or ownership policy. Revisit only
  if P007 plus more rides leave outages practically annoying.


### P007 direction and design handoff - September 27, 2026, Codex

JP approved the direction of Claude's F007 proposal at cb06706: link-caused MQTT
losses and cancellations should become eligible on link return even after short
sessions. The [P007 revision-1 design](p007_link_return_retry_design.md) specifies
main-side link-cycle tracking, terminal adoption and one pending eligibility flag,
while retaining broker-kick stability, genuine-failure spacing and all admission
protections. This is recovery scheduling, not a fix for the underlying beacon loss.
Design submitted for Claude review; implementation is not yet approved. Validation
will use ordinary rides, with no new bench campaign. No code, build or flash.

Claude's review is integrated as the basis of this proposal. Preserve the powered
count distinction: the analysis counts 14 powered beacon events plus one after power
removal, whereas review point 2 calls all 15 powered before identifying the post-power
exception. This wording does not change the 11 recovered outages or P007 rationale.


### P007 implementation handoff - September 27, 2026, Codex

Following Claude's no-blocker design review at 2362ea0, JP approved implementation
and explicitly adopted S1 (link-down counter under the existing owner lock, exposed
through View). The [implementation handoff](p007_implementation_handoff.md) records
the small counter change and main-side retry scheduling, with 405 passing host checks
in 17 suites. Broker-only stability and genuine-failure backoff remain; no radio,
worker execution or timeout change. Await Claude code review before JP builds.
No build, flash or hardware test performed. Ordinary rides will validate recovery
timing and media/refusal frequency; no new bench campaign. I002 remains open.


### P007 code-review notes - September 27, 2026, Codex

Claude cleared ec487bc at 1551e65. A small subsequent logging correction suppresses
the misleading wait-15-seconds retry record on successful Result adoption without
changing scheduling. All 407 host checks pass. Resource deferrals retain their
uncounted 15 s backoff and must be separated from network failures in ride analysis;
pre-dispatch request refusal instead retains prompt eligibility. See the handoff
addendum for the delta awaiting Claude's quick review. No build or flash.


## F008 - September 28-29 car rides: P007 exercised in the field

Analysis: Codex, September 29, 2026. JP reports updating firmware yesterday morning
and no noticeable problems during driving. Scope is every newly appended record since
F007, not just the new-firmware rides. Local times below are synced UTC-04:00; use
up_ms for interval calculations. Stationary/driving boundaries within each powered
session are not independently known.

### Evidence and cumulative boundary

- Folder: evidence/2026-09-car; one file, no screenshots supplied.
- File: start-unknown_77-1-current-1804524.log, **1,804,524 bytes / 8,301 records**.
  Filename length matches; computed CRC32 **A2A2F2ED**.
- SHA256: **6e558c8557882b5e902ecc27403d1b3958dcf0877a0328e9b679bea55578aa3f**.
- The first 1,271,452 bytes hash to F007's recorded SHA256
  **315decdd7e5286d1ceff97405b87c622dcce0fa60718580c7fc1ed6f0e2d9798**.
  Thus the earlier export is preserved byte-for-byte even though its former evidence
  folder currently contains no file. Do not count its historical faults again.
- New portion: **533,072 bytes / 2,412 records**, boot 65 tail plus boots 66-77.
  New sequences are contiguous within each boot. The final snapshot excludes its own
  HTTP_GET_END, so the computed file CRC is not independently matched to that latest
  device END or last-result screenshot. No extra export is needed for this analysis.
- Boot 65 tail now confirms F007 export completion: bytes=writer_bytes=1271452,
  both CRCs=12A4B338, crc_check=match, appends=resumed, paused_ms=20546.

### Firmware and coverage

Boot 66 and the brief unsynced boot 67 still identify compiled="Sep 27 2026 10:48:00".
Boots **68-77** identify compiled="Sep 28 2026 07:49:17". Their wifi_return records
and absence of successful-result retry records support deployment of the P007-era
behavior including the logging correction. The log does not embed a Git SHA; do not
claim an exact flashed commit from current main (79430dc, documentation clearance).

| Boot | Synced coverage | Interpretation |
|---|---|---|
| 65 tail | Sep 27 19:53:41-19:55:16 | Previous export completion and end of previous session |
| 66 | Sep 28 06:45:44-07:07:06 | Old firmware morning ride; recovered long outage |
| 67 | Unknown; only through up_ms=5365 | Brief old-build startup; no SESSION_END |
| 68 | Sep 28 08:14:36-08:19:59 | First new build, brief session with power toggles |
| 69 | Sep 28 09:45:20-10:03:09 | New build, three WiFi-led outages |
| 70 | Sep 28 11:37:20-11:48:03 | New build, three WiFi-led outages |
| 71 | Sep 28 11:57:44-12:17:34 | No recorded WiFi/MQTT loss |
| 72 | Sep 28 16:06:07-16:27:46 | MQTT-only outage while WiFi stays associated |
| 73 | Sep 28 17:11:34-17:34:07 | Two outages; one cancelled attempt during renewed loss |
| 74 | Sep 29 06:44:19-07:21:18 | One outage, mainly waiting for usable WiFi |
| 75 | Sep 29 10:31:07-11:02:09 | Two outages; one cancelled attempt; Live frame gaps |
| 76 | Sep 29 12:08:26-12:33:21 | No recorded WiFi/MQTT loss |
| 77 | Sep 29 13:31:33-13:31:40 snapshot | Successful startup and iPhone export |

Boots 68-76 total **193.64 minutes from boot to shutdown** (not all driving).
All nine end with SESSION_END reason=shutdown pending=0. Boot 68 reset=other/code11
coincides with the build transition; cause is not established, and it is not evidence
of an in-ride watchdog. Boots 69-77 are power_on. No new ride watchdog/brownout or
worker fault is recorded. Boot 67's missing end is explicitly left unexplained rather
than counted as a runtime firmware crash.

### Old-firmware and prior-session additions

Boot 65 loses USB power at Sep 27 19:54:17.558, then Live ends with a fetch timeout
at 19:54:53.495. MQTT reports state=-4 loss at 19:55:00.720 without a new WiFi event;
TCP setup fails after 5004 ms and shutdown follows at 19:55:16.442. This is a
post-power-removal end-session interruption, not a newly powered P007 outage.

Boot 66 has one **83.125 s** recovered MQTT outage, Sep 28 **06:51:03.915 to
06:52:27.041**, with three beacon losses. Initial IP return takes about 25.3 s;
the first reconnect fails at the 10 s TLS allowance, the next is cancelled by renewed
WiFi loss, and old policy waits another 15 s after cancellation (11.480 s after IP
has returned) before success. This precedes the new build and is not a P007 regression.
Its three recovery attempts keep UI <=90 ms, IMU/loop <=87 ms with no >100 counts.

### New-firmware outage timeline

All times in this table use MQTT_LOST to subsequent MQTT_CONNECTED, including cleanup
and retry delays. All twelve outages occur with recorded USB power present and recover.

| Date/time of MQTT loss | Boot | Outage seconds | Measured explanation |
|---|---:|---:|---|
| Sep 28 09:56:11 | 69 | 19.332 | WiFi IP return ~13.2 s; connect 6.110 s |
| Sep 28 09:56:44 | 69 | 12.973 | Short ONLINE 13.581 s; immediate retry after IP, connect 9.362 s |
| Sep 28 09:57:29 | 69 | 8.984 | Short ONLINE 32.179 s; immediate retry, connect 5.389 s |
| Sep 28 11:45:21 | 70 | 8.251 | IP return ~3.6 s then successful connect |
| Sep 28 11:45:45 | 70 | 29.987 | Short ONLINE 15.975 s; WiFi/auth recovery takes ~23.9 s |
| Sep 28 11:47:03 | 70 | 3.421 | Short ONLINE 47.671 s; IP return ~2.7 s and fast connection |
| Sep 28 16:10:27 | 72 | 20.597 | No WiFi loss; TCP setup fails at 5003 ms, retained 15 s retry then success |
| Sep 28 17:15:33 | 73 | 20.577 | WiFi returns, attempt cancelled by another beacon loss; prompt replacement succeeds |
| Sep 28 17:29:31 | 73 | 21.304 | IP return ~13.2 s; connect 8.138 s |
| Sep 29 06:49:54 | 74 | 26.034 | Repeated unsuccessful joins; IP return ~25.3 s, connect 729 ms |
| Sep 29 10:40:01 | 75 | 10.369 | WiFi return then connect 6.711 s |
| Sep 29 10:40:24 | 75 | 18.419 | Short ONLINE 12.159 s; renewed WiFi loss cancels first attempt; prompt replacement succeeds |

**WiFi:** eleven recovered outages begin with beacon loss. Two additional beacon
losses happen inside pending attempts: **13 powered beacon events**, not thirteen
separate MQTT outages. The other 39 driver disconnect records are 19 no_ap_found,
19 sta_leaving and one auth_expired during recovery. Last reported RSSI at beacon
loss is -33 to -47 dBm. These strong samples do not establish uninterrupted radio
quality, nor identify an iPhone/ESP32 cause. Some IP recoveries now take 13-25 s;
P007 cannot eliminate that unavailable-link interval.

### P007 field validation and retained failure policy

All **13** post-startup attempt starts following a fresh WiFi return occur **2-19 ms**
after the latest GOT_IP (device uptime). Each relevant terminal decision reports
policy=wifi_return, link_changed=1, wait_ms=0. No successful-result backoff record
appears; the logging correction is exercised.

- Five lost sessions were ONLINE for less than 60 s: **13.581, 32.179, 15.975,
  47.671 and 12.159 s**. They all receive wifi_return eligibility. Four have usable
  link back before the former 15 s deadline, so the scheduling improvement is directly
  visible; the 15.975 s session takes ~23.9 s to regain IP, where old backoff would
  already have expired. Do not claim a 15 s saving for every short session.
- Sep 28 17:15:46.256: TLS attempt cancelled after a second beacon loss. Result
  adoption at up_ms=272517 schedules zero wait. GOT_IP=273744, next BEGIN=273752:
  **8 ms after IP**, not another 15 s after cancellation.
- Sep 29 10:40:35.646: another TLS cancellation. Decision=586971, GOT_IP=590630,
  next BEGIN=590633: **3 ms after IP**. Both replacements succeed.
- A different path is correctly retained at Sep 28 **16:10:27**: no WiFi event in
  boot 72, MQTT state=-4 loss after ~265.6 s ONLINE, policy=stable. Immediate attempt
  fails TCP setup (5003 ms). policy=backoff at up_ms=283729; next BEGIN=298732,
  **15003 ms later**, and succeeds. This supports I005's recurrence, not a beacon
  cause or a proven broker defect. WiFi association is not proof of internet reachability.

There are **15 post-startup attempts**: twelve successes, two link-caused cancellations
and one TCP failure. All ten startup attempts also succeed. No resource-deferral
result, worker fault, initial-budget exhaustion or IMAGE_REFUSED/LIVE_REFUSED is
recorded. MQTT_IMAGE includes seven ignored_live results, which are the existing
Live exclusivity behavior, not new MQTT lease refusals. This sample therefore does
not show a practical increase in media refusals from prompt retries, but cannot
establish that none will occur in future bursts.

The data validates P007's intended scheduling paths. It does not support a claim
that P007 prevents WiFi loss or that total outage durations are a paired before/after
improvement under identical radio/cellular conditions.

### Responsiveness, memory and logging

- All 15 recovery SERVICE windows: UI gap <=**66 ms**, IMU <=**65 ms**, loop <=**64 ms**;
  every over100 counter is zero. This supports JP's symptom-free observation and I001
  remaining addressed, including failed/cancelled reconnects.
- Startup is separate: boot 72 id=1 records UI=105 ms and IMU=113 ms, one over100 each.
  Setup windows have loop_n=0 and a loop gap equal to the startup window (maximum
  912 ms); that is not evidence the serviced UI was frozen for that whole interval.
- MQTT attempt minimum stack margin **7164 bytes**, internal-largest **47092 bytes**,
  above the 2048/20480 gates. No allocation-failure record. Stack remains PSRAM with
  internal TCB in attempt reports.
- Periodic general-internal largest minimum **28660 bytes**. Retained DMA-largest
  reaches **19444 bytes in car boots 68/72/75/76**, reproducing I006 outside the bike.
  That retained capability-specific minimum is not the same measurement as the general
  internal-largest admission gate; periodic snapshots cannot certify every transient
  outside MQTT attempts. Keep monitoring; do not declare a gate failure or dismiss it.
- HEALTH drops=0, truncated=0 throughout; sequence continuity agrees. Queue high <=9.
  Five slow-operation count increments across boots 68/72/75; measured write/flush
  maxima **118.569/233.910 ms**. Writer stack minimum **3480 bytes**. No logging loss,
  incomplete shutdown queue, or new storage error is recorded.

### Images, Live and other pauses

All **20 still-image requests succeed**, longest total=1824 ms. There are 19 Live
starts: twelve duration completions, one fetch timeout and six without LIVE_END before
shutdown. Do not manufacture failure or FPS values for the six shutdown-truncated
sessions. The timeout is Sep 28 17:34:07, after USB removal at 17:33:13 and immediately
before session shutdown; it is an end-session media failure, not a recovery freeze.

The twelve full cycles deliver **1.92-3.16 FPS**, aggregate **2.73 FPS**
(1979 frames / 723.604 s), calculated from frames divided by elapsed duration.
Five explicit main-loop gaps are media-attributed, **1.029-1.092 s**; no new unmeasured
>=1 s gap reproduces I007. P004 stays deferred because JP reports no practical symptom.

Two separate Live frame gaps are **4.542 s at Sep 29 10:33:09** and **4.370 s at
10:35:39** (boot 75). Both cycles complete. MQTT and WiFi remain connected during
these intervals; the WiFi losses occur later around 10:40. These are frame-delivery
gaps, not proof of a 4.5 s UI/main-loop freeze. No new media worker is justified by
this measurement alone.

### Findings, decision and review handoff

P007's two target cases (short-session link loss and cancellation) now have ordinary-
ride evidence, with the unchanged genuine-failure backoff also exercised. I001 remains
addressed. I002 remains open; I005 recurs as an independently timed TCP/path problem
while associated. I006 now occurs in both car and bike. No new firmware change or
bench campaign is proposed from this symptom-free sample; continue ordinary use.

Request Claude's focused review because this is first field validation of P007 and
changes I005/I006 evidence. Check the cumulative hash boundary, build transition,
short-session/cancellation timelines, the 16:10 MQTT-only outage and memory distinction.
Keep radio/NAT/timeout changes and P004 deferred unless symptoms or stronger evidence
justify them. No code, build or flash performed by Codex; this update is documentation.

### Claude review

Pending.
