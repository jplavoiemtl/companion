# SD diagnostics checkpoint - 2026-09-19, end of day

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
[console](bench_data/sd_stage2_2026-09-20_boot87_live_normal/console.txt) and
[current log](bench_data/sd_stage2_2026-09-20_boot87_live_normal/87-current.log.txt).

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
[console](bench_data/sd_stage2_2026-09-20_boot87_live_outage/console.txt) and
[current log](bench_data/sd_stage2_2026-09-20_boot87_live_outage/87-current.log.txt).

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
[console](bench_data/sd_stage2_2026-09-19_boot86_hotspot/console.txt) and
[current log](bench_data/sd_stage2_2026-09-19_boot86_hotspot/86-current.log.txt).

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


## Latest Stage 2 result: hotspot recovery works; diagnostic fix repeat next

JP reports the 22:14-22:16 hotspot case ran fine on boot 84. Wi-Fi and real
MQTT recovered automatically; MQTT loss state=-3, reconnect 776 ms, Latest
1289 ms total. Download 215429 bytes, CRC32 F2B32C72, exact prior snapshot
prefix; 131 contiguous boot-84 records and 11 paired spans. Zero queue drops,
errors or slow writes; writer margin 3144. Sampled recovery/HTTPS largest
minima 51188/31732 exceed 20480. No new reset. Raw evidence is saved under
bench_data/sd_stage2_2026-09-19_boot84_hotspot/.

Two diagnostic defects were exposed: failed scans' RSSI -128 was marked valid,
and alternating disconnect reasons bypassed consecutive-repeat suppression.
Local corrections reject RSSI <=-128 or >=0, retain explicitly labeled last
valid RSSI, and use four bounded reason/profile suppression slots (five seconds).
Raw RSSI validity and cross-reason suppression counts are explicit; reason 36
is labeled sta_leaving. Retry behavior is unchanged. Sixteen network host checks
pass, including policy replay; this is not C++ compilation or board validation.

Next single case: JP rebuilds/flashes amoled-1-8-core-3-3-11 in VS Code, then
repeats hotspot off 30 seconds / automatic recovery / Latest once. Selected
generated sketch removed before handoff. Keep USB/browser connected, dashboard,
DTR=true/RTS=false and test switches off. Status before outage and while offline;
restore hotspot with settings open, allow up to 90 seconds to recover without
serial off/on or reset. If recovery fails, stop and send console/status. Otherwise
Latest once, status, download current.log, final status. Send console and download.
Verify corrected RSSI/suppression records, CRC, zero drops/errors and memory gate.
No Live in this case. Current measured board still has the pre-correction build.

Normal flags: enabled=1, hooks=0, fixture=0, PSRAM=1. Stage 2 remains local,
uncommitted and unaccepted; Stages 3-4 unstarted. Stage 1B checkpoint f899e3c
is pushed. Known monitor-close limitation remains; iPhone plan untouched.

## Stage 2 preparation history

Stage 1B acceptance/evidence is committed and pushed as `f899e3c` on
`sd-diagnostics`. Stage 2 network-event logging is implemented locally and
uncommitted; no firmware compile/flash or Stage 2 acceptance is claimed.
See [implementation and first case](../src/diagnostics/STAGE2.md).

Normal source configuration: core 3.3.11 profile, enabled=1, hooks=0, fixture=0,
PSRAM=1, build tag `stage2-network`. Selected generated companion.ino.cpp
removed for JP's VS Code rebuild. Last measured board state is still Stage 1B
boot 82 until JP flashes. Next case only: normal startup/status, dashboard for
about 70 seconds, download current.log once, final status and inspect new records.

Validation: 14 source-contract regressions and 47 existing host USB checks
pass; these are not a C++ build or hardware validation. No reconnect-policy,
writer placement, transport-limit or Stage 0 probe changes. Stages 3-4 remain
unstarted. Untracked iPhone plan remains untouched. No Stage 2 commit/push.

JP also observed a possible return toward 48 Hz IMU operation as night images
became smaller. Retained captures show 12.9 KB average Live frames and normal
IMU windows of 42.43/42.79 Hz; no new 48 Hz capture was supplied. Smaller images
plausibly affect Live throughput, but no IMU causal conclusion or renewed
investigation follows from that observation.

## Stage 1B accepted by JP - September 19, 2026

JP explicitly stated: "I accept Stage 1B. Please commit and push then proceed
to the next step." Stage 1B is accepted with the documented large-download-only
FPS exception, accepted IMU rate, unresolved pre-logger monitor-close issue
and deferred unsupported/bad-card testing. The 20480-byte memory gate and
transfer limits remain unchanged. Cases A/B/C pass; normal logging-on flags
remain enabled=1, hooks=0, fixture=0, PSRAM=1 on core 3.3.11 (last measured boot 82).

Commit and push this acceptance/evidence checkpoint first, then implement
Stage 2 network-event logging under the existing plan. JP builds and flashes;
bench instructions remain one case at a time. Earlier pending-acceptance
statements below are historical and superseded by this explicit decision.
Leave the untracked iPhone download plan untouched.

## Pre-acceptance evidence assessment

This update supersedes the historical next-session review below. JP reports
that Claude and Codex reviewed the remaining evidence after commit 19845b8:
existing core 3.3.11 TLS/memory evidence is sufficient against the unchanged
20480-byte gate. Both focused performance comparisons now pass. Stage 1B
still requires JP's explicit acceptance; Stages 2-4 have not started.

Completed in one sitting, one case at a time:

- A: core 3.3.11 logging off; status, Latest once, one full Live cycle without downloads, then status.
- B: same profile with normal logging on; repeat status, Latest once, one full Live cycle without downloads, then status.
- C: same logging-on build; full Live cycle with current plus newest three archives downloaded about 10 seconds into Live, then status.

Calculate FPS from frame count divided by duration. Target no more than about
5% loss for B versus A and C versus B. Review network timings before interpreting
a marginal difference. Require CRC success in C, zero logging-on queue drops,
no new stalls/resets, and measured largest internal blocks at least 20480 bytes.
Reuse passed gates; the roughly 8% exception applies only to large downloads.
Skip older-core Live and writer-core comparisons; IMU acceptance is unchanged.

Source review confirms Live geometry prints only when dimensions change and
summaries print at the end, not every frame. The installed HWCDC driver uses
tx_timeout_ms for TX-lock waits, so the post-a675a76 overlap check remains useful.

Case A passed at 21:16-21:18 on September 19. JP reports normal video.
209 frames / 60.267 s full-Live probe window = approximately 3.468 FPS;
Latest total 1207 ms. HTTPS, all three Live TLS windows and full Live each
have sampled largest minimum 31732 bytes, above 20480. No observed reset or
media failure. Raw capture: bench_data/sd_live_2026-09-19_case_a.txt; full
measurement table and duration-method caveat are in the bench results.

Case B passed at 21:23-21:25, boot 82: 219 frames / 60.142 s = 3.6414 FPS,
5.00% above A (no measured logging penalty; not proof of a benefit). Latest
1191 ms; full-Live largest minimum 28660 bytes, HTTPS/TLS 31732. Writer stack
margin 3752, queue high=1, zero drops/errors/slow writes, no observed reset or
media failure. JP reports normal video. Raw: bench_data/sd_live_2026-09-19_case_b.txt.

Case C passed at 21:28-21:29, same boot 82: 212 frames / 60.193 s = 3.5220 FPS,
3.28% below B, within the 5% target. Bundle starts 10.993 seconds into Live;
current + archives 19/18/17 total 473523 bytes in 4.727 s, all four CRC OK.
Full-Live largest minimum 28660 and TLS 31732; zero drops/errors/slow writes,
no observed reset or new stall. Writer stack margin 3304 after retrieval matches
the earlier passed resource series. JP reports normal video. Raw capture:
bench_data/sd_live_2026-09-19_case_c.txt; complete results in bench results.

Next action: present Stage 1B for JP's explicit acceptance. All agreed focused
checks are complete; no further bench case or rebuild requested. Preserve the
roughly 8% large-download-only FPS exception, accepted 42-43 Hz profile rate,
unresolved pre-logger VS Code monitor-close freeze/reset and deferred
unsupported/bad-card testing. The memory gate remains 20480 bytes.
Stage 1B is not yet accepted and Stage 2 must wait for explicit acceptance.
Current normal configuration remains DIAG_ENABLED=1, hooks=0, fixture=0,
PSRAM=1, profile amoled-1-8-core-3-3-11; last measured board state is boot 82.

No build, flash, commit or push by Codex. JP's untracked iPhone plan is untouched.

## Resume here

JP has stopped for the day. Stage 1 is accepted; **Stage 1B acceptance remains
pending**. Stages 2-4 have not started. Do not resume the IMU investigation:
JP accepts the newer profile's approximately 42-43 Hz rate and reports normal
motion, G-meter and inclinometer operation.

JP restored core 3.3.11 with logging enabled, compiled and flashed from VS Code,
and reports approximately 42 Hz again. Local profile and flags agree. No raw
post-restore capture or new boot number was supplied; the restoration is JP's
confirmation, not a newly analyzed full gate run. Board power state is unknown.

## Current configuration

- Branch: `sd-diagnostics`; checkpoint includes the September 19 documented
  results and review reconciliation. Last runtime-code commit before this
  documentation checkpoint is `a675a76` (bounded USB debug TX waits).
- Profile: `amoled-1-8-core-3-3-11`, Waveshare graphics 1.6.4 and Adafruit expander.
- `DIAG_ENABLED=1`, `DIAG_TEST_HOOKS=0`, `DIAG_USB_TEST_FIXTURE=0`,
  `DIAG_WRITER_STACK_PSRAM=1`. Writer: 8192-byte PSRAM stack, internal TCB,
  priority 1, core 1, parks after cleanup. Stage 0 probes retained.
- Card returned to board. The synthetic fixture was deleted/pruned; archive 19
  is deliberate partial-header recovery evidence, not a disposable test fixture.
- JP builds/flashes. Remove the selected profile's
  `build/build_<profile>/sketch/companion.ino.cpp` before required rebuilds.
- Use the web console with DTR=true, RTS=false. Leave browser test switches off.
- Leave JP's untracked `docs/sd_iphone_log_download_plan.md` untouched.

## Evidence completed

See [bench results](sd_diagnostics_bench_results.md), especially the nine-gate
matrix and September 19 entries, for raw sources and limitations.

- Queue guard at 8/16: all eight injected records persisted; retry passed.
- Selected-reader pruning: synthetic archive removed, transfer ended cleanly,
  current-file retry passed. No synthetic archive from that case remains.
- Core 3.3.11 regression: interrupted rename, partial-header salvage, normal
  shutdown close, deep-sleep close and touch wake all pass.
- NVS/SD overlap: 278 commits, 736 SD records, zero errors, dummy key removed.
- Resource series: five confirmed aborts and five CRC-checked retries, same
  boot. Matching idle internal free memory 105488 bytes, largest block 57332,
  PSRAM 8339860 and writer stack margin 3304 all unchanged. Zero drops/errors.
- Physical-card check: USB current snapshot 130783 bytes, CRC32 9E5F4441,
  exactly matches the prefix of the 133346-byte card file. Additional records
  end with clean shutdown pending=0. No card writes performed by Codex.
- Earlier accepted transport evidence remains: small/current/bundle/2 MiB
  integrity, MQTT overlap, damaged-line recovery, abort/page close/battery
  unplug, 5-second stall, 120-second current deadline, progressing archive
  beyond 120 seconds, and fixture deletion. Reuse these results.

## IMU decision and accepted limits

Same-sitting three-window means: 3.3.11 logging off **42.27 Hz**, logging on
**42.67 Hz**, 3.1.3 logging off **49.25 Hz**. The approximately 14% drop persists
without logging. It is associated with the changed core/library profile;
no specific component is proven responsible. JP accepts it as-is. No writer
core A/B, added timing probes or optimization is planned unless symptoms arise.

Keep the **20480-byte** internal-largest-block gate and every transfer limit.
Keep JP's narrow roughly 8% Live FPS exception during simultaneous large USB
transfers; Live pacing stays reverted. Unsupported/bad-card tests remain
explicitly deferred. VS Code monitor-close freeze/reset predates the logger
and remains unresolved; the USB startup fix does not establish its resolution.

## Next session

1. Review remaining new-profile TLS/memory and performance acceptance evidence.
   Reuse passed tests. If a paired Latest/Live baseline is still missing, give
   JP only that focused case next, using unchanged Stage 0 probes. Any new
   performance comparison must be taken in the same sitting per CLAUDE.md.
   Do not use old-day numbers as a new paired baseline.
2. Present the remaining gate assessment and accepted limitations for JP's
   explicit Stage 1B acceptance. Do not silently mark the stage accepted.
3. After acceptance, implement Stage 2 network evidence: Wi-Fi events/profiles,
   every MQTT attempt, error snapshots before cleanup and TLS phase markers.
   Bench-test with existing off/on controls, one case at a time.
4. Later: Stage 3 operation context and optional log tail; Stage 4 car use
   and retention tuning. USB retrieval should avoid routine card removal.

This checkpoint changes documentation and the accepted-PSRAM config comment;
no runtime behavior change, build or flash by Codex. JP requested commit/push.
