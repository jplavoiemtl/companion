# SD card diagnostic logging plan

Project owner and bench tester: **JP**. References to the owner mean JP.

Status: finalized proposal after [SD review and counter-review](sd_diagnostics_plan_review.md),
[USB review](sd_usb_log_retrieval_plan_codex_review.md) and owner-accepted
[USB counter-review](sd_usb_log_retrieval_plan_counter_review.md). This plan now covers
logging and USB retrieval. Input: [USB investigation brief](ESP32_SD_Log_USB_Investigation.md).
Step 0 passed on the tested board and Chrome with explicit DTR=true, RTS=false.
The VS Code monitor disconnect freeze remains unresolved. See the
[bench results](sd_diagnostics_bench_results.md) for evidence and scope.
Stage 0 initial measurement coverage is complete: normal, Latest HTTPS, Live TLS,
full Live and failed and successful MQTT connects. JP accepted this checkpoint and
authorized Stage 1 on 2026-09-15. The baseline is committed as f406063.
JP built and flashed Stage 1; the initial ready, clock-sync and normal-operation
check passed. Latest then reached a 14836-byte largest internal block, failing
the 20480-byte memory floor. No-card HTTPS recovered to 28660 bytes. An allocation-order
experiment moved the writer after hardware and UI setup, before Wi-Fi initialization.
The card-installed retest still measured 14836 bytes; the experiment did not fix the
memory gate. JP verified retained startup snapshots through log status. The writer
creation interval used 6528 internal bytes; mount readings overlap Wi-Fi startup.
Latest still measured 14836 bytes. Stage 1 acceptance remains on hold. See [Stage 1 handoff](../src/diagnostics/STAGE1.md) for implementation
choices and tests. USB file retrieval and later event hooks remain unimplemented.
The accepted plan remains the review reference. JP requested committing this Stage 1
checkpoint as fe3b5da for code review. Its allocation-order experiment was tested
and showed no memory improvement.

JP accepted the [memory experiment review](sd_diagnostics_memory_experiment_review_claude.md).
JP built and tested the DIAG_WRITER_STACK_PSRAM A/B implementation with hooks off:
internal 6144-byte stack versus experimental 8192-byte PSRAM stack and static
internal TCB. Latest measured 14836 bytes in A and 26612 in B. B passes this
initial 20480-byte memory check; both used 4204 bytes of stack with valid placement.
The captures were about 84 minutes apart with different image sizes, so no
performance improvement is inferred. On 2026-09-16, B completed a full Live cycle
normally after about 11 hours 43 minutes of uptime. Both Live TLS windows and full
Live measured 14324 bytes, below the unchanged 20480-byte floor. The pre-Live
status already retained that minimum. Stage 1 acceptance stays on hold.
Fresh-boot B then passed: TLS minima 25588 and 27636 bytes, full Live 25588 bytes,
normal video and no logger errors or drops. The overnight cause is unresolved.
Three more same-boot Live cycles passed at 25588, 25588 and 26612 bytes, with
normal-window minima of 31732 bytes between cycles, normal video and no logger
errors or drops. G-meter selection and ordinary saves to G-meter and dashboard
then passed, with responsive display and following Live at 26612 bytes.
No logger errors or drops occurred. Inclinometer navigation and both saves also
passed; following Live measured 25588 bytes with unchanged stack usage and no
errors or drops. Latest-to-Live then passed at 26612 bytes in both operation
windows. Seven full Live cycles in boot 13 now pass. Normal shutdown
and card-reader retrieval are complete. The [overnight analysis](sd_diagnostics_overnight_analysis.md)
found a transient low between 07:07 and 07:08 during sampled offline operation,
then reduced contiguous headroom at the 07:34 recorded reconnection.
A short hotspot outage after Latest reproduced 14324 bytes inside mqtt_connect
and subsequent Live. The no-media control passed at 31732 with a shorter outage.
The retained image connection is the leading hypothesis. JP approved an explicit
TLS close after still-body completion. JP built and tested the single stop call:
boot 17 Latest measured 25588 bytes and MQTT reconnect 31732, with no errors or
drops. This first patched sequence passes the unchanged 20480-byte floor.
Full Live after reconnect also passed in the same boot: lowest block 25588,
196 frames in 60.3 s, normal video, no errors or drops and unchanged stack margin.
Manual early Live exit, hotspot loss during Live and subsequent Latest passed.
Back also passed at 26612 bytes and 1427 ms. Automatic camera still-to-Live passed
at 25588 bytes, first frame 1036 ms, with normal video. The targeted cleanup checks
are complete; review this checkpoint before the remaining gates.
The overnight cause remains unresolved;
The same-session logging-OFF/ON pair passed on 2026-09-16: Live fps decreased
3.46% (within 5%), ON media block minima were 26612, and normal IMU average was
49.18 versus 49.26 Hz. No logger errors or drops; stack margin stayed 3988.
The hooks-only NVS/SD runtime stress also passed: 283 flash commits and 993 SD
records, no errors or drops, dummy key removed, writer stack margin 3476.
Card inspection verified all 993 stress records, the final counters and normal
SESSION_END with pending=0; a byte-identical backup is preserved. It also exposed
startup watchdog resets before boots 19 and 21's successful timed tests.
JP reproduced the pre-existing VS Code monitor-close freeze without flashing:
UI remains frozen over 30 seconds; the browser opens but status commands get no
reply. No automatic restart was observed. This remains separate from the earlier
watchdog markers. Use USB recovery and browser-only serial monitoring while the
host/core issue is investigated; no USB or watchdog settings were changed.
JP confirmed recovery in boot 23. Normal-limit forced rotation passed in boot 24:
one archive, generation 2, new file growth 449 to 1008 bytes, no errors/drops,
writer margin 3412. Small-limit rotation/pruning also passed: generation 6,
three archives, two pruned, no errors/drops and unchanged stack margin. Normal
limits were restored by command. Simulated full-card failure then passed:
logger disabled/parked as intended, final writer stack margin 3412, Latest still
working at 26612-byte HTTPS minimum and 1274 ms completion. Reboot recovery
also passed in boot 25: ready, generation 6 and three archives retained, file
growth 4108 to 4664. Interrupted-rename recovery then passed in boot 26:
JP confirmed the pause; generation 7, four archives and file growth 1259 to 1815
were observed. Partial-header salvage then passed its runtime check in boot 27:
generation 9, six archives, file growth 1284 to 1840, no errors/drops.
The supplied card copy now confirms archives 3 through 8, retained sequence
continuity, boot-25 append, boot-26 rename recovery, and boot-27 salvage into
generation 9. Archive 8 preserves the deliberate partial prefix plus a shutdown
record. Current ends with a clean shutdown at 15:11:53, pending=0.
Byte-identical copies and hashes are retained in docs/bench_data/sd_logs_2026-09-16_1511/.
The empty-header hook and intentional panic then passed runtime recovery in
boot 29: generation 10, seven archives, file growth 1397 to 1953, no errors/drops.
Paused file_bytes was a stale snapshot, not a measurement of the new empty file.
Evening SD inspection corrects this coverage: reason=new confirms missing-file
recovery after the empty-header pause; surviving-empty-file recovery is still untested.
Controlled watchdog triggering, restart and continued logging passed in boot 30:
diag_wdt timed out after about 5 seconds; the logger returned ready at generation
10. Follow-up confirmed file growth 80950 to 82629 and writes 7 to 10, no errors
or drops, largest block 31732 and writer margin 3588. Evening SD inspection verified
panic and task_watchdog reset classes and both valid main/writer breadcrumbs. The spring clock hook then passed
runtime checks: synced -> approx -> synced, file growth 85436 to 87852,
writes 15 to 23, no errors or drops and unchanged memory/stack minima.
The autumn hook also passed runtime checks: synced -> approx -> synced,
file growth 88414 to 90798 and writes 24 to 32, with no errors/drops and unchanged
memory/stack minima. Evening SD inspection verified both DST offsets, real-time
restoration, approximate-to-synced correction records and the final clean shutdown.
Eight files and hashes are backed up in docs/bench_data/sd_logs_2026-09-16_1802/.
No-card startup and PSRAM-writer terminal cleanup then passed in boot 31:
expected mount failure, parked writer, final stack margin 4116, Latest successful
in 4722 ms with HTTPS largest block 26612. JP confirmed normal board operation.
JP confirmed E:\sdcard as the actual card. Its logs matched the evening backup.
The 92055-byte current file is preserved as archive 10, and a flushed, closed
zero-byte current.log was prepared. Boot 32 then passed runtime recovery:
generation 11, eight archives, file growth 1257 to 1814 and writes 8 to 9, no
errors/drops, largest block 31732 and writer margin 3588. On-card
reason=empty_recovery still needs verification. The next morning's small-limit
attempt began with a 469869-byte current file, above the below-7000 precondition.
It pruned eight backed-up archives, then disabled on the 32768-byte content budget
with reserve_exhausted before rotating. Current remained 470005 bytes; the writer
parked with margin 3412. The initially offline hotspot was not the cause.
Normal reboot recovered logging in boot 33: ready, generation 11, current
470855 bytes (850 bytes appended), five writes, no errors/drops and largest
internal block 31732. Clock was still unknown at the early 17.5-second status.
Card inspection then verified generation 11 reason=empty_recovery, contiguous
boot-32/33 records, sync at boot-33 uptime 19372 ms and clean shutdown.
The 472389-byte log is backed up in docs/bench_data/sd_logs_2026-09-17_0828/
and preserved on card as archive 11. A flushed empty current.log is prepared.
Boot 34 starting status is verified: ready, synced, generation 12, 1276 bytes,
one archive, no errors/drops and largest internal block 31732. The small hook was
accepted; idle fill then produced generation 13 and rotations=1 before restoring
normal limits. Runtime rotation passes: no errors/drops, largest block 31732,
writer margin 3380. Normal restoration was accepted and logging continued.
Card inspection verified archive 12 at 8136 bytes, current reason=size,
continuous boot-34 sequences 1-31 and clean shutdown. Natural rotation content
verification passes. Originals are backed up in docs/bench_data/sd_logs_2026-09-17_0852/.
The 56-byte incomplete-tail fixture recovered in boot 35: ready, generation 13,
file growth 5099 to 5939, writes 7 to 10, unknown -> synced, no errors/drops,
largest block 31732 and writer margin 3588. Runtime check passes; on-card
TAIL_RECOVERY and prefix preservation await a combined inspection. JP requested
a finite remaining checklist: tail content, unrelated-file protection/no-clock
rotation and deep-sleep close/wake. JP explicitly deferred bad/unsupported-card
testing for version 1 on 2026-09-17; retain this limitation. The 2026-09-17 09:31 card inspection confirms short-tail repair with the original
prefix intact, complete sequences across boots 34–37, and clean deep-sleep close.
Boot 37 records deep_sleep reset, wake_code=2, both retained sleep/close breadcrumbs,
approximate time followed by sync (-118 ms), and a final clean normal shutdown.
JP confirmed touch-only wake and normal operation. These checks pass.
Backups and the preservation fixture are in
[09:31 evidence](bench_data/sd_logs_2026-09-17_0931/README.md).
Current was preserved as archive 13; an empty current and hashed unrelated files
are ready for the remaining preservation and unknown-clock rotation test.
Boot 38 passed the unknown-clock rotation runtime check: three rotations, two
prunes, generation 17, three archives, no errors or drops. The normal-limit hook
was sent and processed; logger stayed healthy. Final card inspection confirms
three FILE_OPEN reason=size records with unknown time, continuous sequences 1–61,
and clean shutdown. All six protected-file names and hashes match.
The agreed bench sequence is complete. See the
[Stage 1 acceptance checkpoint](sd_diagnostics_stage1_checkpoint.md).
JP acceptance remains pending; no further Stage 1 runtime repeat is proposed.
Do not repeat passed tests or start Stage 1B before acceptance.
Earlier, the large-file/small-limit run disabled logging and rejected restoration.
Subsequent guarded runs verified natural rotation and restoration. The original
header-hook reset yielded reason=new; the separately prepared surviving-empty-file
fixture later verified empty_recovery. Both histories remain in the bench record.
Stage 1 acceptance remains pending.
JP's local switch is 1 for diagnosis; the intended default remains 0. See the handoff.

The completed September 16 controlled performance pair used the same patched source with
DIAG_ENABLED=0 then 1, keeping probes, PSRAM selection and hooks-off settings
identical. This holds HTTPS cleanup constant. The original Stage 0 commit remains
the historical reference; the disabled build may retain static diagnostic storage.
See the [exact paired procedure](../src/diagnostics/STAGE1.md).
No performance pass is inferred from comparisons with another day.

## Design decisions

- **Files and rotation:** append across boots to `/logs/current.log`; rotate by size at **2 MiB**. Keep **30
  archives**, at most **62 MiB** of log content, with **16 MiB** free card space. Reserve `/images/` for future use;
  defer weekly rotation.
- **Clock:** Montreal local timestamps with offset, async sync, and `unknown`, `approx`, `synced` quality; 64-bit
  monotonic uptime measures durations.
- **Boot identity:** checked 64-bit NVS counter, once per boot including deep-sleep wake; optional installation tag.
- **Writer:** one low-priority task owns SD access; producers use a bounded queue. Flush pending data about every **2
  seconds** and prioritize failures.
- **Memory:** about **8 KiB** of explicitly allocated PSRAM event storage; internal queue controls, DMA descriptors and
  initial writer stack.
- **Breadcrumbs:** validated RTC no-init records, with separate main-task and writer phases.
- **Shutdown:** close in both power-down helpers, with at most **500 ms** caller wait inside their existing **1-second**
  display delay.
- **Serial and USB:** preserve `off`, `on`, `status`; extend `log status` and add list, get and abort through the
  existing parser. Use the same COM port and a Web Serial page; no USB stack change. Optional tail stays in Stage 3.
- **USB transfers:** one at a time, base64 with sequence and whole-file CRC. Pause current-file appends with close
  and reopen. Abort at **50% queue** or **5 seconds without progress**. Only current.log has an overall limit,
  initially **120 seconds**, confirmed against throughput in Stage 1B. Archives have no overall time limit.
  No transfer IDs or application acknowledgments. Logging, pruning and shutdown take priority.
- **USB rollout:** Step 0 checks connection before measurement probes. Stage 1B starts after Stage 1 acceptance.
  Stage 2 starts after Stage 1B acceptance.
- **Scope:** optional logging must leave the companion usable without a card. No generated SquareLine edits,
  network-policy changes, image and video storage, network uploads, remote controls, on-screen browser or panic core dumps.

## Records and clock

All UTF-8 records, including FILE_OPEN and BOOT, start with these fields in order:
`local, time, seq, boot, up_ms, level, event`. Use bounded lines and escaped and quoted strings.
Capture event time when observed and enqueued; serialize sequence assignment consistently across producers and
writer-generated headers. Sequence is per boot; operation IDs persist across rotation.

```text
local=unknown time=unknown seq=1 boot=214 up_ms=120 level=INFO event=FILE_OPEN format=1 generation=124
local=unknown time=unknown seq=2 boot=214 up_ms=121 level=INFO event=BOOT format=1 build=example reset=power_on
local=2026-09-14T17:39:45.471-04:00 time=synced seq=41 boot=214 up_ms=23023 level=INFO event=MQTT_CONNECT_BEGIN attempt=1 target=test
local=2026-09-14T17:39:50.474-04:00 time=synced seq=42 boot=214 up_ms=28026 level=WARN event=MQTT_CONNECT_END attempt=1 state=-2 elapsed_ms=5003
```

Examples are illustrative. FILE_OPEN identifies format and generation; BOOT identifies format and build and session and
reset and wake context. Repeat session and build and active-operation context at rotation; omit format version from
ordinary events.

Keep UTC and epoch internally; output Montreal time using `EST5EDT,M3.2.0,M11.1.0` (summer -04:00, winter -05:00). Rules
are compiled, so legal changes require a firmware update. The numeric offset resolves the repeated autumn hour and
permits UTC conversion.

| Quality | Rule |
|---------|------|
| `unknown` | `local=unknown`; no validated clock provenance. A plausible epoch or last shutdown time is insufficient. |
| `approx` | Retained time from a previously synchronized boot, including deep sleep, with validated RTC provenance; also after a confidence-invalidating clock discontinuity. |
| `synced` | Synchronized in this boot; include age of last sync in health records. |

Use asynchronous sync without delaying boot and media. Record sync and corrections against 64-bit uptime; never rewrite
earlier records or use raw 32-bit `millis()` for long-term ordering. Invalid RTC provenance means unknown until sync.
Sleep retains time with RC-oscillator drift; full PMIC power removal loses it. Future age rotation must exclude
approximate time.

Increment and check the boot counter in a dedicated diagnostics NVS namespace from internal-stack setup before writer
startup. Do not write NVS for each event or flush. An optional short installation-generation tag distinguishes NVS erase; on
persistence failure report it and use an explicitly nonpersistent session tag.

## Event catalogue

| Stage | Category | Fields and events |
|-------|----------|-------------------|
| 1 | BOOT and setup | Build, format, session, reset and wake reasons, CPU frequency, SD mount result and setup completion. |
| 1 | Resource snapshots | Free and minimum internal heap, largest internal block, free PSRAM, power source and battery voltage. |
| 1B | USB retrieval | USB_GET_BEGIN and USB_GET_END with filename, bytes, duration and result; current-file events bracket the snapshot and resumed appends. |
| 2 | Network setup | Wi-Fi and MQTT setup start, results and durations. |
| 2 | Wi-Fi | Scan and retry start and results, primary, secondary or unknown profile, association, IP acquisition, loss or change, disconnect code and label, current or last valid RSSI. |
| 2 | MQTT | BEGIN and END for every attempt with ID, real or test broker, port and TLS mode, duration, result and state; observed loss and recovery. |
| 2 | MQTT policy | Retry deferral and resumption during media, exhausted startup budget, subsequent recovery. |
| 2 | MQTT application | Image notification accepted or ignored with reason; subscription and important motion and calibration publish results; aggregate power and energy message counts and inbound age. |
| 3 | Stills | Request ID, Latest, Back or MQTT trigger, acceptance or refusal reason, network states, HTTP status and error, expected and received bytes, response, download, decode and total times, completion, cancellation or timeout. |
| 3 | Live | Session and trigger, acceptance or refusal, connection result, first-frame latency, unusual frame gaps, HTTP, TLS or decode failure, stop reason, frames, duration and average fps and existing timing summary. |
| 3 | UI | Accepted Latest, history Back, Live and navigation; screen changes, displayed red, orange or green state, loading timeout and return reason. |
| 3 | Power and motion | USB connection and loss, moving and stationary transitions and inactivity decisions. Stage 1 already records sleep, shutdown and clean close. |
| 1; expands in 2 and 3 | Health every 60 seconds | Available network and RSSI, screen and operation, memory, power and battery snapshots, last-sync age, queue high-water, drops and write statistics. Later hooks add message aggregates and cause-tagged maximum loop gaps. |
| 1 | Logger | Start, rotation and recovery, overflow and suppressed counts, slow and failed write, storage limits, disabled state, session end and serial status. |
| 2 | Bench controls | `off` and `on` requested and applied, `status`, and real or test mode. |

Keep actual attempt results; rate-limit identical repeated errors with suppressed counts. No raw IMU samples, payload
dumps, JPEG bytes, per-frame successes, touch coordinates or individual loop samples. Never store passwords, keys,
tokens, headers containing authorization, full URLs and query strings or raw MQTT payloads; use aliases and sanitized
errors. Local IP is useful. Do not mirror serial output wholesale.

Thresholds: **loop gap >1 second**, **Live frame gap >2 seconds**. Associate gaps with measured blocking spans and
attempt IDs; summarize explained gaps once and report unexplained excess. Separate TCP, TLS and CONNACK waits mean
the observed ~5-second MQTT failure is not every attempt's duration. A touch never processed during a stall has no
button record.

### Network observation rules (Stages 2–3)

Copy driver-event data from task-context `WiFi.onEvent` callbacks into the thread-safe queue; distinguish its timestamp
from later UI polling. Capture secure `lastError()` code and short text, plus HTTP or MQTT results before reuse; capture
first MQTT-loss state before `disconnect()` overwrites it. Record actual values, including -3 lost or -4 timeout where
observed. No TLS query for plain TCP. Inbound age measures application callbacks, not TCP traffic or PINGRESP; use unknown
before the first message. Publish and subscribe return values report library acceptance, not server acknowledgment
unless observed.

Keep hostname-based TLS with SNI and certificate hostname checking.

Log re-association and GOT_IP, matched profile, configured broker and MQTT recovery and any broker mismatch. Firmware's 30-second
`WiFi.begin()` retries apply only when `!g_wifiUpAtBoot`; normal mid-session recovery uses default-enabled
auto-reconnect. MQTT config occurs on initial and one-time late connection, not every reconnection. Existing `[NET]`
records are coarse; add detail without changing recovery policy.

## Storage and recovery

**Board-maker implementation reference:** use Waveshare's
[SDMMC example for this board](https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-1.8/blob/main/examples/esp-idf/09_sdmmc/main/sd_card_example_main.c)
as a starting point to save implementation time. Adapt its mounting, card information, file operations and unmounting
patterns where useful. Inspect the board support package behind `bsp_sdcard_mount()` for board initialization details.
This is an ESP-IDF example; adapt it to our pinned Arduino ESP32 core 3.1.3 and `SD_MMC` configuration below.
Keep the logger's append, archive preservation and graceful error handling rules: the demo uses truncating writes,
deletes an existing demo destination and aborts on some errors. Keep automatic formatting disabled.

Use the bundled `SD_MMC` in **1-bit mode**, explicit CLK=GPIO2, CMD=GPIO1, D0=GPIO3. Confirm board power and initialization
against the pinned core and board example. Use a user-prepared FAT32 card and `format_if_mount_failed=false`. Queue
early, mount after board power setup without waiting for networking. Report mount or allocation failure once; no tight
retry loop. Insert or remove the card with power off. A write failure disables writes for that boot.

Archive names in `/logs/` are exactly **`archive-NNNNNNNN.log` (8 decimal digits)**, e.g. `archive-00000124.log`.
Delete only regular files in that directory whose full name matches **`^archive-[0-9]{8}\.log$`**. Never delete current,
`/images/`, unrelated names or other directories.

- Reopen current for append. Read only bounded FILE_OPEN header, size and last byte; enumerate archive names with a
  bound. No body scan or age metadata. Diagnose excessive entries and unexpected oversized files; do not guess
  ownership.
- **Corrupt or partial header:** ignore its generation. Choose the next free archive number from matching archive
  names, rename current intact, and create fresh current with a valid header. Continue logging; disable it for this
  recovery only if archival rename or fresh-file creation or header writing fails. Never truncate or overwrite evidence.
- For a valid header with an incomplete final line, add a newline and recovery marker within the size cap; readers
  ignore malformed tails. An empty fresh file can receive its first header.
- Rotate before any record, header, repair or metadata exceeds 2 MiB. Flush and close, rename to an unused generation
  archive, then create current with the next header. Put the rotation reason in the new file if the old file is full.
- Resolve the next number before pruning; ordinary generation recovery uses valid current header plus exact archive
  names. Never overwrite or wrap onto an existing name. If interrupted rotation leaves current absent, create it at the
  next free generation; otherwise resume or recover it using the rules above.
- Prune oldest closed matching archives to make room for the 31st archive or restore free space; count current becoming
  an archive. Suspend logging if managed pruning cannot restore reserve space. Filesystem allocation overhead is outside
  the content cap. One active writer handle; archives remain immutable until retention deletes them.

Search current first, then newest archives. Copy soon after an incident. Weekly rotation, if justified later, needs
recoverable sidecar metadata written at rotation and initial valid anchor; no sidecar now.

## Writer, memory and breadcrumbs

Only the writer mounts, reads, writes, flushes, rotates and closes SD files. It never calls LVGL, MQTT, decoder or
NVS. Producers never wait on filesystem locks or serial input and output. The task isolates slow storage from UI and
touch and can drain during MQTT blocks; a loop-only writer would retain queued records until capacity is exhausted.

Explicitly allocate event storage with `MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT`; ordinary malloc and dynamic FreeRTOS
queues do not establish placement. Use an internal writer stack initially, sized from measured margin including
long-filename workspace. An external static stack via `xTaskCreateStaticPinnedToCore` is a later optimization requiring
an audit of SD, VFS, ROM and cache-off behavior and board tests, including concurrent screen and calibration NVS writes from
other tasks.

Use bounded batches, low priority and yields; validate core placement. Reserve capacity for important transitions, shed
routine summaries first and emit loss markers later. Avoid recursive logger-error queuing. Measure slow writes, queue
high-water, internal and DMA-capability minima, largest blocks and stack margin across mount, write and flush, TLS and
Live, not merely the ~50 KB internal heap left after a successful feed. Use peaks to size batches. If measured
contention warrants it, defer routine batches only during bounded TLS-connect phases (Stage 2), never entire Live
sessions; preserve queue priority and existing TLS behavior.

`RTC_NOINIT_ATTR` structure: magic and version, sequence and checksum, prior boot identity, phase, operation ID, uptime.
Separate main and writer slots; read before replacement, reject torn metadata and power-on remnants, restore the prior
or idle phase on completion and mark deliberate sleep and shutdown. Include reset and wake reasons. Stage 1 uses setup,
idle, SD and close phases; later hooks add `wifi_setup`, `mqtt_connect`, `image_fetch`, `live_tls_handshake`,
`live_frame`. No SD writes in panic handlers or ISRs.

## Shutdown and serial

Close centrally in `goToShutdown()` and `goToDeepSleep()`, before radio, peripheral and SD rails fall. Cover
battery-only Wi-Fi setup failure, USB-loss grace-period trip end, inactivity shutdown and motion-dependent sleep and
`TEST_POWER` variants. Fit the bounded close wait inside the existing display interval, preserve its remaining delay,
and proceed on timeout. If logger never initialized, close is a no-op. Expected radio-off events after close do not
reopen the log.

Retrieve logs through the existing USB cable with a single-file Web Serial page. Keep
Wi-Fi, MQTT, display and sensors running; the iPhone hotspot is unchanged. Close the
VS Code monitor, connect the page to the same COM port, download, disconnect, then reopen
the monitor. Use no companion network server or network transfer. The appendix compares
this shared-port approach with dual CDC and mass storage.

### USB retrieval commands (Stage 1B)

Use the existing `netBenchLoop()` parser: its 24-byte buffer holds 23 command characters
and a terminator. The commands fit. They do not modify or delete log contents.
Existing status commands use thread-safe writer snapshots. Commands still wait behind
a blocked main-loop call; file sending runs in the writer task.

| Command | Reply |
|---------|-------|
| `log status` | One `@@STATUS` line with boot, uptime, logger state, current size, newest archive, drops, card size, free space and file count. Extend the Stage 1 status snapshot. |
| `log list` | One `@@FILE` line per managed file, then `@@LIST_END`. |
| `log get current` | Download `/logs/current.log`. |
| `log get <generation>` | Download `archive-NNNNNNNN.log`; for example `log get 124`. |
| `log abort` | Stop the transfer and reply `@@ERR reason=aborted` after cleanup. Also confirm when already idle. |

Only regular files named `current.log` or matching `^archive-[0-9]{8}\.log$` inside
`/logs/` are eligible. Accept decimal generations from 0 through 99999999 and format
exactly eight digits. Reject overflow, signs, arbitrary paths and trailing text.
A valid number without a matching file returns `not_found`.

One transfer runs at a time. During it, accept only `log status` and `log abort` among
log commands; all other log commands return `@@ERR reason=busy`. Existing `off`, `on`
and `status` retain their behavior. Optional `log tail` stays in Stage 3 and uses the
same writer and bounded output machinery.

### Line format

Protocol records start with `@@`. Other text goes to the console panel. The following
example is illustrative; the base64 payloads are abbreviated.

```text
@@STATUS boot=214 up_ms=812345 logger=ready current_size=183422 newest=124 drops=0 card_mib=30436 free_mib=30102 files=12
@@FILE name=current.log size=183422
@@FILE name=archive-00000124.log size=2096980
@@LIST_END count=12
@@BEGIN version=1 name=archive-00000124.log size=2096980
@@D 1 <base64 payload>
@@D 2 <base64 payload>
@@END name=archive-00000124.log bytes=2096980 lines=14563 crc32=1A2B3C4D
@@ERR cmd=get reason=not_found
@@ERR reason=aborted
```

- Use ASCII protocol lines, maximum **240 bytes including the leading and trailing LF**.
  Send each complete line with one `write()` call. The leading LF separates a partial
  debug print; the page ignores empty lines.
- Each `@@D` contains up to **144 file bytes**, encoded as at most 192 base64 characters.
  Sequence numbers begin at 1 and must be consecutive. A full line is about 206 wire bytes.
- Buffer incoming browser reads until LF. Reads can split lines or contain several lines.
  Enforce the line limit for protocol parsing and bounded storage for ordinary text.
- Require strict standard base64: valid alphabet and padding, no embedded whitespace,
  and decoded length within the chunk limit. Reject malformed data.
- Use **CRC-32/ISO-HDLC**, the standard zlib CRC-32, over decoded file bytes in order.
  `@@END` contains eight hexadecimal CRC digits, byte count and data-line count.
  Validate these against `@@BEGIN` size and the received bytes before reporting success.
- Require a supported BEGIN version and a matching END filename. Missing END or any
  validation mismatch fails the download. Version 1 has no offset resume or per-line CRC.
- There is no transfer ID. Ignore data lines outside a BEGIN-to-END transfer. After
  `log abort`, wait for `@@ERR reason=aborted` before sending another get. If confirmation
  cannot be obtained, close the connection instead of issuing a retry into that session.

### Pacing and transfer bounds

Before every transfer line, and on each retry while waiting for space:

1. Abort if shutdown is requested or `USBSerial.isConnected()` is false.
2. Abort at **50% event-queue occupancy**, leaving half the queue as headroom for logging.
3. For **current.log only**, abort at the configured overall limit, initially **120 seconds**,
   measured from accepting get. Stage 1B's throughput gate may raise this limit.
4. Abort with `reason=stalled` after **5 seconds without a complete transfer line sent**.
5. Check `availableForWrite()`. If the complete line does not fit, yield and retry later.
6. Write once when space appears available. Abort if the return value is short.

Start the no-progress timer when get is accepted. Reset it only after a complete transfer
line is accepted by `write()`, not for status replies or ordinary debug output.
The writer task checks this independently of main-loop MQTT calls; the host continues
reading USB while those calls block. Keep applicable timers active during TX-space waits.

The space check does not reserve capacity. A concurrent print can cause a wait, so bound
work per writer turn and measure interference. Keep the existing global USB timeout.
Do not use periodic application acknowledgments.

A connection check reports the driver's state, not browser receipt. In core 3.1.3,
`isCDC_Connected()` returns true while its `connected` flag stays set and USB is plugged.
A sender waiting for free space may never enter the write timeout that clears that flag.
The no-progress timer handles this gap. The overall timer bounds only current.log's pause.
Archive downloads have no overall deadline; use the no-progress timeout, connection check
and `log abort`, while preserving the queue, pruning and shutdown rules.

### Writer ownership and aborts

The serial parser validates and posts requests. It never accesses the card. Status uses
an existing thread-safe snapshot. The writer reads bounded chunks between logging batches;
logging and close requests take priority. Control replies also use bounded output.

On abort, close the download handle and restore appending if it was paused. Do this
before attempting an error reply. Use `logger_busy` for queue pressure and `timeout` only
when current.log reaches its configured overall limit. Use `stalled` for a short write
or five seconds without progress, and
`read_failed` for a read error or premature EOF. A lost connection stops output; error delivery must not hold logging paused.

Pruning occurs at rotation. If it selects the archive being downloaded, abort and close
that reader before pruning. No open download handle survives removal of its archive.

Shutdown aborts the transfer immediately without sending a reply. Complete the bounded
close hook above; do not wait for USB output. Existing in-flight storage work
still follows that close deadline.

Record `USB_GET_BEGIN` and `USB_GET_END` with filename, bytes, duration and result.
For current.log, write BEGIN before the snapshot and END after appends resume. Shutdown
uses the existing close records rather than delaying close to send or enqueue replies.

With no host, retrieval is inactive with negligible cost: bounded state and existing
parser polling remain. No download work runs automatically.

### Downloading current.log

Pause appends rather than force rotation. Repeated downloads should not create small
archives and shorten retention.

1. Flush and close the append handle. Record the file size and open a read handle.
2. Send exactly that size. New events remain queued while appends are paused.
3. On success or any abort, close the reader and reopen for append, then drain the queue.
   An initial read-open failure takes this same cleanup path.
4. If reopening for append fails, disable logging for that boot under the existing
   write-failure rule. Never truncate the file as recovery.

Apply the 50% queue rule throughout the pause, including space waits. Bench acceptance
requires no event loss on the high-water abort.

### Web Serial page

One file: `tools/sd_log_browser.html`, with no build step or install.

- Connect and Disconnect buttons. Open at 115200, then request `log status` and `log list`.
- A file table, per-file Download, progress, and verified or failed result.
- Keep **Download current + newest 3** as a page-side loop of sequential gets. Validate
  each result before continuing. Test single-file retrieval before this convenience loop.
- Show non-protocol output in a console panel with a fixed history cap; discard oldest
  display entries when full. Do not retain unlimited partial lines or console text.
- Save normal browser downloads as `<boot>-<name>`, such as `214-archive-00000124.log`.
- On cancellation, send abort and follow the confirmation rule. On transport loss,
  invalidate any incomplete download and release browser stream locks and the port.

Check `isSecureContext` and `navigator.serial` before offering Connect. Request port
permission from a user click. Start with a disk-opened page in desktop Chrome or Edge;
use localhost if that environment does not permit it.

DTR and RTS settings take effect after the driver opens the port. The page cannot
prevent the first driver control-line change. Record the settings used in Step 0.
Compare boot and uptime with a prior observation when one exists; the first connection
has no automatic pre-open reference.

### Optional tail (Stage 3)

`log tail [n]` defaults to **20** records, maximum **100**, with a **16 KiB** byte budget.
The writer reads current backward in chunks with logging and close priority. Return fewer
records with a limit notice; no full scan, archive traversal or unbounded allocation.
Snapshot file identity, include format context and report busy or unavailable. Use a
bounded output queue and small paced chunks, without bulk printing or flushing.

## Limits and unverified points

- **DMA buffer:** a temporary internal buffer for PSRAM SD writes is likely but unverified in this SDK; frequency, size and direct-DMA behavior are driver-specific. Check the matching implementation or allocation traces: before and
  after heap snapshots can miss transients.
- **Brownout breadcrumbs:** restart and deep-sleep retention is supported; brownout survival depends on voltage and
  reset domain. Discard invalid data and power-on remnants and interpret phases alongside reset reasons.
- **Durability:** power loss can lose queued and unflushed records or corrupt FAT and rename state. Two-second flushes
  are targets. Close deadlines cannot cancel an in-flight write; deferral and full queues increase unsaved history.
  A missing END or clean-close marker alone does not diagnose a crash. Downloading current.log holds recent events
  in RAM until appends resume.
- **TLS errors:** hostname DNS failure in core 3.1.3 can return before updating `lastError()`, leaving stale or zero data; do not infer a fresh TLS cause from it.
- **Retention:** daily growth, record sizes and powered hours need measurement; appendix figures are conditional.
  Profile and recovery evidence may help explain car faults but does not establish their cause in advance.
- **USB liveness:** driver connection state and successful writes do not establish browser receipt. Test a forced
  read stall; check the no-progress timer and current.log's configured overall timer between bounded operations.
  Neither can cancel an in-flight filesystem call. Main-loop MQTT blocking can delay serial command parsing.
- **USB environment:** Step 0 verifies reset behavior and disk-page permission on the installed browser and Windows
  driver. Post-open DTR and RTS settings cannot prevent the driver's first change.
- **USB performance:** throughput, queue headroom and serial contention need same-session measurements. Base64 and
  CRC validate the file; they do not preserve every ordinary debug print.

## Staged rollout and bench gates

### Before firmware work

The owner reviews this finalized proposal and commits all seven documents as the fixed
reference for code review. This editing pass makes no commits.

- `docs/sd_diagnostics_plan.md`
- `docs/sd_diagnostics_plan_review.md`
- `docs/implementation_plan.md`
- `docs/ESP32_SD_Log_USB_Investigation.md`
- `docs/sd_usb_log_retrieval_plan.md`
- `docs/sd_usb_log_retrieval_plan_codex_review.md`
- `docs/sd_usb_log_retrieval_plan_counter_review.md`

**Every stage:** the owner compiles and flashes from VS Code. Stage 1 touches both `companion.ino` and `src/`; before
each rebuild delete **`build/build_amoled-1-8/sketch/companion.ino.cpp`** per the stale-build rule in
[CLAUDE.md](../CLAUDE.md). The assistant does not compile or flash.

### Step 0: USB connection check

Build only the page console: connect, show text and send a typed command. No firmware
change. The existing `status` command reports uptime.

1. Request status in VS Code and retain its uptime, then close the monitor.
2. Connect the page and request status. Record whether uptime restarted or boot output
   appeared. Record the browser version and DTR/RTS settings.
3. Disconnect, reopen VS Code and request status. Record any reset on that transition.
4. Repeat five times, including during Live. Check disk-page permissions as part of this.

Step 0 determines whether this setup resets the board. If it does, review the observed
behavior with the owner before proceeding; changing signals after open may not fix it.

**Recorded result, 2026-09-15:** browser driver defaults caused a visible reset on
disconnect. Explicit DTR=true, RTS=false passed repeated cycles and a full Live
cycle without a visible interruption. Use those explicit settings for the browser.
Compare status uptime before and after reconnecting in that same page; switching
to VS Code is not required for this check. The VS Code close freeze remains open,
so the original VS Code round trip is not marked passed.
See [bench results](sd_diagnostics_bench_results.md).

### Stage 0 — measurement probes

Make a minimal probe-only change to the current firmware. Record minimum free
internal heap and the lowest observed largest free internal block, including
inside blocking MQTT, HTTPS and Live TLS connects and throughout Live. Use the
same internal-memory capability filters in every measurement.

Use `heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL)` for the since-boot internal
heap low-water mark; no sampling is needed for that value. The largest internal block
has no low-water API. Sample `heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)`
every **10 ms** using the existing task-dispatched esp_timer, including while the loop blocks.
Report the interval, periodic count, maximum gap and scan cost for each window.
Boundary readings supplement sampling; incomplete coverage is not a passing memory test.
Print one summary per window and no per-sample output.

Also report minimum and arithmetic-mean IMU rate from the existing `sampling_frequency`
in normal-operation windows of up to **60 seconds**. Exclude setup, media and measured
connects; skip the first interval after returning to normal. Stage 1B must exclude USB
file transfers using the same eligibility input. Do not change the IMU calculation.
Keep the probes identical for Stage 1. No SD logger or operational event hooks are added.

See [probe fields and baseline sequence](../src/diagnostics/README.md) and the
[Step 0 console page](../tools/sd_log_browser.html).

The owner flashes this probe-only build in VS Code and measures the baseline:
Live fps and frame gaps, Latest total time, internal heap minimum and lowest
largest internal block, plus normal-operation IMU minimum and average rate. Stage 1
retains these same probes unchanged. Compare
baseline and logging enabled back-to-back in the same sitting, as required by
CLAUDE.md; figures from another day are not comparable.

### Stage 1 — basics only; stop for bench acceptance

JP authorized this stage after accepting the recorded Step 0 and Stage 0 results.
The implementation is prepared in src/diagnostics; see the
[Stage 1 handoff](../src/diagnostics/STAGE1.md). Bench results now support the
[acceptance checkpoint](sd_diagnostics_stage1_checkpoint.md); JP acceptance is pending.
The initial writer used a 6144-byte internal stack. The validated B configuration
uses an 8192-byte PSRAM stack and internal TCB on core 0 at priority 1. Other choices are
a PSRAM queue of at most 8192 bytes with four reserved slots, a 1024-byte PSRAM
formatter, and 20 MHz SDMMC. Directory scans stop after 256 entries and yield
every eight. Writes run in batches of at most four records, with 20 ms waits.
Writes or flushes over 100 ms count as slow; clock discontinuities over 2 seconds
lower confidence. These choices require the planned measurements.

Implement these basics:

- Mount with formatting disabled, PSRAM event queue and writer task.
- Append and common headers, size rotation, pruning and recovery.
- Boot counter, clock quality states, asynchronous sync and Montreal rule.
- BOOT and reset-reason records, RTC breadcrumbs and both shutdown close hooks.
- Minute health record and logger state in serial status.

Carry-forward implementation notes (not implemented in Stage 0):

- Add a `DIAG_TEST_HOOKS` pause after creating current.log, before writing its header,
  with a short-prefix variant to test salvage of a real partial header.
- The task watchdog checks core 0 idle (5 seconds, panic). A writer on core 0 must
  block or yield in long loops, including listing, pruning, backward tail reads and USB transfers.

No network, image, Live or UI event hooks yet. Health uses available snapshots;
event-derived fields remain unavailable until their stages. Storage works before
time sync. Measurement probes remain separate from operational event hooks.

#### Test hooks

Provide a compile-time `DIAG_TEST_HOOKS` flag, default **0** and off in normal
builds. Test-only triggers and limit overrides exist only when enabled. Each
hook is explicitly triggered and logged as a test, never armed automatically
on reboot. Restore the normal configuration after fault tests; performance and
normal-use drop measurements use the normal limits with test hooks off.

| Hook | Gate test using it |
|------|--------------------|
| Set the clock just before either DST transition; pause real sync during the test and label the injected clock source as test | Clock gate: both Montreal offset transitions, sync corrections and clock quality. Re-enable real sync afterward. |
| Trigger a controlled panic with a known active breadcrumb | Breadcrumb gate: recover prior phase and boot identity, and check panic reset reason. |
| Trigger a real task-watchdog reset using a registered test task that stops feeding the watchdog | Breadcrumb gate: watchdog reset reason and RTC recovery; do not substitute a plain software restart. |
| Stop the writer after archival rename and before creating new current | Storage gate: owner resets or power-cycles the backed-up test card at this boundary; next boot must create current and preserve the archive. |
| Override file-size, archive-count and reserve limits with small test values | Storage gate: rapid rotation and oldest-archive pruning, including operation without a clock and preservation of unrelated files. |
| Simulate insufficient free space and a full-card write error, without filling or deleting unrelated card data | Storage gate: pruning, reserve exhaustion, full-card error reporting and continued companion operation. |

Hook code belongs to Stage 1 test infrastructure, not the deferred network or
media event hooks. Controlled brownout and power-loss tests remain separate;
a simulated panic or watchdog reset does not test electrical retention.

#### Proposed pass and fail limits

The owner confirms these limits **before measuring the baseline**:

- No new allocation failures or TLS failures attributable to logging.
- During TLS connects, the largest free internal block must stay at or above
  **20 KiB (20,480 bytes, approximately 20 KB)** in adequately covered measurements.
  Any observed dip below the confirmed threshold fails the memory gate.
- Live fps must be within approximately **5%** of the same-session baseline.
  Record paired frame gaps and Latest time as well; investigate results outside
  that band before acceptance rather than attributing them automatically to code.
- **Zero queue drops in normal use**, with normal limits and fault hooks off.
  Report intentional fault-test drops separately.

Record internal heap minima, writer stack margin, queue high-water and slow writes.
Review measured stack margin against observed peaks; a stack overflow or new
starvation blocks acceptance. If the probe-only baseline already fails a proposed
limit, resolve that with the owner before proceeding, rather than silently
lowering the limit or blaming the logger.

#### Stage 1 gate — run in this order

1. **Memory and performance:** compare logging enabled with the same-session
   Stage 0 baseline. Check internal minima and largest blocks during Live, all
   TLS connect paths and SD operations; fps, frame gaps, Latest time and touch.
   Check writer stack margin, queue drops, high-water and slow writes. Exercise
   repeated media cycles and concurrent screen and calibration NVS activity.
   Investigate the DMA-buffer hypothesis. Any external-stack alternative needs
   separate validation.
2. **Storage and rotation:** test repeated-boot append and headers, corrupt-header
   salvage, incomplete tails, rotation and pruning with small limits and no
   clock, interrupted rename and create, and every shutdown close path. Check
   no-card and simulated full-card behavior. JP deferred bad-card and unsupported-card
   tests for version 1 on 2026-09-17; record these as untested, not passed. Preserve
   dummy `/images/` data and unrelated files. Restore normal limits afterward.
3. **Clock and breadcrumbs:** test cold boot, deep-sleep wake, approximate time,
   real sync and corrections, both injected DST transitions, and controlled panic
   and watchdog resets. Check breadcrumb validation and distinct reset classes;
   test brownout retention separately where practical.

Run one test at a time and back up the card before deliberate power interruption.
Record pass and fail results and measurement coverage. **Stop: Stage 1B starts only after
the owner accepts Stage 1 results. Stage 2 starts only after Stage 1B acceptance.**

### Stage 1B: USB retrieval

Start only after Stage 1 acceptance, using its writer, storage layout and rotation.
The owner compiles and flashes in VS Code using the build precautions above.

Run gate tests one at a time:

1. **Integrity:** compare an archive byte for byte with its card-reader copy. For current,
   compare the download with the first N bytes of the later copy, where N is BEGIN size.
   The remaining file may contain later records. Check size and CRC in both cases.
2. **Throughput:** measure decoded-file bytes per second and wire throughput separately
   for a 2 MiB archive. Check that the proposed **120-second current.log limit is at least
   three times the measured 2 MiB download time**. If not, raise the configured limit to
   at least that value before accepting Stage 1B. Record the accepted limit and use it
   in test 5. After single-file tests pass, time current plus newest three.
3. **Non-interference:** download during Live and MQTT `off` and `on` cycles. Meet Stage 1's
   same-session performance and memory limits. Allow no additional UI stalls beyond
   existing MQTT blocking, no new watchdog resets and no queue drops. Check internal heap,
   largest internal block and writer-stack margin.
4. **Debug and validation:** verify ordinary prints remain visible during retrieval.
   Use a page-side test switch to drop or damage one data line. Validation must reject
   the file, and a later retry must succeed.
5. **Abort and liveness:** close the page, send abort, deliberately stop browser reads,
   and test USB unplugging. Use a **battery-equipped board** for the unplug case and
   record the board and power source. Without a battery, unplugging is a power-loss test,
   not a transport-disconnect test; back up the card before such deliberate interruption.
   With a battery, USB loss enables inactivity shutdown, so account for that timer.
   Observe connection state and verify the five-second no-progress abort with
   `reason=stalled`. Add a **page-side slow-read test switch** to keep current.log making
   progress at intervals shorter than five seconds while exceeding its overall limit.
   Keep queue occupancy below 50% for this case. Verify `reason=timeout` at the accepted
   current.log limit (initially 120 seconds). An archive making continued progress must
   not abort at that overall limit. No TX-space wait may bypass an applicable timer.
   Confirm appending resumes and the next download succeeds. Verify abort confirmation ordering.
6. **Current pause:** generate events while retrieving current.log. Exercise the 50% queue
   abort and verify zero dropped events, resumed appends and an error reply when connected.
7. **Pruning:** use the existing small rotation and retention test limits. Make pruning
   select the archive being downloaded. Confirm its reader is closed before deletion,
   the transfer fails cleanly and logging continues.
8. **Repeated downloads:** repeat successful and aborted transfers. Check heap, largest
   block, writer stack and file handles for leaks or accumulating resource loss.
9. **Round trip:** disconnect the page and reopen VS Code. Confirm output and status,
   and check for resets against the Step 0 observations.

Keep the shutdown-abort and read-error handling rules, but add no dedicated
shutdown-during-download bench test or SD read-failure injection hook in version 1.

### Stage 2 — network evidence

Start only after the owner accepts Stage 1B results.

Add Wi-Fi events, profiles and UI-independent driver timing, every MQTT attempt, first-loss and error snapshots
before cleanup, inbound age and application events, retry deferrals, recovery, TLS-phase breadcrumbs and markers.
Bench-test hotspot loss and existing serial `off` and `on`, including Live during MQTT outage and Latest after recovery;
compare duration and responsiveness. Conditional TLS-aware writer deferral follows measurement.

### Stage 3 — operation context, then optional tail

Add still and Live lifecycle, buttons and visible screen state, power and motion transitions and cause-tagged loop gaps.
Exercise accepted and ignored requests, normal completion, errors, cancellation and timeouts and repeated media and NVS cycles.
Then add optional tail; stress bounded reads and output while logging and verify commands and touch are not starved.
Repeat paired performance checks after changes.

### Stage 4 — car use and retention tuning

Measure powered hours, bytes per hour, trip and day, plus incident frequency; tune file size and count from growth.
After an incident, download current and recent archives over USB, or power down and copy `/logs/` with a card reader.
Inspect Montreal time and boot markers. Weekly rotation remains deferred pending demonstrated need.

## Appendix — evidence and retention arithmetic

Counter-review: 2026-09-14, firmware `main` at `4ae23bf`. `sketch.yaml`, `amoled-1-8` and generated
`build/build_amoled-1-8/build.options.json` agree: ESP32 **3.1.3**, CPU **240 MHz**, PSRAM enabled, LoopCore=1,
EventsCore=1. Use this profile rather than unrelated generic settings in `.vscode/arduino.json`.

Generated `build/build_amoled-1-8/sdkconfig` equals installed ESP32-S3 SDK config; SHA-256:
`0B02347898DFEF469DF8D2AC3220B1CA2CA1501C55D0E87E4A646E8F962B2D6E`.
SDK: `esp32-arduino-libs/idf-release_v5.3-489d7a2b-v1` under
`%LOCALAPPDATA%/Arduino15/internal/esp32_esp32-arduino-libs_idf-release_v5.3-489d7a2b-v1_80ffc9027a/esp32s3`.
Core: `%LOCALAPPDATA%/Arduino15/internal/esp32_esp32_3.1.3_e149c3cd368ed269`.

Inspected core sources: `libraries/SD_MMC/src/SD_MMC.h`,
`libraries/NetworkClientSecure/src/{NetworkClientSecure.cpp,ssl_client.cpp}`, `libraries/WiFi/src/STA.cpp`,
`libraries/Network/src/NetworkEvents.cpp`; SDK `esp_attr.h` and `sdmmc_cmd.h`. Public headers and config were checked,
not the exact compiled transfer implementation. No hardware or SDK changes were made. Firmware evidence is in
`pin_config.h`, `companion.ino`, `src/net/net_module.cpp`, `src/screen_memory/screen_memory.cpp` and
`calibration.cpp`.

| Verified configuration | Consequence |
|------------------------|-------------|
| `CONFIG_FATFS_ALLOC_PREFER_EXTRAM=y`, `CONFIG_FATFS_PER_FILE_CACHE=y` | FATFS and its cache prefer external RAM. |
| `CONFIG_FATFS_LFN_STACK=y`, `CONFIG_FATFS_MAX_LFN=255` | Budget caller-stack filename workspace. |
| `CONFIG_FATFS_SECTOR_4096=y` | Buffer budgeting cannot assume 512-byte FATFS sectors. |
| `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096` | Allocations <=4096 bytes prefer internal RAM, with fallback. |
| `CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL=0` | No configured internal reserve protects DMA allocations. |
| `CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y` | Explicit external static stacks permitted; ordinary task stacks stay internal. |
| PSRAM instruction and rodata relocation disabled | Cache-off restrictions remain. |
| `CONFIG_NEWLIB_TIME_SYSCALL_USE_RTC_HRT=y`, `CONFIG_RTC_CLK_SRC_INT_RC=y` | Retained sleep clock uses internal RC. |
| `CONFIG_MBEDTLS_HAVE_TIME_DATE` unset | Clock sync does not enable certificate-date checks; hostname verification still applies. |

References: [board](https://www.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.8),
[SDMMC](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/sdmmc.html),
[Quebec time](https://www.quebec.ca/gouvernement/portrait-quebec/changement-heure),
[IDF 5.3 time](https://docs.espressif.com/projects/esp-idf/en/v5.3/esp32s3/api-reference/system/system_time.html),
[external RAM](https://docs.espressif.com/projects/esp-idf/en/v5.3/esp32s3/api-guides/external-ram.html). Match online
examples to the pinned build.

At **250 bytes per minute**, 24-hour health logging is `250 * 1,440 = 360,000 bytes/day = 0.3433 MiB per day` (decimal
**360 KB**, not KiB).

| Total growth | Fill 2 MiB current | 30 full archives (60 MiB) |
|--------------|--------------------|--------------------------|
| 0.3433 MiB per day, health only | 5.83 days | 174.8 days |
| 1 MiB per day | 2 days | 60 days |
| 2 MiB per day | 1 day | 30 days |
| 5 MiB per day | 9.6 hours | 12 days |

Current adds zero to nearly one file. At **1 MiB**, durations halve and the cap becomes **31 MiB**. Two powered hours
give about **30,000 health bytes**; other events and larger records add volume. Above 1 MiB per day, two days normally
exceed a 2 MiB file; **8 MiB** is a later sizing option with count and budget adjusted explicitly.

### HWCDC build facts (USB review, 2026-09-15)

Checked against `sketch.yaml`, firmware, core 3.1.3 and the generated `sdkconfig`.
The generated SDK configuration matches the installed configuration.

| Fact | Evidence | Consequence |
|------|----------|-------------|
| Hardware USB Serial/JTAG is active | `USBMode=hwcdc`, `CDCOnBoot=cdc`; firmware defines `HWCDC USBSerial` | A needs no USB configuration change. |
| HWCDC ignores the baud argument | `HWCDC::begin(baud)` does not use it | Open the page at 115200 for compatibility. |
| A write holds the TX mutex for its buffer | `HWCDC.cpp:418-488` | One call protects a protocol line against other HWCDC writes. Low-level console and panic output can bypass this mutex. |
| Mutex acquisition has a 100 ms timeout | `tx_timeout_ms=100`; failure returns zero | Keep writes short. This is not a total time limit for an entire write. |
| The default TX ring buffer is 256 bytes | `setTxBufferSize(256)`; `availableForWrite()` around line 406 | The space check is best effort. It can wait for the mutex and does not reserve space for the next write. |
| Disconnected writes may report full size despite discarded bytes | `write()` calls `flushTXBuffer()` when disconnected | Check connection state and short writes. Verify the received size, sequence and CRC on the page. |
| FatFs file locking is disabled | `CONFIG_FATFS_FS_LOCK=0` | Close the append handle before opening current.log for reading. Enabling locking would not permit duplicate opens involving write access. |
| Dual CDC and MSC are compiled capabilities | `CONFIG_TINYUSB_CDC_MAX_PORTS=2`, `CONFIG_TINYUSB_MSC_ENABLED=y`; board menu requires USB-OTG for MSC | B and C require TinyUSB on the existing USB connection. They are not active now. |

### USB options comparison

#### Option A: shared COM port (recommended)

- Preserve USB mode, board profile and upload workflow.
- Extend the existing serial parser. The planned SD writer owns all card access.
- Continue normal debug output in a browser console panel.
- Keep commands usable from a terminal or a later Python script.

Windows gives the COM port to one application at a time. Opening or closing it may
reset the board; Step 0 checks this before logger implementation.

Throughput is unmeasured. About 206 wire bytes carry 144 file bytes, so a 2 MiB file
uses about 3.0 MB on the wire. At 100,000 wire bytes per second, allow roughly 30 seconds.
Download current and recent archives rather than all 30 by default.

#### Option B: two CDC ports (rejected)

Core 3.1.3 already provides `USBCDC(0)` and `USBCDC(1)` with descriptors. A custom
composite USB stack is not needed. It would let the VS Code monitor stay open, but:

- Switching from HWCDC to TinyUSB replaces the current USB Serial/JTAG connection.
  The two controllers share the internal PHY on this connection.
- Existing `HWCDC USBSerial` declarations must migrate to the application CDC type.
- A broken application USB stack may require BOOT and RESET for upload recovery.
- The application's TinyUSB console starts with that stack. Early and panic output
  cannot be relied on through it. ROM USB console behavior is a separate facility.
- Hardware JTAG over this same connection is lost while TinyUSB uses the internal PHY.

The extra migration and validation are not justified for version 1.

#### Option C: USB mass storage (rejected)

MSC needs TinyUSB and its migration work. Writable MSC also needs exclusive ownership:
flush, close and unmount the firmware filesystem before Windows access, then remount
after eject. Windows writes and cached writes make an incorrect handoff a corruption risk.

Core provides `USBMSC::isWritable(false)`. Read-only MSC prevents Windows writes, but
Windows still caches a FAT view that firmware logging would change. Logging would need
to freeze for the session, or a separate snapshot would be needed. Neither is simpler
than A for this version.

## Review resolution

| ID | Outcome and resolution |
|----|--------------------|
| M1 | Agree: size-only storage; clock setup in Stage 1. |
| M2 | Refine: retained time is approximate until current-boot sync; epoch plausibility alone is insufficient. |
| M3 | Agree and refine: bounded metadata reads; salvage corrupt headers by archival rename and fresh current. |
| M4 | Agree: compiled Montreal rule and explicit offsets; rule changes need firmware updates. |
| S1 | Refine: 2 MiB files, 30 archives and 62 MiB total; measured growth controls retention, generations still needed. |
| S2 | Refine: checked boot counter and fallback; reject claims that screenMem necessarily writes more. |
| S3 | Agree: header-only format version; writer isolates input and output, queue buffers main-loop stalls. |
| A1 | Refine: separate validated phase slots; reject guaranteed brownout survival or causal proof. |
| A2 | Refine: explicit PSRAM queue and internal initial stack; DMA copying remains unverified. |
| A3 | Refine: errors before cleanup, DNS-stale lastError caveat, hostname TLS preserved. |
| A4 | Refine: both helpers and all callers; close wait inside existing delay, before rail teardown. |
| A5 | Refine: profile and broker recovery correlation; reject “no trace” since coarse `[NET]` exists. |
| A6 | Agree with bounds: existing parser, snapshots, writer-owned tail and chunked output. |
| A7 | Agree: attribute measured gaps and keep unexplained excess. |
| USB | Accepted counter-review merged: shared serial, bounded current pause, 50% queue, 5 s stalled and current-only overall limit (initially 120 s, at least 3× measured 2 MiB time); Stage 1B includes slow reads and battery-aware unplugging. |
