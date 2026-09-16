# Stage 1 SD logging — JP bench handoff

Status: JP built and flashed Stage 1 on 2026-09-15. The initial readiness and
normal-operation check passed: ready, synced, growing file, no drops or errors.
Latest succeeded but its 14836-byte largest internal block failed the 20480-byte
memory floor. The no-card comparison recovered to 28660 bytes. Stage 1 acceptance
remains on hold. The writer-start-order experiment was tested and still measured
14836 bytes. JP's retained-snapshot test passed late USB retrieval; Latest still
measured 14836 bytes. The writer-creation interval used 6528 internal bytes.
Mount readings overlap Wi-Fi startup and do not isolate SD allocation costs.
JP confirmed a battery-equipped board and 16 GB card. The assistant did not compile
or flash; JP builds and flashes from VS Code. No Stage 1B file download or Stage 2/3
operational event hooks are included.

The accepted plan is [sd_diagnostics_plan.md](../../docs/sd_diagnostics_plan.md).
The baseline is commit `f406063` on `sd-diagnostics`; measurements are in
[bench results](../../docs/sd_diagnostics_bench_results.md).
The existing probe source and measurement windows are unchanged.

## Next bench test: accepted writer-stack A/B experiment

JP accepted [Claude's experiment review](../../docs/sd_diagnostics_memory_experiment_review_claude.md).
All seven required changes are implemented. JP built and tested A and B with hooks
off on 2026-09-15. A reproduced 14836 bytes; B measured 26612 bytes during Latest,
passing this run's 20480-byte memory floor. Both used 4204 bytes of writer stack.
Placement checks passed, with the TCB internal in both builds. On 2026-09-16,
B completed Live normally after about 11 hours 43 minutes of uptime, but both
Live TLS windows and full Live measured 14324 bytes, below the 20480-byte floor.
The pre-Live status already retained that low value. A fresh-boot repeat (boot 13)
passed at 25588 bytes over full Live, with TLS minima 25588 and 27636 bytes.
JP reported normal video in both runs. Three further same-boot Live cycles passed
at 25588, 25588 and 26612 bytes, with normal video, no errors or drops, unchanged
stack usage and 31732-byte normal-window minima between cycles.
G-meter selection and both ordinary NVS saves passed without reducing the
recorded memory minima. Following Live passed at 26612 bytes with normal video.
The inclinometer route and both preference saves also passed; following Live
measured 25588 bytes, with normal display/video and no errors or drops.
Latest-to-Live also passed at 26612 bytes in both operation windows, with normal
behavior and unchanged stack and historical memory minima. Seven full Live
cycles now pass in boot 13. The overnight cause remains unresolved.
JP supplied the SD log. Analysis located a transient low between 07:07 and
07:08, before the 07:34 recorded reconnection and persistently smaller blocks.
The short-outage test after Latest reproduced 14324 bytes inside mqtt_connect,
before Live, and in both subsequent Live TLS windows. Next is the fresh-boot
hotspot control without any preceding image or Live request.
Cleanup and hooks-phase results are pending. Stage 1 is not accepted yet.
The 20480-byte gate and Stage 0 probes are unchanged. No commit until JP confirms.

Set the switch in [diagnostics_config.h](diagnostics_config.h):

| Build | DIAG_WRITER_STACK_PSRAM | DIAG_TEST_HOOKS | Expected stack status |
|-------|-------------------------|-----------------|-----------------------|
| A, default | 0 | 0 | internal, 6144 bytes, external=0 |
| B, experimental | 1 | 0 | psram, 8192 bytes, external=1 |

Keep DIAG_ENABLED=1. B uses a file-scope internal StaticTask_t and an explicit
SPIRAM | 8BIT stack. There is no fallback. Failed creation frees only the unowned
stack, reports create_failed, and disables logging while the companion continues.
A keeps dynamic creation/deletion. B cleans up file/card/formatter, captures the
final margin, acknowledges close, then suspends permanently. A stray resume
immediately suspends again. Stack, TCB and task runtime state remain until reboot.

The writer must never initiate flash, NVS or partition calls, or pass stack-local
data buffers to raw SDMMC host/command APIs. Normal sector wrappers remain in use.

### Current next test: fresh-boot hotspot recovery without prior media

The short-outage test after Latest reproduced the 14324-byte memory failure
inside mqtt_connect run 2 (618 ms), before any Live. Live later also measured
14324. Functional recovery and video succeeded, with no reported logger error.
This narrows timing to MQTT reconnection in this sequence; it does not identify
the allocation owner or prove a leak. No overnight wait is needed to reproduce
this measured symptom. Details are in the bench record and overnight analysis.

Run one control, changing only the prior media history:
1. Shut down normally and restart the same B firmware, hooks off, card installed.
2. Wait for green Wi-Fi/MQTT. Connect the console and send log status.
   Do not press Latest, Live, Back or change screens.
3. Turn the actual hotspot off for about 90 seconds. Confirm offline and send
   log status while offline.
4. Turn the hotspot on. After green MQTT returns, send log status and stop.
   Do not start Live in this control.

Send the full capture, especially mqtt_connect and normal probes. No reflash,
card removal or serial MQTT off/on command is needed. Stop and preserve evidence
on errors or a sub-20480 probe.

Source clue: image completion calls HTTPClient::end, but pinned core 3.1.3 keeps
the client open when reuse is allowed. Screen unload frees image buffers without
explicitly stopping that secure client. prepareForRequest explicitly stops it,
and Live stops it on exit. A retained image connection is therefore a candidate
interaction, not an established cause. This control separates MQTT recovery
alone from recovery after a still-image request before choosing a code change.

Stage 1 remains unaccepted, with the 20480 floor and Stage 0 probes unchanged.
Hooks stress and paired performance acceptance remain paused. Local B is still
selected for diagnosis; intended default remains 0. No firmware edits or commits.

### Exact first paired test (reference; initial captures received)

Same battery-equipped board, card, hotspot and Latest image, in one sitting.
Close the web port before each VS Code upload. Do not run fault commands.

1. Build and flash A with both switches at 0. Let Wi-Fi and MQTT reconnect.
2. Connect the web page with explicit DTR=true, RTS=false. Send log status.
   Require ready, hooks=0, active writer, valid placement, internal stack and
   tcb_internal=1. Press Latest once, wait for completion, return to the normal
   screen, then send log status again.
3. Change only DIAG_WRITER_STACK_PSRAM to 1. Leave hooks=0. Build and flash B
   immediately in the same sitting.
4. Repeat log status, Latest, log status. Require ready, active writer, valid
   PSRAM placement and tcb_internal=1.
5. Send both complete captures labelled A and B: LOG, LOG STACK, LOG MEM, PROBE,
   Latest total time and image size, plus any visible issue.

No startup capture is required. If A changes substantially from about 14836 bytes,
investigate the new baseline before assigning a gain to B. The different stack
sizes and static TCB compare practical configurations, not only placement.

B must meet 20480 bytes for image_https largest_min with adequate sampling,
without allocation/TLS errors, drops, storage errors, corruption or resets.
Passing Latest does not accept Stage 1. Stop on failure; restore switch 0 for
ordinary use. Send the pair before moving to Live.

These edits are under src/, not companion.ino. Changing the switches does not
require deleting the generated sketch. After any future companion.ino edit,
delete build/build_amoled-1-8/sketch/companion.ino.cpp before rebuilding.

### Retained status fields

| Field | Meaning |
|-------|---------|
| stack_mode, stack_bytes | Configured variant and stack size |
| placement_valid | Writer reached entry and captured placement |
| stack_start | Historical stack-start address, never dereferenced by status |
| stack_external, stack_local_external | PSRAM checks of stack start and local variable; -1 if unavailable |
| tcb_internal, tcb_bytes | Internal byte-accessible TCB check and sizeof(StaticTask_t); flag -1 before entry |
| writer_lifecycle | off, starting, active, parked, create_failed, deleted |
| stack_used_max | Configured bytes minus lowest recorded margin; -1 before a reading |
| stack_final_margin | Watermark after terminal cleanup; -1 before final reading |

Both stack-location checks must match the requested variant. Compare used bytes,
not raw margins from different-sized stacks. A static TCB occupies BSS before
the creation snapshot, so no fixed 350-byte creation delta is expected.
Shutdown uses starting/active lifecycle, never a possibly stale task handle.
deleted means cleanup is complete and the internal task's RTOS deletion follows.
parked means cleanup is complete and only suspension follows.
A formatter failure can leave heap measurements unavailable (measured=0) while
a valid final stack measurement still exists.

### Later tests, after reviewing the paired memory result

After the B Live memory failure is resolved and reviewed, pair full Live and
TLS-connect checks with hooks off in the same sitting.
Retain the roughly 5% Live-fps gate and the remaining Stage 1 requirements.

Then use a separate PSRAM build with DIAG_TEST_HOOKS=1 on a backed-up card:

- From the normal screen with real Wi-Fi/MQTT connected and normal file limits,
  send log test nvs. The main task commits a changing dummy key in namespace
  diagStress, key pulse, at most every 100 ms for 30 seconds, up to 300 writes.
  It does not burst to catch up after a main-loop stall.
- The writer produces and flushes one test record per normal iteration. It never
  calls Preferences. Stress work stops at the deadline; main-task key cleanup
  may occur later if the main loop was blocked.
- After the completion message, send log status. Require nvs_active=0,
  summary_pending=0, key_removed=1, nvs_errors=0, advancing NVS/SD counts with
  overlapping first/last intervals, and no logger errors or drops. This shows
  sustained interleaved activity, not unsafe execution while caches are off.
- Inspect TEST_NVS_START, TEST_NVS_SD and TEST_NVS_END via a card reader.
  The final record contains both counts and timestamps. If logging fails, serial
  counters remain available but the final SD record may be impossible.
- log test nvs stop ends early and removes the key. Other hooks are refused
  until the summary flushes. Power loss can leave the key; the next explicit
  stress command removes it before starting.
- Separately test log test small and repeated log test rotate for rotation and
  pruning. Do not combine these with NVS stress. Inspect logs and unrelated files.
- Test no-card cleanup, disabled/parked status, both shutdown/deep-sleep close
  paths, and final stack margins. Stop stress before timing shutdown: aborting
  an active hooks-only run also requires an NVS cleanup commit.
- Restore normal limits, DIAG_TEST_HOOKS=0 and the accepted stack setting.

Remaining storage, clock and breadcrumb gates still apply. No hardware result
is inferred from source checks.

## Implementation

- `sd_diagnostics.cpp` owns the queue, writer, file operations, health and serial snapshots.
- `diagnostics_clock.cpp` owns the checked NVS boot counter, validated RTC metadata,
  Montreal formatting and asynchronous SNTP. No writer-task NVS calls.
- `diagnostics_config.h` defaults to `DIAG_ENABLED=1` and `DIAG_TEST_HOOKS=0`.
  A logger-disabled build is possible, but the committed Stage 0 build remains
  the reference for the required paired comparison.
- The existing `companion.ino` setup starts the queue early and the writer after
  hardware and UI initialization, immediately before Wi-Fi initialization.
  This allocation-order experiment follows the failed HTTPS memory gate.
  Main-task cached snapshots run once per second; the
  writer records health once per minute. No writer calls LVGL, MQTT or the PMIC.
- `status` keeps the existing MQTT reply and adds logger snapshots.
  `log status` prints only the logger snapshots. No list, get or tail commands yet.

### Retained startup memory (retrieval already verified)

Startup text does not need to be captured. Temporary measurement-only snapshots
stay in RAM until reboot and print after the usual two lines of log status
(and status). They are not written to the SD card. Reading them does not clear them.

Ten phases cover asynchronous clock setup, writer creation, formatter allocation,
SD mount, the first writable current.log open, and completion of storage setup.
before_writer is just before task creation; writer_entry is the first task action,
after its internal stack and control block exist. The file-open pair covers either
append or exclusive creation. Later rotations do not replace that pair.
storage_done also appears after a failed storage attempt; check [LOG] state
and error. Phases never reached print captured=0.

Example format (values below are placeholders, not measurements):

    [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
    [LOG MEM] phase=before_mount up_us=<uptime> free=<bytes> largest=<bytes> heap_min_boot=<bytes>

free and largest are point readings; heap_min_boot is the allocator's
since-boot low-water mark. The three queries are sequential, not an atomic heap
snapshot. Other tasks, including Wi-Fi initialization, can allocate concurrently.
Use phase timestamps and the existing probes together; differences alone do not
prove an allocation belongs to SD. No startup barriers or new waits were added.
The fixed snapshot array adds at most 240 internal bytes and small transient
measurement stack use. Keep that overhead in mind when comparing builds.
No per-sample printing or additional SD writes are introduced.

1. JP compiles and flashes with the card installed, then lets setup finish.
2. Connect the web console with explicit DTR=true and RTS=false.
3. Send log status; save both [LOG] lines and all [LOG MEM] lines.
4. Press Latest once, wait for the image, then send log status again.
5. Send the full output, including image_https and Latest total time.

The retained lines should repeat unchanged during the same boot. [LOG] counters
and the existing [PROBE] measurements can change. No Live or fault-hook test yet.
This is allocation diagnosis for the failed Stage 1 memory gate, not a memory fix.

### Storage and task choices

| Setting | Stage 1 value |
|---------|---------------|
| SD wiring | 1 bit; CLK 2, CMD 1, D0 3 |
| SD frequency | 20 MHz (SDMMC_FREQ_DEFAULT) |
| Filesystem | User-prepared FAT32 card; never format on mount failure |
| Mount and directory | /sdcard VFS mount; /logs on the card |
| Active file | /logs/current.log, append across boots |
| Normal rotation | Before exceeding 2097152 bytes (2 MiB) |
| Archives | At most 30; exact archive-NNNNNNNN.log names, eight decimal digits |
| Content budget | 31 × 2 MiB = 62 MiB; filesystem overhead additional |
| Free-space reserve | 16 MiB |
| Directory scan bound | 256 entries in /logs, including unrelated entries |
| Record line bound | 1024-byte formatting buffer; complete newline-terminated records |
| Event queue | At most 8192 bytes explicitly in PSRAM; 4 slots reserved for important records |
| Formatter | 1024 bytes explicitly in PSRAM; not a claimed DMA-capable buffer |
| Writer stack | Default 6144 internal bytes; A/B option 8192 PSRAM bytes and static internal TCB; core 0, priority 1 |
| Writer work | At most 4 queued records per batch; 20 ms blocking wait between normal iterations |
| Directory yielding | Block for one tick every 8 entries and after each prune |
| Flush | Every 2 seconds when dirty; also on rotation and close |
| Slow-operation threshold | 100000 microseconds for write or flush; aggregate counts |
| Shutdown wait | At most 500 ms, within the existing 1-second display interval |
| Clock discontinuity threshold | More than 2000 ms relative to 64-bit uptime; lower confidence and record correction |

Only the writer accesses the filesystem. SD_MMC mounts it; POSIX descriptors
provide checked write, fsync, close and exclusive-create results.
The free-space query is `esp_vfs_fat_info("/sdcard", ...)`: it reports errors
and resolves the right mounted volume. The pinned SD_MMC usedBytes helper
returns zero on failure and assumes FatFs volume 0.

Archive generation is resolved before pruning. Files are never opened with
truncation. A corrupt nonempty header is preserved by rename before creating
a fresh current file. An empty current gets its first header. A valid header
with a partial tail gets a newline and a recovery marker, or rotates if space
is insufficient. Extra-large current files stop logging for inspection;
oversized managed archives are reported and counted against normal retention.
Unexpected directories and other filenames are never deleted.

A terminal allocation, mount or storage error disables logging for that boot.
There is no mount retry loop. The companion continues. Resource cleanup stays
in the writer; the queue remains allocated to avoid late callback use-after-free.
Insert and remove the card only with board power off.

### Records and clock

Every record begins with local, time, seq, boot, up_ms, level and event.
The writer assigns sequence numbers, including FILE_OPEN and BOOT headers.
Queued events retain their capture time; sequence is serialization order, so a
BOOT captured before mounting can follow a later-timestamped FILE_OPEN.

BOOT repeats at rotation with session, build timestamp and available operation
context. Stage 1 records logger start, setup completion, previous breadcrumbs,
clock sync, offset transitions, clock corrections, minute health, loss and slow
operation counts, tail recovery and session end. Health identifies stale or
unavailable application snapshots. Do not treat absent later-stage fields as zero.

The Montreal rule is `EST5EDT,M3.2.0,M11.1.0`. Time remains unknown without validated
provenance. A retained synchronized clock starts approximate; SNTP in this boot
makes it synced. SNTP is asynchronous, using pool.ntp.org and time.nist.gov.
Sync age is -1 before a current-boot sync. Test clocks are labeled and never
establish retained real-clock provenance.

The boot counter uses NVS namespace diagnostics and key boot, with one checked
64-bit write per enabled boot. Failure produces a volatile session tag.
RTC main and writer slots include checksums, version, sequence, boot, phase,
operation and uptime. Power-on remnants and invalid slots are rejected.
Panic and watchdog handlers never write to the card.

## Reading status

`log status` prints two short groups of measurements:

- State, boot and session, clock quality, current size and generation, newest
  archive, archive count, card and free bytes, queue use, drops and last error.
- Writer stack low-water margin in bytes; internal and DMA heap minima and
  sampled largest blocks; write and flush maxima; rotation and pruning counts.

`measured=0` means the writer has not collected memory readings. Its zero fields
then mean unavailable. `oversized=1` means an oversized managed archive was seen.
`dma_min` comes from INTERNAL | DMA, not PSRAM.
Writer memory queries bracket SD operations. The unchanged periodic probes
cover their overlapping normal, media and connect windows. A short temporary
DMA allocation can still escape largest-block sampling; mount has boundary
readings but no added Stage 0 probe window. Source confirms sector-wrapper DMA
bounce buffering; its contribution to a measured dip remains unverified.

## Historical first bench check

Use a prepared FAT32 card with a backup of anything important.
Keep fault hooks at 0 and normal limits. JP compiles and flashes.
Delete `build/build_amoled-1-8/sketch/companion.ino.cpp` before each rebuild
after sketch edits; it was removed during preparation.

Before flashing the logging build, retain a same-sitting probe-only baseline:
normal IMU summary, Latest, full Live, and MQTT off/on connect windows.
If the work moves to another sitting, repeat the baseline first; the older
numbers are not a paired performance comparison.

After flashing, connect the page with explicit DTR=true and RTS=false.
Let setup complete and send `log status`. Save startup messages, both status
lines and any probe summaries. Record board identity, card capacity and power
source. If state is ready, leave the normal screen for at least one minute,
then request status again. Confirm size and writes increase without drops.
Send these results before proceeding to the paired Latest and Live checks.
The readiness and first Latest results are recorded in the bench results. Latest
failed the memory floor; no-card HTTPS recovered above it. The card-installed
writer-start-order retest also failed at 14836 bytes. Retained snapshots and review
led to the accepted paired stack experiment at the top of this handoff.
Live, MQTT, writer peak paths and the remaining Stage 1 gate tests remain pending.

The gate order remains memory and performance, storage and rotation, then clocks
and breadcrumbs. Confirm the proposed 20480-byte TLS block floor, roughly 5%
Live fps band, no new allocation/TLS failures and zero normal-use drops with JP.
Check writer stack margin as measured; do not infer a pass from a successful mount.

## Fault hooks — later gate tests only

Compile `DIAG_TEST_HOOKS=1` only for deliberate tests on a backed-up card.
Commands fit the existing 23-character serial limit and require Enter.
They reset to normal behavior on reboot; none are persisted or auto-armed.

| Command | Action | Gate |
|---------|--------|------|
| log test spring | Set March 8, 2026 01:59:50 EST, pause SNTP; offset changes after 10 seconds | Spring DST and clock quality |
| log test autumn | Set November 1, 2026 01:59:50 EDT, pause SNTP | Repeated autumn hour and numeric offset |
| log test sync | End injected clock mode and restart real asynchronous SNTP | Correction and return to synced |
| log test panic | Set known main breadcrumb and call abort | Panic reset reason and retained phase |
| log test watchdog | Register a separate task that sleeps without feeding the existing watchdog | Actual task-watchdog reset and breadcrumb |
| log test small | 8 KiB file limit, 3 archives, 16 KiB reserve; rotate if needed | Rapid size rotation and pruning |
| log test rotate | Force one managed rotation | Repeated rotation, including without clock |
| log test rename | Rotate and stop after rename, before create | Reset at interrupted rotation |
| log test header | Rotate and stop after create, before any header bytes | Empty-current recovery |
| log test partial | Rotate, write and flush a real incomplete header, then stop | Corrupt-header salvage |
| log test space | Report insufficient free space; prune only managed archives until exhausted | Reserve exhaustion and disabled state |
| log test full | Simulate a failed write with ENOSPC | Full-card failure without filling unrelated data |
| log test normal | Restore normal limits before any terminal error | End small-limit tests |
| log test nvs | Up to 30 seconds of main-task dummy-key commits plus writer-owned SD records | PSRAM stack with NVS activity |
| log test nvs stop | Stop stress, remove key and request final SD summary | End stress early |

The three boundary-stop hooks sleep rather than spinning. Reset the board to
continue; a normal power transition may also end the test.
After a terminal storage error, reboot is required; log test normal cannot
re-enable that boot's writes. The insufficient-space test can prune managed
archives on the test card. It never deletes unrelated names or /images.
Return to DIAG_TEST_HOOKS=0 before ordinary measurements.

For SD log inspection in Stage 1, power down and use a card reader. USB file
retrieval is Stage 1B, after JP accepts this gate.

## Source evidence checked

The maker's [SDMMC demo](https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-1.8/blob/main/examples/esp-idf/09_sdmmc/main/sd_card_example_main.c)
delegates mounting to the BSP. Its current manifest requires IDF >=5.5, so
it is a hardware reference, not code to compile unchanged against core 3.1.3.
The [BSP at 9f4030c](https://github.com/waveshareteam/Waveshare-ESP32-components/blob/9f4030c6e5cb888ad4cc268bfa7584c93ad53e30/bsp/esp32_s3_touch_amoled_1_8/esp32_s3_touch_amoled_1_8.c)
uses 1-bit SDMMC and no separate SD power-control operation in its mount helper.

Checked installed core 3.1.3 SD_MMC and esp32-hal-time.c, plus SDK 5.3
esp_vfs_fat.h, esp_sntp.h and esp_task_wdt.h. The configured core 0 idle watchdog
is 5 seconds with panic. Writer loops always block or yield.

Preparation checks: source review, unchanged probe diff, serial-command length
bounds, DST epochs against the installed JavaScript timezone database, and
git diff --check. These are not a substitute for JP's build and board tests.
