# SD card diagnostic logging plan

Status: finalized proposal after [review and counter-review](sd_diagnostics_plan_review.md).
Firmware is not implemented. The owner reviews and commits this reference before firmware work.

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
- **Serial:** preserve `off`, `on`, `status`; add logger snapshots and optional bounded `log tail [n]` through the
  existing parser.
- **Scope:** optional logging must leave the companion usable without a card. No generated SquareLine edits,
  network-policy changes, image and video storage, uploads, remote controls, on-screen browser or panic core dumps.

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

Use the existing bounded **24-character** parser in `netBenchLoop`, not another serial reader. `status` and
`log status` read a thread-safe writer snapshot (health, current size, newest archive, drops). Optional `log tail [n]`:
default **20**, maximum **100** records, **16 KiB** byte budget; request backward chunked current-file reads from the
writer, with logging and close priority. Return fewer records with a limit notice; no full scan, archive traversal or unbounded allocation. Snapshot file identity, include format context, report busy or unavailable, and transmit through
a bounded output queue in small nonblocking chunks, without bulk printing or flushing. Commands still wait behind a blocked
main-loop call.

## Limits and unverified points

- **DMA buffer:** a temporary internal buffer for PSRAM SD writes is likely but unverified in this SDK; frequency, size and direct-DMA behavior are driver-specific. Check the matching implementation or allocation traces: before and
  after heap snapshots can miss transients.
- **Brownout breadcrumbs:** restart and deep-sleep retention is supported; brownout survival depends on voltage and
  reset domain. Discard invalid data and power-on remnants and interpret phases alongside reset reasons.
- **Durability:** power loss can lose queued and unflushed records or corrupt FAT and rename state. Two-second flushes
  are targets. Close deadlines cannot cancel an in-flight write; deferral and full queues increase unsaved history. A missing END or clean-close marker alone does not diagnose a crash.
- **TLS errors:** hostname DNS failure in core 3.1.3 can return before updating `lastError()`, leaving stale or zero data; do not infer a fresh TLS cause from it.
- **Retention:** daily growth, record sizes and powered hours need measurement; appendix figures are conditional.
  Profile and recovery evidence may help explain car faults but does not establish their cause in advance.

## Staged rollout and bench gates

### Before firmware work

The owner reviews this finalized proposal and commits **this plan, the review and `implementation_plan.md`** as the
fixed reference for code review. This editing pass makes no commits.

**Every stage:** the owner compiles and flashes from VS Code. Stage 1 touches both `companion.ino` and `src/`; before
each rebuild delete **`build/build_amoled-1-8/sketch/companion.ino.cpp`** per the stale-build rule in
[CLAUDE.md](../CLAUDE.md). The assistant does not compile or flash.

### Stage 0 — measurement probes

Make a minimal probe-only change to the current firmware. Record minimum free
internal heap and the lowest observed largest free internal block, including
inside blocking MQTT, HTTPS and Live TLS connects and throughout Live. Use the
same internal-memory capability filters in every measurement.

Boundary readings alone miss temporary allocation lows. Use lightweight sampling
that runs during blocking calls or suitable allocation instrumentation. Report
sampling coverage and interval; incomplete coverage is not a passing memory test.
Keep serial output to summaries so the probes do not create new frame delays.
No SD logger or operational event hooks are added in Stage 0.

The owner flashes this probe-only build in VS Code and measures the baseline:
Live fps and frame gaps, Latest total time, internal heap minimum and lowest
largest internal block. Stage 1 retains these same probes unchanged. Compare
baseline and logging enabled back-to-back in the same sitting, as required by
CLAUDE.md; figures from another day are not comparable.

### Stage 1 — basics only; stop for bench acceptance

Implement these basics:

- Mount with formatting disabled, PSRAM event queue and writer task.
- Append and common headers, size rotation, pruning and recovery.
- Boot counter, clock quality states, asynchronous sync and Montreal rule.
- BOOT and reset-reason records, RTC breadcrumbs and both shutdown close hooks.
- Minute health record and logger state in serial status.

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
   no-card, bad-card, unsupported-card and simulated full-card behavior. Preserve
   dummy `/images/` data and unrelated files. Restore normal limits afterward.
3. **Clock and breadcrumbs:** test cold boot, deep-sleep wake, approximate time,
   real sync and corrections, both injected DST transitions, and controlled panic
   and watchdog resets. Check breadcrumb validation and distinct reset classes;
   test brownout retention separately where practical.

Run one test at a time and back up the card before deliberate power interruption.
Record pass and fail results and measurement coverage. **Stop: Stage 2 starts only
after the owner accepts Stage 1 results.**

### Stage 2 — network evidence

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
After an incident, power down, copy `/logs/`, and inspect current and newest archives with Montreal time and boot
markers. Weekly rotation remains deferred pending demonstrated need.

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
