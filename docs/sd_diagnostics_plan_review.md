# Review: SD card diagnostic logging plan

Reviewed document: [sd_diagnostics_plan.md](sd_diagnostics_plan.md)
Reviewer: Claude Code, 2026-09-14. The project owner has accepted these proposals;
they are pending a counter-review and a plan update.

## Summary

The plan is sound. It records the right communication events, keeps secrets out of
the log, uses a queue plus a single SD writer, keeps logging optional, and validates
the logger by comparing runs with logging on and off. Its statements about the
current firmware check out:
- The SD pins in `pin_config.h` are correct.
- No SD code exists yet.
- Detailed MQTT attempt timing exists only in bench mode.
- The `off`/`on`/`status` serial commands are present.

Its main weakness is complexity around file rotation. There is also one ordering
error, a clock-validity mistake, and a boot-time scan that is risky.

Each item has an ID so the counter-review can answer it point by point.

## Mistakes

### M1. Rotation is scheduled before the clock exists

Step 1 of the implementation sequence builds "rotation", which includes the 7-day
age rotation. Clock sync and Montreal timestamps are not added until step 3, and
age rotation cannot work without a valid clock.

**Proposal:** move clock sync into step 1, or ship size-only rotation first (see S1).

### M2. Clock validity is undefined after deep sleep

`goToDeepSleep()` uses ESP32 deep sleep, which preserves system time. This build
uses the internal RC oscillator for the RTC (`CONFIG_RTC_CLK_SRC_INT_RC=y`), which
drifts noticeably, so time kept through a sleep is only approximate. A PMIC
shutdown (`goToShutdown()`) loses time entirely. A check such as "epoch after
2020" would therefore report a drifted clock as valid after a deep-sleep wake.

**Proposal:**
- Define validity as "synchronized during this boot".
- Mark time carried through deep sleep as `approx` until the next sync.
- Never make rotation decisions from approximate time.

### M3. The boot-time scan of `current.log` is risky

The "Append and rotation recovery" section proposes a streaming scan of up to
8 MiB at boot to recover the file's generation and age anchor. That scan runs
during the busiest and most memory-constrained period: Wi-Fi setup, the TLS
handshake and the first MQTT attempts. On a 1-bit SDMMC bus it can take seconds,
and the queue can overflow while it runs.

**Proposal:** remove the need for the scan (S1). If weekly rotation is kept, store
the generation number and age anchor in a tiny sidecar file or in NVS, written
once per rotation, and read only that at boot.

### M4. "Automatic daylight saving" is a compiled-in rule

A POSIX TZ string such as `EST5EDT,M3.2.0,M11.1.0` is fixed in the firmware; the
ESP32 has no tzdata updates. If Quebec changes its time rules, timestamps stay
wrong until the firmware is rebuilt.

**Proposal:** state this in the plan, and name the exact TZ string. Records keep
their UTC offset, so logs remain convertible regardless.

## Simplifications

### S1. Rotate by size only, at least in the first release

Most of the recovery logic exists only to support age rotation:
- clock anchors
- suspending rotation after implausible clock jumps
- reconstructing generation numbers
- `undated` archive names

Size-only rotation still bounds storage, needs no clock and no boot scan, and
every line already carries its own timestamp.

**Proposal:**
- Use files of 1-2 MiB and keep about 30 archives. Tune the count from the
  measured daily growth.
- Keep generation-numbered archive names.
- Add weekly rotation later only if it proves useful, using the sidecar from M3.

### S2. Use a boot counter instead of a 128-bit random session ID

A 128-bit ID is unreadable when searching a log and cannot order boots.

**Proposal:**
- Keep a boot counter in NVS, incremented once per boot (for example `boot=214`).
- One write per boot is not a "frequently updated" NVS value; `screenMem` already
  writes NVS more often than that.
- If collision resistance is wanted (for example after an NVS erase), add a short
  32-bit random suffix.

### S3. Minor

- Drop `v=1` from every line; the `FILE_OPEN` header already records the format
  version.
- Correct the reasoning for the writer task. A loop-based writer would not lose
  records during blocking MQTT work, because the queue holds them. The real reason
  for a task is that slow SD writes or flushes would stall the UI and touch
  handling. The conclusion stays the same.

## Additions

### A1. Crash breadcrumb in RTC memory

Keep the current phase in an `RTC_NOINIT_ATTR` variable. Example phases are
`mqtt_connect`, `wifi_setup`, `image_fetch`, `live_tls_handshake`, `live_frame`
and `sd_write`. The variable survives panic, task-watchdog and brownout resets,
but not power loss. On the next boot, log it with `esp_reset_reason()`.

This respects the plan's rule against SD writes from panic handlers, and is much
stronger evidence than an operation that has a BEGIN record but no END. Validate
it with a magic value or checksum, so garbage after a power-on reset is ignored.

### A2. Concrete memory placement

Relevant settings, from the core 3.1.3 build
(`esp32-arduino-libs/idf-release_v5.3-489d7a2b-v1/esp32s3/sdkconfig`):

| Setting | Consequence |
|---------|-------------|
| `CONFIG_FATFS_ALLOC_PREFER_EXTRAM=y` | FATFS and per-file buffers already prefer PSRAM |
| `CONFIG_FATFS_LFN_STACK=y`, `CONFIG_FATFS_MAX_LFN=255` | Long-filename work buffers use the calling task's stack; size the writer stack for it |
| `CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y` | The writer's stack may be placed in PSRAM (`xTaskCreateStaticPinnedToCore`), sparing internal heap. The writer must never write SPI flash or NVS. Validate on the board. |
| `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096` | `malloc` of 4096 bytes or less goes to internal RAM; allocate the queue explicitly with `MALLOC_CAP_SPIRAM` |

SDMMC transfers from non-DMA (PSRAM) buffers probably need a temporary DMA-capable
internal buffer for each write. This is likely, not yet verified.

**Proposal:**
- Measure minimum free internal heap and the largest free block during Live and
  during TLS handshakes, with the logger on and off.
- Consider holding routine writes while a TLS handshake is in progress, because
  that is when mbedTLS needs its contiguous ~16 KB buffers.

### A3. Transport error details

- After a failed MQTT or HTTPS attempt, log `WiFiClientSecure::lastError()`
  (code and short text) and `mqttClient.state()`.
- When MQTT drops from a connected session, log `state()` (-3 connection lost,
  -4 timeout), Wi-Fi state, RSSI and the time since the last inbound message.
- Do **not** separate DNS from TCP failures by connecting to a resolved IP address:
  that loses SNI and certificate hostname checking.

### A4. Name every shutdown path

The logger's drain, flush and close must be reached from all of these paths:
1. The battery-only shutdown when Wi-Fi fails during setup
   (`attemptWiFiConnection()`).
2. The grace-period shutdown after USB power is lost (`loop()`, task 10). In the
   car, this is the normal logged end of a trip.
3. Inactivity-triggered shutdown or deep sleep.

`goToDeepSleep()` and `goToShutdown()` already delay about 1 s before powering
down. The proposed 500 ms flush wait fits inside that delay, with no user-visible
change.

### A5. Wi-Fi recovery after a mid-session drop

If Wi-Fi was up at boot, the loop's periodic `WiFi.begin()` retry is disabled
(`!g_wifiUpAtBoot`), so recovery relies on the driver's auto-reconnect. MQTT is
reconfigured only on the late-connect path (`g_mqttConfiguredLate`).

**Proposal:** log reconnection explicitly, including which network profile it
rejoined and whether MQTT resumed afterwards. This is a likely car failure pattern
and currently leaves no trace.

### A6. Serial log access

Add optional `log status` and `log tail [n]` serial commands. They let recent
records be read without pulling the card; the plan currently defers every on-device
viewer.

`log tail` must ask the writer task to read the file and must not touch SD from
`loop()`. `log status` must come from a snapshot of statistics the writer
publishes.

### A7. Loop-gap noise

Each failed MQTT connection attempt blocks for about 5 s by design, so it would
also trigger the 1-second loop-gap warning. Tag each gap with its known blocking
cause (for example `cause=mqtt_connect attempt=N`), or leave out gaps fully
explained by an already-logged blocking operation.

## Verified, no change needed

- **Clock sync does not change TLS behavior.** `CONFIG_MBEDTLS_HAVE_TIME_DATE` is
  not set, so setting system time will not start certificate validity-date
  rejections.
- **Long archive filenames are supported** (`CONFIG_FATFS_MAX_LFN=255`).
- **Never auto-formatting is correct.** Pass `format_if_mount_failed=false` to
  `SD_MMC.begin()`.
- **The flush and health-record rates are reasonable.** A 2-second flush with
  immediate flush on failures is fine. A 60-second health record costs about
  360 KB a day at roughly 250 bytes per record.
- **Wi-Fi events are safe to enqueue from `WiFi.onEvent` callbacks.** They run in
  the Arduino events task (`EventsCore=1`), not in an ISR.
