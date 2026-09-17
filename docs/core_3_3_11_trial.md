# Core 3.3.11 trial

Prepared for JP on 2026-09-17. JP requested committing and pushing this pre-flash checkpoint. JP confirmed successful compilation with core 3.3.11 and the separate Waveshare graphics 1.6.4 copy after the compatibility corrections. JP has flashed the trial and reports normal Latest, Back, Live, inclinometer, G-meter, motion detection and IMU operation. Boot-47 startup/status checks are healthy and the first 8059-byte archive download passed browser CRC validation. Independent saved-file comparison and the remaining quantitative/runtime gates are pending.

## Changes

- `sketch.yaml`: only `amoled-1-8-core-3-3-11` now pins core 3.3.11. It selects the installed Adafruit_XCA9554 1.0.0 and Adafruit_BusIO 1.17.4 folders instead of ESP32_IO_Expander.
- The two 3.1.3 profiles are unchanged. Arduino Maker had already selected the new profile in `.vscode/arduino.json` and as `default_profile`; those selections remain.
- `companion.ino` selects the old expander for cores below 3.2 and Adafruit for newer cores. This preserves rollback with the old profiles and dependencies.
- Adafruit uses the existing Wire bus at 0x20. Pins 0, 1 and 2 still receive the same low, 20 ms delay, high reset sequence. Initialization failure prints an error and waits instead of continuing to the display.
- A startup marker reports `[BUILD] core=3.3.11 expander=Adafruit_XCA9554`. The old profiles report their own core and old driver.
- The revised trial selects Waveshare graphics 1.6.4 from `C:/Users/photo/Documents/Arduino/libraries/GFX_Library_for_Arduino_Waveshare_1_6_4`. The original graphics 1.4.9 folder and rollback profiles remain untouched.
- The 3.3.11 code path drops the SH8601 constructor's old IPS argument and calls `setBrightness(150)`. The 3.1.3 path retains the old constructor and `Display_Brightness(150)`. These branches assume the documented core/library pairings.
- The explicit 20 MHz QSPI setting, all other libraries, Stage 0 probes, USB protocol, writer placement and timeouts are unchanged. Fault hooks remain off. The 20480-byte memory gate remains.

`sketch.yaml` is gitignored. A [tracked trial-profile snapshot](build_profiles/amoled-1-8-core-3-3-11.yaml) records the compiled configuration. Arduino Maker still reads workspace `sketch.yaml`; the snapshot does not change active settings. External libraries are not included in Git.
The pre-edit profile and sketch were copied to `%TEMP%/companion-core3311-before/` for this preparation.

## Installed SDK checks

Read the actual installed core at:
`C:/Users/photo/AppData/Local/Arduino15/packages/esp32/hardware/esp32/3.3.11/`.
The S3 SDK is at:
`C:/Users/photo/AppData/Local/Arduino15/packages/esp32/tools/esp32s3-libs/3.3.11/`.
Its `esp_idf_version.h` reports 5.5.5.

- The Waveshare board still supports the selected hardware CDC, CDC-on-boot, 240 MHz, core 1 loop/events, 16 MB flash and application/FAT partition options. PSRAM enabled selects OPI, so the applicable configuration is `qio_opi/include/sdkconfig.h`.
- `CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM=1` and static allocation are enabled. `StackType_t` remains uint8_t and the static pinned-task stack length is in bytes. The existing 8192-byte allocation remains appropriate as an experiment.
- The writer's file-scope internal `StaticTask_t` and runtime placement checks remain. Its size is obtained from the new headers and reported at runtime, not hard-coded to the previous SDK's 352 bytes.
- Static-task cleanup is not enabled. The existing static writer still suspends after cleanup, retaining its stack and task state, instead of deleting itself.
- The task watchdog still checks core 0 idle with a five-second timeout and panic enabled. Existing writer yields and core 1 placement remain.
- mbedTLS uses internal allocation. Recheck measured TLS memory after the core change; old measurements are not acceptance of the new SDK.
- The installed HWCDC.cpp contains the newer partial-FIFO stash and critical-section handling motivating this trial.

## Tagged source checks

Read ESP-IDF v5.5.5 sources in addition to installed headers/configuration:

- [FreeRTOS heap checks](https://github.com/espressif/esp-idf/blob/v5.5.5/components/freertos/heap_idf.c): the TCB must be internal and byte-accessible; external stacks are allowed when the configured flag is enabled.
- [Flash cache coordination](https://github.com/espressif/esp-idf/blob/v5.5.5/components/spi_flash/cache_utils.c): scheduler and other-core coordination remain. The PSRAM writer must still never initiate flash, NVS or partition writes. Boot-counter NVS stays on the main task.
- [FatFS SDMMC glue](https://github.com/espressif/esp-idf/blob/v5.5.5/components/fatfs/diskio/diskio_sdmmc.c) calls the sector wrappers for reads and writes.
- [SDMMC sector wrappers](https://github.com/espressif/esp-idf/blob/v5.5.5/components/sdmmc/sdmmc_cmd.c) exclude external buffers from direct transfer when SOC_SDMMC_PSRAM_DMA_CAPABLE is false; that capability is not defined by the installed S3 soc_caps.h. They use a DMA buffer, copy data, and propagate allocation/transfer errors. The newer wrappers support configurable chunks; the zero/default chunk is one block. Do not assume all allocations match 5.3.

These checks found no blocker for attempting the build. They are not a full SDK audit or hardware validation. No raw SDMMC data-buffer calls were added to the writer.

## First JP checkpoint

1. Confirm Arduino Maker has `amoled-1-8-core-3-3-11` selected.
2. Compile only in VS Code. Check the build output selects core 3.3.11, graphics 1.6.4 from the separate Waveshare folder and both Adafruit libraries. ESP32_IO_Expander should not be selected for this profile.
3. Send the success summary, or the first compiler error with surrounding lines. Resolve this checkpoint before flashing.

The stale sketch intermediate was removed from the existing baseline build directory and generic build/sketch directory during preparation. The new profile's intermediate did not yet exist.
Before later rebuilds after sketch edits, remove `build/build_amoled-1-8-core-3-3-11/sketch/companion.ino.cpp` if present. Use the corresponding profile path when reverting.

After compilation is accepted, check startup, display and touch, logger status, then archive 14 download and CRC verification. If USB retrieval succeeds, continue memory, Live, IMU and shutdown checks with same-session comparisons. Stage 1B remains pending.


## First compile result: graphics compatibility blocker

JP's first 3.3.11 build failed. The dependency summary confirms core 3.3.11,
Adafruit XCA9554 1.0.0, Adafruit BusIO 1.17.4, and graphics 1.4.9.
ESP32_IO_Expander is absent from that summary, as intended.

The reported errors are three calls to spiFrequencyToClockDiv in
Arduino_ESP32SPI.cpp (lines 116 and 163) and Arduino_ESP32SPIDMA.cpp (line 63).
The old calls supply only frequency; the installed 3.3.11 declaration requires
spi_t* plus frequency. Arduino compiles these backends even though companion
instantiates Arduino_ESP32QSPI for the display.
This was missed by the earlier check limited to the active QSPI path.

The original-board Waveshare 1.6.4 copy in JP's Downloads directory has a
version-aware gfxSpiFrequencyToClockDiv helper in both files, selecting the
new signature for core >= 3.3.10. This directly addresses the reported error.
No successful full build is claimed; later errors may remain.

Recommended next choice: use that maker-bundled graphics 1.6.4 in a separate
folder for the trial, adapt the SH8601 constructor and brightness call, and
preserve 20 MHz and the old profile/library. Alternative: backport the small
compatibility helper to a separate 1.4.9 copy to minimize rendering changes.
JP's choice is pending; no graphics/profile/firmware change was made in response
to this failed-build report. No compile, flash, commit or push by Codex.


## Revised trial after JP's choice

JP accepted the separate Waveshare 1.6.4 library rather than maintaining a 1.4.9 patch.
Copied all 321 files from the original-board `examples/arduino/libraries/GFX_Library_for_Arduino`
in JP's downloaded repository to the separate sketchbook folder above.
SHA-256 comparisons verified every copied file against the source.
Only the new profile selects it; original profiles still select the old library.
The two source API adaptations are version-gated for rollback.
The brightness call casts the existing Arduino_GFX pointer to its actual Arduino_SH8601 type; the new method is not declared on Arduino_GFX. Shared base-pointer declarations elsewhere remain unchanged.

The library itself changes QSPI DMA buffer sizes, rotation implementation and other rendering internals.
The explicit 20 MHz clock, dimensions, zero initial rotation and requested brightness 150 are retained.
Successful compilation will not establish equivalent rendering or memory/performance behavior.
The new profile's stale `companion.ino.cpp` was removed again after these edits.
JP should compile only and return the build summary or first error before flashing.


## Second compile result: legacy BLACK alias

JP's next output confirms core 3.3.11 and graphics 1.6.4 from the intended
separate folder, with both Adafruit dependencies. The reported error is
`BLACK` not declared in `initDisplay()`.
Changed the single use to `RGB565_BLACK`, which both installed graphics
versions define as RGB565(0, 0, 0), preserving the same black fill.
No other legacy color uses were found in companion.ino or src/.
The old brightness method remains only in the guarded rollback branch.
The trial sketch intermediate was removed. JP recompiles; no compile,
flash or commit by Codex, and no hardware result yet.


## Compilation checkpoint passed

JP reports `Command executed successfully`. The supplied dependency summary
confirms ESP32 core 3.3.11, Waveshare graphics 1.6.4 from the separate folder,
Adafruit XCA9554 1.0.0 and BusIO 1.17.4. SensorLib 0.3.1, LVGL 8.4.0,
DriveBus 1.0.1, XPowersLib 0.2.6, PubSubClient 2.8, TJpg_Decoder 1.1.0
and ESP32_JPEG 0.0.1 remain selected. ESP32_IO_Expander is absent.

Next: JP flashes the bench board using the same profile, with the hotspot on.
First check startup without repeated resets, display orientation/colors,
animation and touch, then connect the web console with DTR=true/RTS=false
and send `status`. Review that output before the archive-download test.
Compilation alone does not pass the Stage 1B hardware gate.
No source change, rebuild, flash, commit or push by Codex at this checkpoint.


## First hardware checkpoint: boot 47, 2026-09-17 14:21

JP reports normal operation of Latest, Back and Live; inclinometer, G-meter,
motion detection and IMU also look normal. This is a visual check, not a
measured Live/performance acceptance.

The web console connected with DTR=true, RTS=false and listed four files.
At uptime 35724 ms the logger was ready, clock synced, hooks off, generation 17,
three archives (newest 16), current.log 123750 bytes, error=none, queue=0/16,
high=1 and drops/suppressed/truncated all zero.
The final @@USB reply was active=0, paused=0, bytes=0, result=none.
The earlier active=1 reply coincided with file listing; it was not a stuck download.

Memory/status: internal_min=92760 bytes, internal_largest=51188 bytes,
dma_min=85096 bytes, dma_largest=51188 bytes. Writer stack is 8192-byte PSRAM,
placement_valid=1, stack and local external, TCB internal (352 bytes), core 1,
active, used_max=4424 bytes and minimum remaining margin=3768 bytes.
Seven writes, no slow writes, write_max=1807 us, flush_max=5183 us,
sd_max=68191 us. These early-boot observations do not replace TLS or full Live gates.

The 18267 ms normal probe reports 1827 samples at a nominal 10 ms interval,
gap_max=10866 us and scan_max=404 us. IMU n=765, minimum=26.31 Hz,
average=42.31 Hz. Record the lower average as an observation for a later full
normal-operation window and same-session comparison; this short initial
window alone does not establish a regression or cause.

Next single test: download archive-00000014.log with every page test switch off,
then send status. Expected reference: 8059 bytes, CRC 99C24CB1. Ask JP for the
console, saved file path and responsiveness observation. No reflash or card
removal needed. No source change or commit at this checkpoint.


## First successful USB archive retrieval: boot 47, 2026-09-17 14:22

JP reports that the browser saved the file. The console records archive-00000014.log,
8059 bytes, 0.12 seconds, decoded 64992 B/s, protocol wire 91661 B/s, CRC OK.
Post-transfer @@USB: active=0, paused=0, bytes=8059, result=ok.
The logger remains ready and synced, hooks=0, queue=0/16, high=1,
drops/suppressed/truncated=0, error=none. current.log grew to 125207 bytes.
Writer stack margin is 3320 bytes (used_max=4872), placement valid, core 1.
Internal_min=92760 and internal_largest=51188 bytes; no slow writes.

This passes the browser/device small-archive transfer check. Independent byte
comparison to the saved reference is still pending the downloaded file path.
CRC OK confirms the browser matched the device's transmitted checksum;
the console does not print its numeric CRC. Do not claim a local-file hash match yet.

Most likely explanation: the 3.3.11 HWCDC implementation fixes partial-FIFO
handling and task/interrupt coordination in the failing sustained-transmit path.
The USB protocol and writer core-1 placement stayed the same across the upgrade;
core 3.1.3 had still stalled after about 1 KiB with that placement.
This is strong supporting evidence, not isolation of a particular upstream fix:
core/SDK, expander and graphics libraries changed together, including memory layout.
The graphics update also reduced its DMA buffer allocation, but that alone does
not establish the previous failure's cause. Larger downloads and the remaining
Stage 1B gate tests remain pending; 0.12 seconds on 8059 bytes does not validate
the 2 MiB throughput/current-file deadline gate.

IMU normal run 2 now supplies a full 60003 ms window: n=2521,
minimum=29.43 Hz, average=42.45 Hz. Run 3 is 52113 ms, n=2188,
minimum=27.76 Hz, average=42.43 Hz. The roughly 42 Hz observation persists
beyond the short initial window and needs a same-session performance comparison.
Do not dismiss it as startup-only; cause and acceptability remain unmeasured.
No firmware changes, compilation, flash, commit or push by Codex.


### Archive-14 independent file comparison passed

Located `C:/Users/photo/Downloads/47-archive-00000014.log` and compared its raw
bytes with `docs/bench_data/sd_logs_2026-09-17_offline_rotation/logs/archive-00000014.log.txt`.
They are identical: 8059 bytes, CRC32 99C24CB1,
SHA-256 5ec652cb9e9268d92c44511b452746bbc43f82d3dc246eb9eea69a7e23c12eb0.
This completes the small-archive independent integrity check, not the entire USB gate.
Next: one normal current.log download, immediate status, then another status
70 seconds later to check append growth. All page fault switches stay off.
Current snapshot prefix comparison and later gate cases remain pending.


## Current-log retrieval and resumed appends: boot 47, 2026-09-17 14:28

JP reports the file downloaded successfully. Browser: 128183 bytes in 1.28 s,
decoded 99909 B/s, protocol wire 140159 B/s, CRC OK.
Located `C:/Users/photo/Downloads/47-current.log`: exactly 128183 bytes,
CRC32 11E25C6B, SHA-256 fa2d3e736a9991501a7cca38b9d96eeec378c9bdbf03fa4ef0387f1a64b26dc5.
It decodes as UTF-8, contains 281 complete lines, and ends with a newline.
The final record is boot-47 seq=17 USB_GET_BEGIN for current.log, consistent
with the implementation's flush-before-snapshot ordering. Its own completion
record is expected in later appends, not inside this snapshot.

Immediate status: current size=128345, USB active=0/paused=0, bytes=128183,
result=ok. At the next status 63.298 seconds later, current size=128908
(+563 bytes), writes 18 -> 19. Queue=0/16, high=1, drops/suppressed/truncated=0,
error=none, logger ready, same boot 47 and generation 17 throughout.
Stack margin remains 3320 bytes, internal minimum 92760 bytes and sampled
largest minimum 51188 bytes. This passes the ordinary current-download and
resume check; independent current-prefix comparison and stress gates remain pending.
The requested wait was about 70 seconds; the observed 63 seconds already spans
a health write, so no repeat is needed merely to reach 70 seconds.

Normal IMU averages remain 42.54 and 42.66 Hz over full minute windows,
consistent with the previously recorded follow-up concern.

### Separate earlier watchdog observation

The downloaded file records boot 46 and boot 47 with compile stamp
Sep 17 2026 14:10:07. Boot 46 contains Live and screen/motion health snapshots,
then boot 47 has reset=task_watchdog, reset_code=6. Its initial local timestamp
is approximately 14:20:38 (clock quality approx). This precedes the successful
archive and current download tests, which remain in boot 47.
Retained main/writer breadcrumbs both show idle; they do not identify the fault.
No panic trace is available in this downloaded file.

JP confirms a freeze/restart happened when closing the VS Code monitor;
the board has been stable afterward. This supports treating the earlier reset
as associated with monitor closure, separately from the successful downloads.
The driver or firmware mechanism remains unproven without a panic trace.
The monitor-close problem remains open on the new-core trial.
Next ordinary-use test: download current plus newest three, with all page
fault switches off, then capture status. No rebuild or monitor handoff is needed.
The 2 MiB throughput gate remains pending; this smaller bundle cannot satisfy it.
No firmware change, compile, flash, commit or push by Codex.


## Four-file bundle passed: boot 47, 2026-09-17 14:37

JP confirms all four files downloaded. The browser reports CRC OK for each:

| File | Bytes | Browser duration | Decoded B/s | Wire B/s |
|---|---:|---:|---:|---:|
| current.log | 133577 | 1.37 s | 97701 | 137058 |
| archive-00000016.log | 7942 | 0.12 s | 68643 | 96889 |
| archive-00000015.log | 7936 | 0.13 s | 61952 | 87447 |
| archive-00000014.log | 8059 | 0.13 s | 63457 | 89496 |

Total payload is 157514 bytes. First command to final completion takes 1.742 s.
Final status stays on boot 47, logger ready and synced, hooks=0, queue=0/16,
high=1, zero drops/suppressed/truncated and error=none. USB active=0, paused=0,
bytes=8059, result=ok describes the final archive, not the whole bundle.
current.log is 134753 bytes, consistent with resumed appends after its snapshot.
Writer stack margin remains 3320 bytes, used maximum 4872, placement valid.
No slow writes; write maximum 6443 us, flush maximum 10251 us.

Internal minimum is now 44540 bytes and sampled largest minimum 31732 bytes,
above the 20480-byte gate. These retained minima do not isolate this transfer.
The normal probe emitted at the first request already reports heap_min_boot=44540,
so that decrease from the earlier 92760 cannot be attributed to this bundle.
Its normal-window IMU average is 42.68 Hz over 26.510 seconds.
This passes consecutive ordinary transfers; it does not replace the 2 MiB test.

Next: one full Live cycle without downloading, retaining its timing/probe lines
and post-cycle status. Then a second Live cycle with a bundle download in the
same sitting can test interference. Do not compare with another day's fps.
No firmware changes, build, flash, commit or push.


## Live baseline without retrieval: boot 47, 2026-09-17 14:39

JP reports normal video throughout the full cycle. Same firmware, hooks=0,
web console connected; no file downloads during Live.

- 181 frames in 60.426 seconds (reported 60.4 s, 3.0 fps; about 2.995 fps
  from frame count and probe duration). Frame average 334 ms, first frame
  1190 ms, maximum frame gap 1106 ms.
- HTTP 325 ms = TTFB 153 + transfer 172 ms; decode 61 ms, blit 80 ms.
  Average JPEG 18.3 KB, transfer rate 106 KB/s.
- Live TLS windows: 651 ms / 65 samples and 747 ms / 75 samples.
  Both sampled largest internal minima are 31732 bytes.
- Full Live: sampled largest internal minimum 26612 bytes, above the
  20480-byte gate by 6132 bytes. Exact free-internal low since boot falls
  from 44540 to 37628 bytes. Sampler interval 10 ms, 6042 samples,
  maximum sample gap 11226 us, maximum scan 1289 us.
- Post-status logger sample minimum largest block is 28660 bytes; this
  coarser logger measurement does not override the probe's lower 26612.
- Same boot 47, logger ready/synced, USB idle/unpaused, queue=0/16 high=1,
  zero drops/suppressed/truncated and error=none. Writer margin stays 3320
  bytes, used maximum 4872, valid PSRAM placement and core 1.
- current.log grows 135885 -> 136450 bytes; writes 37 -> 38.

This supplies the same-session reference for the next Live cycle with one
current-plus-newest-three download about ten seconds into the feed.
Use frame count and duration as well as rounded fps for the approximately
5 percent comparison (baseline lower bound about 2.846 fps).
The measured blit time of 80 ms and normal IMU average of 42.71 Hz warrant
tracking in the migration review; historical 61 ms and roughly 49 Hz readings
are not a controlled same-session comparison of the core/library change.
No allocation or TLS failure is reported. This baseline does not yet validate
Live/download interference or the whole Stage 1B gate. No firmware changes.


## Live with four-file retrieval passed: boot 47, 2026-09-17 14:45

The replacement capture contains the concurrent transfers. JP previously
reported smooth video with in-frame seconds advancing and all files saved;
this capture supplies the measured comparison after JP repeated the test.

| Measurement | No-download baseline, 14:39 | Live plus bundle, 14:45 |
|---|---:|---:|
| Frames / probe duration | 181 / 60.426 s | 181 / 60.256 s |
| Reported fps | 3.0 | 3.0 |
| Average frame | 334 ms | 333 ms |
| First frame | 1190 ms | 1256 ms |
| Maximum frame gap | 1106 ms | 901 ms |
| HTTP / decode / blit | 325 / 61 / 80 ms | 324 / 61 / 80 ms |
| Live sampled largest internal minimum | 26612 bytes | 31732 bytes |
| Exact internal free minimum since boot | 37628 bytes | 37552 bytes |
| Writer stack remaining margin | 3320 bytes | 3320 bytes |

Frame count divided by duration changes by about +0.28 percent, inside the
approximately 5 percent performance gate. This is a same-session comparison
on the same firmware and boot, not a comparison of the two core versions.
The 14:45 run is Live run 3 because JP repeated the intervening test.

The bundle starts 9.973 seconds after Live starts and finishes in 1.971 s.
All four files have browser CRC OK: current 141350 bytes in 1.52 s
(decoded 92902 B/s, wire 130325 B/s), archive 16 7942 bytes in 0.12 s,
archive 15 7936 bytes in 0.10 s, archive 14 8059 bytes in 0.23 s.
Total payload=165287 bytes. Both TLS windows (767 and 598 ms) have sampled
largest minima of 31732 bytes. Full Live has 6026 samples at 10 ms,
maximum sample gap 11182 us, scan maximum 1052 us.

Final logger is ready/synced, hooks=0, queue=0/16 high=1, no drops, suppression,
truncation or errors. USB is idle/unpaused with final archive bytes=8059,
result=ok. Boot remains 47. File size grows 140622 -> 143095, writes 51 -> 61.
The logger's retained largest minimum remains 26612 from prior activity;
the current Live window minimum is 31732. Both exceed the 20480-byte gate.
No allocation/TLS failure or reset appears. No accumulating stack loss is
observed here. Normal pre-Live IMU average is 42.58 Hz.

This passes the ordinary-size Live/download interference case. It does not
cover sustained 2 MiB transfer, MQTT retries, fault injection or the full gate.
Next: the page's one-line damage switch on current.log, confirm rejection
and abort completion, then a normal retry and status. No rebuild required.


## Damaged-line rejection and retry passed: boot 47, 2026-09-17 14:49

JP confirms the test succeeded. Current retrieval starts at 14:49:54.673;
the page sends log abort at 14:49:54.722. Firmware replies reason=aborted
at 14:49:54.744 and the page reports malformed data with abort confirmed.
This exercises deliberate browser-side line damage, not SD corruption.
The capture has no successful DOWNLOAD for the rejected attempt.

The normal retry at 14:50:13.667 saves 145280 bytes in 1.44 s, CRC OK,
decoded 100980 B/s and protocol wire 141663 B/s. Final status stays boot 47,
logger ready/synced, hooks=0, queue=0/16 high=1, zero drops/suppressed/truncated,
error=none. USB active=0, paused=0, bytes=145280, result=ok. Current size is
145443 after resuming appends. Writer stack margin remains 3320 bytes,
used maximum 4872. Retained internal minimum 37552 and largest minimum
26612 bytes are unchanged. No reset or slow write is reported.
Normal IMU average is 42.72 Hz in the full minute preceding the test.

This passes malformed-line rejection, confirmed cancellation and clean retry.
Next: stop browser reads before requesting current, resume after about ten
seconds, then capture status before retrying. Expect reason=stalled only
if transport buffering fills and prevents a complete line for five seconds.
If the small file fits in host buffers, a successful transfer is inconclusive
for this gate and requires a larger-file test later. No firmware change.


## Stopped-reader test: cleanup observed, direct stall reply absent (14:52)

Boot 47 current retrieval requested at 14:52:16.869 with browser reads paused.
Buffered output is received around 14:52:27.913. No END or direct stalled
error appears. At 14:52:43.824 the page sends log abort after its own missing-
response timeout; firmware acknowledges reason=aborted at 14:52:43.846.
Final status retains USB bytes=9072, result=stalled, active=0, paused=0.
Logger is ready/synced, current size=147467, writes=73, queue=0/16 high=1,
zero drops/suppressed/truncated and error=none. Boot, memory minima and writer
margin remain unchanged: 47, internal minimum 37552, largest minimum 26612,
stack margin 3320 bytes. No successful download is reported.

This confirms a firmware-side stalled result and completed cleanup. It is
not a successful file transfer, nor direct timing proof of the five-second
no-progress guard. diagnostics_usb.cpp closes the reader and resumes appends
before entering its terminal-error phase. That phase retries error output
for up to another five seconds, then releases transfer state even when the
reply cannot be delivered. Thus a roughly eleven-second stopped-reader period
can outlast both the progress timeout and the error-delivery retry window.
This is a source-supported explanation for the missing reply, not a traced
proof of its exact delivery failure. The late idle log abort acknowledgment
does not replace the retained stalled result.

Next: all browser switches off, normal current.log retry and status, without
reset or reconnect. Inspect the saved log's USB_GET_END if available to
corroborate cleanup duration; the clean-retry portion of this case is pending.
No firmware changes or timeout changes.


## Recovery retry failed: boot 47, 2026-09-17 14:54

JP reports the ordinary current.log retry stopped around 68 percent. Request
at 14:54:52.623, page missing-response abort at 14:55:08.831, acknowledged
at 14:55:08.843. Final USB status retains bytes=101808 result=stalled,
active=0 paused=0. This is a second firmware-side stall, not just a browser
CRC rejection. No successful DOWNLOAD or END appears.

Logger remains ready/synced, hooks=0, same boot 47, current size=148929,
writes=77, queue=0/16 high=1, no drops/suppressed/truncated or storage error.
Internal minimum=37552, retained largest minimum=26612 and stack margin=3320
are unchanged. Thus logging cleanup works, but recovery of USB retrieval
has failed this test. Stage 1B is not accepted. Earlier successful ordinary
transfers remain valid observations; the newer core has not eliminated all
USB failures. Whether the pause test caused persistent state or this is an
independent intermittent stall is unresolved.

Source inspection: sendLine waits for space for an entire protocol line,
then calls HWCDC.write once. A space timeout or short/failed write can both
produce result=stalled. The installed 3.3.11 HWCDC write path re-arms TX
interrupts; availableForWrite only checks free space. This is a candidate
pacing/driver interaction, not an established cause. The detailed phase,
tx_free and write_bytes fields are currently only in the lost terminal
error reply, not retained USB status. Firmware status cannot distinguish
these branches from the supplied capture. No timeout or driver patch made.

Next isolation test: use the web page's Disconnect and Connect only, keeping
USB physically connected, with explicit DTR=true/RTS=false and test switches
off. Capture status before retry (confirm same boot/continuing uptime), then
normal current.log retrieval and status. A reset would invalidate a claim
that reopening the host session alone restored transfers. Do not use the
VS Code monitor or unplug USB for this step. If it freezes, stop the test
and report that observation. Documentation only; no build/commit/push.


## Reopening web serial recovered download without reset: 14:58

JP confirms success. Connection opens at 14:58:25 with explicit DTR=true,
RTS=false; boot remains 47, uptime continues to 2284157 ms at final status.
The same-boot observation distinguishes reconnecting serial from rebooting.
Current retrieval saves 150788 bytes in 1.53 s, CRC OK, decoded 98522 B/s,
wire 138245 B/s. Final USB active=0 paused=0 bytes=150788 result=ok.
Logger ready/synced, current size=151517, queue=0/16 high=1, zero drops,
suppression, truncation and errors. Stack margin=3320, internal minimum=37552
and retained largest minimum=26612 remain unchanged. Auto-list briefly
reports active=1 and resets published bytes to zero while retaining the
previous stalled result; after listing active=0, as expected.

Inspected the actual saved file C:/Users/photo/Downloads/47-current (5).log,
150788 bytes. Its USB_GET_END records provide new device-side evidence:

- Paused-reader attempt: seq=72, up_ms=1902769, bytes=9072,
  duration_ms=5133, result=stalled. This is consistent with the five-second
  no-progress guard after initial transmission; last-progress time itself
  is not recorded. A subsequent health record confirms continued appends.
- Normal failed retry: seq=77, up_ms=2054449, bytes=101808,
  duration_ms=1054, result=stalled. This CANNOT be the five-second timeout
  for that attempt: total duration was only about one second.

Correction/refinement to the preceding failure discussion: the normal retry
was terminated early by a path mapped to stalled, not by five seconds of
no progress. sendLine can report a short/failed write or failed connection
check; its negative return maps to stalled. A transient stop condition on
its -2 path can also fall back to stalled if the condition has cleared by
the next check. The existing evidence does not distinguish these paths.
Detailed phase/space/write-return evidence was only in the missing terminal
reply. The page's later 15-second timeout is separate from device failure.

Reopening restored one transfer; it does not prove a persistent host fault
or complete recovery reliability. Recommended next change is diagnostic
only: retain the exact stop path and last send phase, available space, line
length and returned write length in serial status, independently of terminal
error delivery, before another deliberately stalled transfer. Preserve all
transport behavior and limits. Not implemented in this turn. Stage 1B pending.


## Diagnostic-only follow-up prepared with JP approval

The web page is compacted as requested: remove capability/signal explanation
paragraphs and the bottom bench instructions. Keep signal controls, runtime
checks and connection/error console messages.
Firmware now retains the last stalled transfer's exact caller path, send check,
pre-write stop reason, phase, file, uptime, elapsed/idle time, payload bytes,
line length, available space and write return. Two ordinary status lines
([LOG USB FAIL] and [LOG USB SEND]) expose the snapshot even after a lost
error reply. Reconnect, list, late abort and successful retry preserve it;
another stall or reboot replaces it. No transport behavior or limit changes.
See [handoff](../src/diagnostics/STAGE1B.md) for field meanings and first test.
Source only: JP builds and flashes; no commit or build by Codex.


## Retained-diagnostic build: first ordinary download passed (15:11)

JP reports closing the VS Code monitor again required unplugging/replugging
USB; the board restarted before connecting the web page. The monitor-close
problem remains unresolved and is separate from this transfer result.
The capture shows boot 49 with continuous uptime during the web test.
Both new LOG USB FAIL/SEND lines appear before and after retrieval, proving
the new diagnostics are in the flashed build. valid=0 and the unset fields
are expected because this boot has not recorded a stalled transfer.

Current download: 160243 bytes in 1.61 s, CRC OK, decoded 99647 B/s,
wire 139859 B/s. Final current size=160404, USB active=0 paused=0,
bytes=160243 result=ok. Logger ready/synced, hooks=0, queue=0/16 high=1,
zero drops/suppressed/truncated and error=none. Writer stack margin=3320
after transfer (3768 before), used maximum=4872, matching prior successful
transfer high-water use. Internal minimum=94856, sampled largest minimum=51188;
no new low during retrieval. No reset during the captured transfer.

Next: reproduce the paused-reader/retry sequence without resetting or
reconnecting. Pause browser reads, request current, unpause after about ten
seconds, await failure/abort completion, and capture status. Then normal
current retry with all switches off, await completion, capture status again.
The first status preserves the deliberate stall evidence before a later
stall can replace it. Inspect path/check/stop and elapsed/idle times rather
than treating all stalled labels as five-second expiries. Hardware coverage
of valid=1 capture and retention remains pending. No new code changes.


## Retained diagnostics verified: deliberate stall and retry pass (15:13)

Boot 49: request at 15:13:45.981, direct stalled reply received at
15:13:53.276. Retained failure: valid=1, at_ms=248461, elapsed_ms=5138,
idle_ms=5000, phase=data, path=stop_guard. Last send observation: bytes=9216,
line_bytes=201, tx_free=119, write_bytes=-1, check=space, stop=none.
This directly identifies the five-second no-progress guard: only 119 bytes
of TX space were available for a 201-byte line, and write was not called
on that attempt. It is not a short-write failure. The browser received the
original error this time, without its later missing-response abort.
The elapsed host request-to-error receipt is 7.295 s; exact checkbox timing
is not logged. Do not equate this with the earlier roughly eleven-second
read-pause case where the terminal reply was absent.

JP confirms the all-switches-off retry downloaded successfully. Normal retry
at 15:14:17.508 saves 162580 bytes in 1.62 s with CRC OK, decoded 100451 B/s,
wire 141001 B/s. Same boot/connection, no reboot or reconnect shown. Final
USB active=0 paused=0 bytes=162580 result=ok. Both retained diagnostic lines
still describe the earlier deliberate stall, verifying persistence across
a successful retry. valid=1 is historical, not a new failure or active stall.

Logger ready/synced, hooks=0, queue=0/16 high=1, zero errors, drops, suppression
or truncation. Current grows 162419 -> 162742 bytes; writes 15 -> 17.
Internal minimum=94856, sampled largest minimum=51188, writer margin=3320
are unchanged. This passes this deliberate timeout/notification/clean-retry
case and the new snapshot reporting. It does not resolve the earlier 1054 ms
normal-transfer failure, which has not recurred with detailed evidence.
Next: three ordinary current downloads on the same connection, all switches
off, status after each; stop on any failure and retain the diagnostic lines.
No new firmware change, build or commit.


## Three ordinary downloads passed: boot 49, 2026-09-17 15:16

JP confirms all downloads worked. All three browser CRC checks pass:

| Current snapshot bytes | Duration | Decoded B/s | Wire B/s |
|---:|---:|---:|---:|
| 164029 | 1.64 s | 99853 | 140167 |
| 164352 | 1.69 s | 97215 | 136462 |
| 165239 | 1.75 s | 94465 | 132605 |

All post-status captures remain boot 49, logger ready/synced, hooks=0,
queue=0/16 high=1, zero errors/drops/suppression/truncation and USB idle/unpaused
with result=ok. Final current size=165401. Writer margin=3320 and used
maximum=4872; internal minimum=94856 and largest minimum=51188 stay unchanged.
These are stable retained minima, not a full heap-leak proof. The retained
stall stays at up_ms=248461, bytes=9216, path=stop_guard, idle_ms=5000,
confirming successful downloads do not replace the diagnostic snapshot.
The earlier unexpected one-second failure has not recurred and is not fixed
by this diagnostic-only change.

Next focused reproduction: pause browser reads for at least twelve seconds
after requesting current, then resume and await any failure/abort completion.
Capture status before an ordinary retry, and again afterward, without reset
or reconnect. This covers a pause longer than the five-second progress guard
and subsequent five-second terminal-reply window; the last successful
pause/retry capture received the error only 7.295 s after its request.
The aim is to reproduce the original late-resume condition with retained
send diagnostics, not to rerun normal throughput checks.
No firmware changes, build, flash, commit or push.


## Longer stopped-reader case and retry passed: 15:19

Boot 49 request at 15:19:23.181; stalled error received at 15:19:38.866
(15.685 s later). Snapshot: at_ms=585660, elapsed_ms=5124, idle_ms=5000,
phase=data, path=stop_guard; bytes=9072, line_bytes=201, tx_free=104,
write_bytes=-1, check=space, stop=none. This again identifies the expected
five-second no-progress guard, not an early failed/short write. The error
was delivered despite the longer pause. Host receive time is not send time:
a terminal reply may have entered USB/host buffers before the device's
five-second error-delivery retry window expired. This does not prove it
was still retrying the reply at 15.685 seconds.

JP confirms the normal retry succeeded, with no intervening reconnect shown:
167580 bytes in 1.74 s, CRC OK, decoded 96249 B/s, wire 135115 B/s.
Final current size=167742, writes=33; logger ready/synced, hooks=0,
queue=0/16 high=1, zero errors/drops/suppression/truncation. USB idle/unpaused,
bytes=167580 result=ok. Same boot 49, internal minimum=94856, sampled largest
minimum=51188, writer margin=3320 unchanged. The retained failure still
identifies the deliberate stall. JP omitted the intermediate status before
retry, but the delivered stall error and surviving snapshot cover this case;
no repeat needed.

Stop repeating this pause case now. The earlier 1054 ms ordinary-transfer
failure remains unresolved, with diagnostic capture ready if it recurs.
Next ordinary-use case: MQTT off, wait for the first failed retry to finish,
download current plus newest three, restore MQTT with on, then status.
This checks retrieval with MQTT unavailable, not sustained overlap with a
blocking retry. Large-file throughput and sustained overlap remain pending.
No firmware change, build, flash or commit.


## Retrieval while MQTT unavailable passed: boot 49, 15:22

JP reports downloads worked. MQTT off at 15:22:05.799 keeps Wi-Fi connected.
Ghost-broker attempt 1 fails after 5004 ms; attempt 2 fails after 5005 ms.
Between these attempts the bundle completes with CRC OK for every file:
current=169031 bytes in 1.66 s, archive16=7942 in 0.11 s, archive15=7936
in 0.13 s, archive14=8059 in 0.13 s. Current decoded rate=101986 B/s,
wire rate=143172 B/s. Total payload=192968 bytes; wall-clock command-to-last
completion spans 2.086 s. Browser duration rounding and console scheduling
mean individual durations need not sum exactly to that wall-clock span.

MQTT on at 15:22:39.390 restores the real broker in a 508 ms attempt;
NET returns to REMOTE CONNECTED. Final status: same boot49, logger ready/synced,
hooks=0, queue=0/16 high=1, no errors/drops/suppression/truncation, USB
idle/unpaused with result=ok and final archive bytes=8059. Current=170770,
writes=44. Writer margin=3320 and largest minimum=51188 unchanged.
Internal free low since boot falls from 94856 to 94596 during the real
MQTT connect probe; do not attribute that 260-byte change to download leakage.
Ghost attempts have sampled largest minimum=63476; real connect=55284.
Historical deliberate stall snapshot at up_ms=585660 remains unchanged.

This passes retrieval with MQTT unavailable and subsequent broker recovery.
Transfers run 15:22:25.139-27.225, after first retry ended at 15:22:15.902
and before second began at 15:22:31.004, so this does not test USB sending
while a blocking MQTT connection attempt is in progress.

Next useful gate is a managed 2 MiB archive for sustained throughput,
three-times current deadline margin, and later overlapping MQTT/Live tests.
The current file is only about 171 KB and existing hooks provide no simple
large-file generator. Recommend a test-only writer-owned fixture command
to avoid another card removal; this is a proposal, not implemented here.
The unexpected early USB stall and monitor-close issue remain open.
No code change, build, flash, commit or push.


## JP-approved fixture creation and deletion prepared

Added log test file (2 MiB synthetic managed archive) and log test del <number>
(exact-pattern validation before deletion) behind DIAG_USB_TEST_FIXTURE.
The flag is 1 for this bench handoff; fault hooks stay 0. Restore the fixture
flag to 0 for normal builds after completing and deleting the test fixture.
Existing writer, one 1 KiB batch per turn, normal logging priority, no new
allocation/task/NVS call. Exclusive temporary creation refuses collisions
and insufficient headroom; generation skips preserve current/archive names.
Completed files are synced/closed before publication. Deletion verifies all
2097152 bytes before removal and cannot target current.log by command.
See [fixture handoff](../src/diagnostics/STAGE1B.md) for lifecycle and tests.

Known fixture CRC32=8D218D21. Desktop browser suite now has 14 passing checks;
Python expected-byte verifier also rejects corrupted/truncated/non-fixture
files. No firmware build, flash, commit or push. The first hardware step is
create, one normal 2 MiB download, status and independent saved-file check.
Keep the fixture until the remaining large-file tests are complete, then use
the serial delete command and restore the bench flag.


## First 2 MiB fixture download failed: boot 51, 15:39

JP generated archive-00000018.log successfully: exactly 2097152 bytes,
result=ok, errno=0. Creation ran from 15:38:24.500 to 15:39:17.023
(52.523 s). The fixture remains on the card for further tests.

Download started at 15:39:26.573. The page requested abort at 15:39:51.956
and received confirmation, reporting Missing response or END. Subsequent
status retained bytes=904320 (43.125% of the file), result=disconnected,
USB inactive and unpaused. This is not the earlier stalled result.
The firmware byte count represents submitted payload, not verified host receipt.
The 120-second overall limit applies only to current.log, not this archive.

Same boot 51 continued to respond. Logger ready/synced, hooks=0, queue=0/16
high=1, zero drops and errors. Free internal low=92524; sampled largest
minimum=49140; writer PSRAM stack margin=3336. No memory-gate failure appears.
LOG USB FAIL valid=0 is a diagnostic limitation: finishError currently saves
only stalled failures, not disconnected failures. It does not mean no failure.

Source inspection: diagnostics_usb.cpp stopReason immediately ends the
transfer when USBSerial.isConnected() returns false and sends no terminal
reply for disconnected. That explains the later browser timeout and abort.
Installed core 3.3.11 HWCDC.h maps isConnected to isCDC_Connected; HWCDC.cpp
returns false immediately when isPlugged is false. Its comments explicitly
identify transient SOF-watchdog false readings on healthy links. It also
returns false until the TX interrupt establishes connected again.
This supports a transient connection indication as a hypothesis, not proof
of a cable disconnect or the exact cause of this instance.

Recommended next change, not implemented here: tolerate a brief false USB
connection indication without sending while disconnected; retain a bounded
connection-loss guard and the existing five-second no-progress protection.
Extend retained failure evidence to disconnected outcomes, including their
reason and duration. Do not change the core, CRC checks or overall deadlines.
Then retry the same archive with all browser test switches off; no new fixture
or card removal is needed. The large-file throughput gate remains pending.
Documentation only; no firmware change, build, flash, commit or push.


## Connection-loss tolerance prepared after boot 51

JP approved this correction. The writer stops sending immediately on a false
USB connection reading, but aborts as disconnected only after 1000 ms of
continuously observed loss. Recovery before that resumes the pending line.
The five-second no-progress deadline remains active during loss or flapping;
connection checks do not reset progress. Current retains its 120-second limit.
Archives still have no overall time limit. Abort, queue pressure and shutdown
checks continue to run. No sleep, new task or per-line serial output was added.

Both pre-write connection checks now return retry without writing on a false
reading. A final stop check that observes loss also prevents the write.
Control replies use their existing bounded retry period for transient loss.
Short or failed writes still terminate the transfer; they are never retried
as a whole line, which would risk duplicate or corrupt protocol data.
A connection change inside the core's write call remains possible; this
change cannot make connection checking and driver writing atomic.

Retained LOG USB FAIL now covers stalled and disconnected outcomes and adds
reason and loss_ms. LOG USB LINK reports losses, max_loss_ms, pending and
grace_ms=1000 for the latest transfer. These observations reset at the next
list/download, so request status before Refresh files or another download.
The failure snapshot survives successful retries, listing and late aborts.
A sustained disconnect still closes the reader and resumes current appends
before release; no reply is attempted while classified disconnected.

Desktop validation: ten source-level guard simulations and fourteen browser
protocol checks pass. Simulation covers brief loss, continuous loss, loss
at each pre-write check, flapping, unchanged deadlines, abort/queue/shutdown,
short writes and control replies. This is not a C++ firmware build or a
hardware validation. JP builds and flashes from VS Code. No commit or push.

Next bench test: build/flash the 3.3.11 profile, keep the card installed and
archive18 unchanged, connect with explicit DTR=true RTS=false, all browser
fault switches off, hotspot on and Live stopped. Download archive18 once,
then send status before any refresh/retry. Send the whole console, saved-file
path if successful, and whether the board stayed responsive. Do not regenerate
or delete the fixture yet. Only src changed, so this update does not require
removing the generated companion.ino.cpp. Large-file acceptance remains CRC
and independent byte verification, no unexpected failure, and at most 40 s
for the initial 120-second current-file deadline's three-times margin.


## 2 MiB download and independent integrity passed: boot 53, 15:52

JP reports successful download after the connection-loss correction.
Archive18: 2097152 bytes in 20.39 s; decoded 102859 B/s, protocol wire
145178 B/s, CRC OK. Desktop verification of
C:/Users/photo/Downloads/53-archive-00000018.log matches every expected byte.
CRC32=8D218D21; SHA256=
b79a649116ba358243b2c9388b68ac718b9f65cef94f241236ad8550394f65be.

LOG USB LINK reports losses=1 max_loss_ms=3 pending=0 grace_ms=1000.
The transfer successfully tolerated an observed three-millisecond false
connection indication. This validates the recovery path and supports the
boot51 transient-loss hypothesis; it does not retrospectively prove that
failure's exact cause or establish that all USB problems are fixed.
Retained failure valid=0, USB idle/unpaused, bytes=2097152 result=ok.

Logger ready/synced, same boot53, hooks=0, queue=0/16 high=1, no drops,
suppression, truncation or errors. Internal low=95088, sampled largest
minimum=51188, writer PSRAM margin=3320 (used4872). Fixture status archive=0
result=none describes this boot's generator activity, not absence of archive18;
the existing file survived the reflash and was downloaded successfully.

The 120-second current limit is 5.89 times this 20.39-second transfer time,
above the required three-times margin (61.17 s). Keep the limit unchanged.
This passes ordinary 2 MiB throughput and independent synthetic integrity.
Stage1B remains open for the remaining interference/abort/pause cases.

Next single test: with all browser fault switches off and Live stopped,
send off, immediately download existing archive18 so its roughly twenty-second
transfer overlaps the first ghost-broker retry, then send on after completion
(or abort confirmation). Wait for MQTT recovery and send status. Capture the
whole console including PROBE, MQTT attempts, download result and USB diagnostics.
Keep the card installed and retain archive18; no new build or fixture needed.
No firmware change, build, flash, commit or push for this results review.


## 2 MiB retrieval overlaps blocking MQTT retry: boot 53, 15:55

JP reports download complete. off at 15:55:25.675; attempt 1 runs
15:55:30.778-35.782 (5004 ms). Browser sends log get 18 at 15:55:31.599,
while the main loop is blocked, so command processing must wait for attempt 1.
Attempt 2 runs 15:55:50.881-55.885; download completes at 15:55:54.270,
inside that blocking attempt. This confirms actual transfer/connect overlap,
unlike the earlier short bundle between retry windows.

Archive18: 2097152 bytes, browser elapsed=22.67 s, decoded=92509 B/s,
wire=130570 B/s, CRC OK. Browser timing includes the initial command wait;
it is not a clean comparison of active transfer throughput with 20.39 s.
The saved C:/Users/photo/Downloads/53-archive-00000018 (1).log independently
matches every expected byte, CRC32 8D218D21 and the reference SHA256.
120 s still exceeds three times even this elapsed figure (68.01 s).

USB result=ok, inactive/unpaused, no retained failure. losses=1 max_loss_ms=4
pending=0 shows another brief USB connection indication recovered. Same boot53,
logger ready/synced, hooks=0, queue=0/16 high=1, zero errors/drops/suppression/
truncation. Writer margin=3320 and largest minimum=51188 unchanged.
Third ghost retry takes 5005 ms. on at 15:56:24.974 restores MQTT in 618 ms;
NET returns REMOTE CONNECTED. Final free-internal low=92472 occurs during
that real MQTT connect (previously 95088); do not attribute it to download leakage.
Normal IMU windows around the blocked calls average 42.72-43.61 Hz; the
connect windows themselves are excluded by the probe design.

Pass: 2 MiB integrity and retrieval during blocking MQTT reconnect, then
normal broker recovery. Stage1B still needs remaining large Live, abort,
current-pause/timeout and cleanup cases; the VS Code monitor-close issue
and the earlier unexplained short-write/stall case are not declared fixed.
JP requested a commit/push checkpoint of tested code, browser and documentation.
DIAG_USB_TEST_FIXTURE remains1 for the ongoing bench tests, fault hooks=0.
Its delete command exists but on-board deletion remains untested. Keep archive18.
