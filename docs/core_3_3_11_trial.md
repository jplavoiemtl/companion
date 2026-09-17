# Core 3.3.11 trial

Prepared for JP on 2026-09-17. JP requested committing and pushing this pre-flash checkpoint. JP confirmed successful compilation with core 3.3.11 and the separate Waveshare graphics 1.6.4 copy after the compatibility corrections. First flash and hardware results are pending.

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
