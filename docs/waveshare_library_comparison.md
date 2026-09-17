# Waveshare library comparison

Reviewed 2026-09-17 for JP. Review only: no libraries, build profiles or firmware changed; no build or flash.

## Reference and method

Compared the original SH8601/FT3168 board, not V2, at Waveshare revision
`78e13f852929c2ab4f9d5e0ad1c50ea378dbf2b4`.

- [Bundled libraries](https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-1.8/tree/78e13f852929c2ab4f9d5e0ad1c50ea378dbf2b4/examples/arduino/libraries).
- [CI toolchains](https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-1.8/blob/78e13f852929c2ab4f9d5e0ad1c50ea378dbf2b4/docs/CI.md) specify Arduino-ESP32 3.3.11 with the matching bundled libraries. Compile coverage is not a companion hardware test.
- Local selection comes from `sketch.yaml` and `build/build_amoled-1-8/includes.cache`, rather than every library installed in the sketchbook.
- Compared Git blob hashes against local C/C++ headers and sources under `src/`, plus root headers, sources and properties. Normalized CRLF to LF. Inspected relevant changed sources separately. Examples and other assets were not exhaustively compared.

## Differences and recommendations

| Component | Companion build | Waveshare bundle | Recommendation |
|---|---|---|---|
| ESP32 core | 3.1.3 | CI uses 3.3.11 | Isolated upgrade trial; preserve the current profile and dependencies for rollback. |
| I/O expander | ESP32_IO_Expander 0.0.3 | Adafruit_XCA9554 1.0.0 | Main migration candidate. Requires changing initialization and dependency selection. |
| Expander dependency | Adafruit BusIO not selected by this build | Adafruit_BusIO 1.17.4 | Required by XCA9554. The installed sketchbook copy is already 1.17.4; its 10 compared files match. |
| Arduino GFX | 1.4.9 baseline; 1.6.4 revised trial | 1.6.4 | Unchanged 1.4.9 failed to compile with core 3.3.11. JP accepted a separate maker copy and the API adaptations. |
| Arduino_DriveBus | 1.0.1 local folder | 1.0.1 | Keep. Compared functional source is unchanged. |
| SensorLib | 0.3.1 pinned | 0.3.3 | Optional later update. QMI8658 driver and register constants match. |
| XPowersLib | 0.2.6 | 0.2.6 | Keep. All 12 compared files match. |
| LVGL | 8.4.0 | 8.4.0 | Keep. All 358 common compared files match; remote has 74 additional demo-related source/header files. |
| lv_conf.h | Project configuration | Bundled configuration | Same text after newline normalization. Keep our project-owned copy. |
| PubSubClient | 2.8 | Not bundled | Keep; absence from the demo bundle is not a discrepancy requiring removal. |
| TJpg_Decoder | 1.1.0 | Not bundled | Keep our still-image decoder. |
| ESP32_JPEG | 0.0.1 local folder | Not bundled | Keep our Live decoder; reassess SDK compatibility in a core trial. |
| Mylibrary, ui_a, ui_b, ui_c | Our pin configuration and generated UI | Demo pin configuration and UIs | Do not copy over companion files or generated UI. Mylibrary contains pin_config.h. |

There is also SensorLib 0.1.6 in the sketchbook, but the build cache confirms that the profile uses the internal 0.3.1 installation. Comparing only sketchbook versions would give the wrong answer.

### I/O expander: likely way past the I2C conflict

Our `ESP_IOExpander.cpp` includes `driver/i2c.h` and calls `i2c_driver_install()`.
The current link map attributes that legacy driver dependency to ESP32_IO_Expander.
This supports the documented 3.2/3.3 legacy-versus-new-I2C conflict in README.md.

Waveshare's current examples use `Adafruit_XCA9554` at address 0x20, through `Adafruit_I2CDevice` and Arduino `Wire`.
Our expander is used in `initIOExpander()` to reset pins 0, 1 and 2.
The companion already initializes Wire first.
A migration can preserve these pins, the 20 ms reset interval and existing ordering.
The old library dependency must also be removed from the trial profile.
Removing this legacy-driver user is a source-supported migration approach; successful companion operation on 3.3.11 is still untested.

Source: [current Waveshare Widgets example](https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-1.8/blob/78e13f852929c2ab4f9d5e0ad1c50ea378dbf2b4/examples/arduino/examples/13_LVGL_Widgets/13_LVGL_Widgets.ino).

### Graphics: actual compatibility changes

The local SH8601 constructor takes `(bus, reset, rotation, ips, width, height)`.
The bundled 1.6.4 constructor removes `ips` and adds optional offsets after width and height.
Leaving our old arguments in place can silently shift dimensions into the wrong parameters.
`Display_Brightness()` becomes `setBrightness()`; `SetContrast()` becomes `setContrast()`.
Our current brightness call therefore needs porting. Rotation behavior and the OLED base class also changed.

The QSPI default changes from 8 MHz to 40 MHz, but companion explicitly passes 20 MHz.
Preserve that accepted 20 MHz setting.
The transfer buffer setting changes from 8192 to 1024 pixels.
Both versions allocate two DMA buffers of twice that pixel count: nominal buffer payload falls from 32768 to 4096 bytes, before allocator and SPI-driver overhead.
That may improve internal memory availability, but changes transaction granularity; measure Live performance and memory again.
Do not present the nominal 28672-byte difference as a measured heap gain.

The QSPI source also adds newer-core SPI configuration fields and fixes shared-bus release handling.
Our bus uses the default non-shared mode, so the latter does not explain the USB download failure.
Across compared graphics files, 107 match, 85 differ, and 17 are present only in the remote bundle.
These counts include metadata and formatting changes, not just behavioral changes.

Sources: [QSPI header](https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-1.8/blob/78e13f852929c2ab4f9d5e0ad1c50ea378dbf2b4/examples/arduino/libraries/GFX_Library_for_Arduino/src/databus/Arduino_ESP32QSPI.h),
[QSPI source](https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-1.8/blob/78e13f852929c2ab4f9d5e0ad1c50ea378dbf2b4/examples/arduino/libraries/GFX_Library_for_Arduino/src/databus/Arduino_ESP32QSPI.cpp),
[SH8601 header](https://github.com/waveshareteam/ESP32-S3-Touch-AMOLED-1.8/blob/78e13f852929c2ab4f9d5e0ad1c50ea378dbf2b4/examples/arduino/libraries/GFX_Library_for_Arduino/src/display/Arduino_SH8601.h).

### Touch, IMU and configuration

DriveBus has 18 identical compared files. The other three differences are the properties name/spacing and comment whitespace in ETA4662 and CST2xxSE headers.
The FT3x68 touch and bus implementations match.

SensorLib has 134 identical compared files, 11 changed files and three added files.
The QMI8658 driver, constants, Wire helpers and relevant common communication files match.
Changes include RTC, other sensor and touch drivers, plus version metadata.
Updating SensorLib is therefore not a demonstrated fix for either our IMU or USB problem.

## Suggested next action

The first experiment kept graphics 1.4.9, but it failed to compile the changed SPI clock-divider API. JP then accepted a separate maker-bundled 1.6.4 copy for the 3.3.11 trial, with the SH8601 constructor and brightness calls adapted. Rollback profiles retain 1.4.9.
See [core trial preparation](core_3_3_11_trial.md) for the implemented profile and SDK checks.
Keep LVGL, DriveBus, XPowersLib, MQTT and JPEG versions fixed initially; defer SensorLib unless the trial exposes a need.
Keep the exact current local library folders available instead of overwriting the shared sketchbook.
The gitignored sketch.yaml and external library folders need explicit backups or separate trial paths; a Git branch alone does not preserve them.

Before JP flashes, recheck the new SDK's PSRAM stack, internal TCB, cache/flash, SDMMC DMA and task-lifecycle assumptions from the memory review.
After a successful build, first check startup, display and touch, then the same archive-14 USB download that currently stalls.
If it succeeds, repeat memory, Live, IMU and shutdown checks against a same-session baseline. The 20480-byte largest-internal-block gate remains unchanged.

HWCDC is part of the ESP32 core, not one of the libraries in this folder.
The previously identified upstream HWCDC fixes motivate the core trial; replacing unrelated demo libraries alone will not bring in those fixes.
Waveshare also sets USB TX timeout to zero for demo debug output. Do not copy that policy into our checked log-transfer protocol without a separate review.
