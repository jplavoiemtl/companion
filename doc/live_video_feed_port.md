# Live Video Feed – Port Plan from the Home Panel

## Purpose

Port the live camera feed feature from the `home_panel` project to the companion.

The home panel version is finished and in production: the Screen1 camera button shows a
live view of the entrance camera at **9.6 fps**, and a motion-detection event shows the
captured still for 10 seconds before rolling into a 60 second live feed. See
`home_panel/doc/live_video_feed.md` for the full design, six measurement runs, and two
approaches that were measured and rejected.

**This is not a copy-paste port.** The image *logic* transfers almost unchanged, but the
display stack, orientation model, colour byte order and CPU clock all differ.

## How the feature works (recap)

The ESP32-S3 has no hardware video decoder and the camera stream is H.264, so RTSP on the
panel is impossible. Instead the panel polls Frigate's snapshot API on the Jetson, which is
already decoding that camera continuously for object detection, and re-encodes single
frames as JPEG on request.

Four things make it fast enough:

1. **ESP32_JPEG** (SIMD) instead of TJpg_Decoder — 3.8× faster decode
2. **Prefetch** — the request for frame N+1 is issued before frame N is decoded and
   blitted, so network latency overlaps with ~80 ms of work instead of adding to it
3. **Zero-copy display** — LVGL clips an oversized frame instead of a centre-crop copy
4. **`WiFi.setSleep(false)`** — worth 10 ms and eliminated all network stalls

## Companion vs Home Panel

| | Home panel | Companion | Impact |
|-----------------|--------------------------|------------------------------|--------|
| Board | JC3248W535EN | Waveshare AMOLED 1.8" (SH8601) | — |
| Display stack | `esp_bsp` + `lv_port` | `Arduino_GFX` + `my_disp_flush` | Rewrite blit integration |
| Resolution | 480×320 landscape | 368×448 portrait | New geometry |
| CPU clock | 240 MHz | **80 MHz → changing to 240** | Decision made, see below |
| ESP32 core | 3.0.2 | 3.1.3 | Probably none |
| LVGL | 8.4.0 | 8.4.0 | None |
| `LV_COLOR_16_SWAP` | 1 | **0** | **Decoder output must be LE, not BE** |
| `LV_MEM_CUSTOM` | 1 (malloc) | 0 (LVGL static heap) | Watch LVGL heap headroom |
| LVGL draw buffer | Full screen, PSRAM | **1/10 screen, internal RAM** | Likely faster |
| Image orientation | Fixed `ROT_90`, toggling corrupts | **Toggled: `ROT_90` UI, `ROT_NONE` image** | Central design question |
| JPEG decoder | ESP32_JPEG + TJpg | **TJpg only** | Must add ESP32_JPEG |
| Flashing | OTA (ElegantOTA) | **USB only** | Slower iteration |
| Motion/button split | Had to be added | **Already exists** | Head start |
| Screen power module | Yes (night dimming) | No | One less interaction |

## What works in our favour

**No software rotation for images.** The companion displays images at `LV_DISP_ROT_NONE`
and only rotates the UI to `ROT_90`. On the home panel the 90° software rotation was
~33 ms of the 45 ms blit — the single largest cost after decode. If the feed can be shown
at `ROT_NONE` that cost disappears entirely.

**A faster draw path.** The draw buffer is `static lv_color_t buf[screenWidth * screenHeight / 10]`
— internal RAM, one tenth of the screen, so partial refresh from fast memory. The home
panel used a full-screen PSRAM buffer with `full_refresh = 1`. `my_disp_flush()` is a plain
`gfx->draw16bitRGBBitmap()` with no rotation work in it.

**The widget is already `lv_pct(100)`.** `ui_imgScreen2Background` is sized by percentage
and centre-aligned, exactly like the home panel's, so the zero-copy clipping crop works
identically — and the same rule applies: **never call `lv_obj_set_size()` on it.** Doing so
converts it to fixed sizing and causes a white flash on the next visit to the screen.

**The motion/button split already exists.** `requestLatestImage(bool fromNotification)`
already distinguishes an MQTT push from a button press. On the home panel this had to be
retrofitted. The still→video chain has a natural hook.

## Decisions already taken

**CPU clock 80 → 240 MHz** in both `sketch.yaml` profiles. Verified safe: nothing in the
sketch, headers or `src/` references `getCpuFrequencyMhz`, `setCpuFrequencyMhz`, `F_CPU` or
a hardcoded clock value, and the ESP32-S3 APB clock stays at 80 MHz regardless of CPU
frequency, so `Arduino_ESP32QSPI` display timing is unaffected.

It affects the whole project, not just video, so the existing inclinometer, G-meter and IMU
behaviour should be sanity-checked after the change. The board will run hotter and draw
more current.

**No 80 MHz build variant.** The two existing profiles (`amoled-1-8`, `amoled-1-8-no-bat`)
turn out to be identical — same FQBN options, same libraries, differing only by serial port
(COM4 / COM5). There is no battery variant today. If one is ever wanted it is a ten-line
copy-paste, so nothing is lost by deferring.

**Branch:** `test/live-video-feed`, merged to `main` once proven on the bench. Same approach
that worked for the home panel.

## Work items

### 1. Toolchain

- Add `ESP32_JPEG` to both profiles in `sketch.yaml`, using the same `dir:` style already
  used for `GFX_Library_for_Arduino`. It is already installed at
  `C:\Users\photo\Documents\Arduino\libraries\ESP32_JPEG`.
- Change `CPUFreq=80` to `CPUFreq=240` in both profiles.

### 2. Colour byte order — do not copy this from the home panel

The home panel has `LV_COLOR_16_SWAP 1` and therefore decodes to
`JPEG_RAW_TYPE_RGB565_BE`. **The companion has `LV_COLOR_16_SWAP 0` and needs
`JPEG_RAW_TYPE_RGB565_LE`.** `my_disp_flush()` confirms this — it selects
`draw16bitRGBBitmap()` rather than the `Be` variant. Getting this wrong produces a
recognisable image with wrong colours rather than an obvious failure.

### 3. Orientation — the main open design question

The companion's stills are **pre-rotated by ffmpeg** in Node-RED
(`crop=448:368:...,transpose=2`), producing a 368×448 portrait image that is displayed at
`ROT_NONE`. That is how the project avoids LVGL's software rotation.

**Frigate's snapshot API cannot rotate.** So a live frame arrives landscape and something
has to rotate it. Three options, in order of preference:

**Option A — rotate during decode (preferred).** ESP32_JPEG supports
`JPEG_ROTATE_90D` in `jpeg_dec_config_t`, but only when *both* dimensions are divisible
by 8. Frigate's `avant` detect stream is 640×360; 640 % 8 == 0 and 360 % 8 == 0, so
requesting the native size qualifies. Decoding with 90° rotation gives 360×640, which is
8 px narrower than the 368 screen and 192 px taller — crop the height with
`lv_img_set_offset_y()`, the same free-clipping trick used for width on the home panel.

Cost is unknown and must be measured. The library may rotate almost free or may be much
slower than an unrotated decode.

**Option B — show the feed at `ROT_90`** like the UI, letting LVGL rotate. Simple, but
reintroduces the per-pixel software rotation that Option A avoids, which on the home panel
was ~33 ms. Probably still viable at 240 MHz.

**Option C — pre-rotate server-side.** Would require an ffmpeg process on the Jetson rather
than Frigate's API, reintroducing a supervised process. Rejected on the home panel for the
same reason; keep as last resort.

**Measure A and B before committing.** If A is fast, take it; if the rotation penalty is
large, B is the fallback.

### 4. Geometry

Target 368×448 portrait. Under Option A the decoded frame is 360×640, so:

- Width 360 vs 368 → 8 px total, 4 px each side. Either accept a thin border or request a
  slightly larger source.
- Height 640 vs 448 → crop 192 px with `lv_img_set_offset_y(-96)` for a centred crop.

Frigate's resize parameter is **`height`, not `h`** — `h` is silently ignored. Confirm the
`width % 8 == 0` constraint still holds for whatever size is finally chosen.

### 5. Existing behaviour to respect

The companion's current values differ from the home panel's and should not be blindly
overwritten:

| Constant | Companion | Home panel |
|--------------------------------|-----------|------------|
| `SCREEN2_DISPLAY_TIMEOUT` | 60 s | 180 s |
| `SCREEN2_LOADING_TIMEOUT` | 20 s | 30 s |
| `MAX_JPEG_SIZE` | 128 000 | 60 000 |
| `HTTP_TIMEOUT_MS` | 15 s | 30 s |
| `NOTIFICATION_ECHO_WINDOW_MS` | 10 s | none |

**`NOTIFICATION_ECHO_WINDOW_MS` needs thought.** The companion drops an MQTT-pushed request
if an image reached the screen within the last 10 seconds, to suppress the image server's
own notification echoing back after a "new" press. The motion → still → video chain must
not be defeated by that suppression, and equally the suppression must keep doing its job.
Resolve before implementing the chain.

### 6. Module structure

Create `src/video/video_stream.{h,cpp}`, mirroring the home panel's. The config struct
needs the companion's five screen pointers rather than three.

`buttonNew_event_handler` moves from `image_fetcher.cpp` to the video module — the same
move as on the home panel. `ui_events.h` declares it and `ui.h` includes that, so the
SquareLine wiring resolves regardless of which translation unit defines it.

### 7. Guard against the crash the home panel hit

An MQTT image request arriving during a feed must be ignored. Without it,
`requestLatestImage()` runs `prepareForRequest()` → `cleanupImageRequest()` and frees
buffers the video module is still rendering from. **This reset the home panel's board.**
Two guards are needed: an early return in `requestLatestImage()`, and an early return in
`imageFetcherLoop()` because the two modules share Screen2.

## Phased plan

Same approach that worked on the home panel: **measure before building.**

### Phase 0 — toolchain

Change the CPU clock, add ESP32_JPEG, confirm the project still builds and the existing
features still work. No video code yet.

### Phase 1 — measurement spike

Port a stripped burst that fetches from Frigate and logs per-frame `ttfb`, `xfer`, `decode`
and `blit`. Measure **both** orientation options from work item 3.

Decision gate on `decode + blit`:

| Combined cost | Interpretation |
|---------------|--------------------------------------------|
| Under ~80 ms | Proceed, expect 8-10 fps |
| ~80–150 ms | Proceed at a lower target |
| Over ~150 ms | Reconsider; the feature may not be worth it |

Note the home panel's instrumentation cost ~11 ms per frame in per-frame `Serial.printf`,
so the production figure will be better than what Phase 1 measures.

### Phase 2 — the feature

Only once Phase 1 is acceptable: promote the burst into `src/video/video_stream.{h,cpp}`,
wire the Live button, add the motion chain, add both guards.

### Phase 3 — merge

Bench-test, then merge `test/live-video-feed` to `main`.

## Practical notes

**No OTA on the companion**, so every iteration is a USB flash. The home panel took about
seven flash cycles to reach production. Two traps already encountered on that project and
worth watching for here:

- **Wrong upload port.** Check the port in `.vscode/arduino.json` and `sketch.yaml` matches
  the port the board actually enumerates on. A build can succeed while the upload silently
  goes nowhere.
- **Stale `.ino` intermediate.** Arduino sometimes fails to regenerate
  `build/.../sketch/companion.ino.cpp`, silently compiling a previous version of the
  sketch. Files under `src/` are unaffected, so a change spanning both ships half applied.
  Delete the intermediate after editing the `.ino`, and verify the `.bin` is newer than
  every source before flashing.

**Adding OTA is out of scope** for this port, but would make iteration much faster and is
worth considering separately.

## Open questions to resolve during implementation

1. Is the companion's image source the same entrance camera? Node-RED's
   "ESP32 picture Companion" node pulls from the Jetson's object-detection stream on port
   8554, which is the same camera Frigate calls `avant`. Confirm the live feed should show
   that camera and not a different one.
2. Does `ROT_NONE` with a rotated-during-decode frame actually land the right way up? The
   rotation direction (90 vs 270) needs confirming on hardware.
3. Does the existing 10 second notification-echo suppression interfere with the motion
   chain?
4. How much LVGL heap headroom is there, given `LV_MEM_CUSTOM 0` means LVGL uses a fixed
   static pool rather than malloc?
