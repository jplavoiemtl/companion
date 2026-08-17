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
| CPU clock | 240 MHz | **240 MHz** (was 80, changed in Phase 0) | Done |
| ESP32 core | 3.0.2 | 3.1.3 | Probably none |
| LVGL | 8.4.0 | 8.4.0 | None |
| `LV_COLOR_16_SWAP` | 1 | **0** | **Decoder output must be LE, not BE** |
| `LV_MEM_CUSTOM` | 1 (malloc) | 0 (LVGL static heap) | Watch LVGL heap headroom |
| LVGL draw buffer | Full screen, PSRAM | **1/10 screen, internal RAM** | Likely faster |
| Image orientation | Fixed `ROT_90`, toggling corrupts | **Toggled: `ROT_90` UI, `ROT_NONE` image** | Central design question |
| JPEG decoder | ESP32_JPEG + TJpg | ESP32_JPEG + TJpg (added in Phase 0) | Done |
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

## Blocking issue: the companion cannot reach Frigate

**Found after Phase 0, from a serial log.** The port plan assumed a LAN path to Frigate
that does not exist on this board.

`secrets_private.h` contains `#define CAR`, described in the header as *"Car module: ONLY
uses iPhone (both ssid1 and ssid2 = iPhone)"*. In practice the companion runs on a phone
hotspot:

```text
Attempting to connect to primary network: iphone-jp
IP: 172.20.10.2
Initiating HTTPS GET: https://<remote-image-server>/esp32/latest?token=***
```

It is on `172.20.10.x`, not the home LAN, and fetches images through the remote HTTPS
server over the internet. **Frigate is LAN-only at `<frigate-host>:5000` with no
authentication, so the companion cannot reach it.**

Both LAN services are confirmed up and mutually reachable — Node-RED and Frigate each
return HTTP 200 from a machine on the home LAN, and they are on the same subnet.

### Options

**A1 — proxy Frigate through Node-RED. CHOSEN.** Implemented as `/esp32/live`; the
importable flow is in `doc/node-red/esp32_live_endpoint.json`. Five nodes on the
"ESP32 Companion" tab:

```text
http in GET /esp32/live -> Build Frigate URL -> http request -> Set image headers -> http response
```

`height` and `quality` pass through as query parameters, clamped (height 120-480, quality
10-80, defaulting to 360 and 40). Clamping matters because this endpoint reaches an
internal service and must not forward arbitrary values. `persist: true` keeps the
connection to Frigate alive, which matters at several requests per second.

**Frame rate lives on the panel, not here.** The endpoint serves one JPEG per request; the
companion paces itself. The chosen target is **3 fps**, giving roughly 2.2 MB of cellular
data per 60 second view at quality 30 — against 12.3 MB at 10 fps.

**Original A1 description:** Add an endpoint alongside the existing
`/esp32/latest`, which fetches from Frigate and returns the JPEG. Reuses the proven TLS
and token path with **no reverse-proxy reconfiguration**. Cost: Node-RED sits in the
per-frame path, so at 10 fps it handles 10 proxied requests per second while also running
the house automation.

**A2 — reverse-proxy route straight to Frigate.** Fewer hops and Node-RED stays out of it,
which is architecturally better for a video stream. Requires reverse-proxy configuration,
and the proxy must enforce authentication because Frigate's port 5000 has none.

**B — reduce the frame rate for the cellular case.** Independent of A1/A2 and probably
wanted regardless. Data cost per view:

| Rate | Duration | Frame size | Data per view |
|------|----------|------------|---------------|
| 10 fps | 60 s | 20.5 KB | **12.3 MB** |
| 3 fps | 60 s | 20.5 KB | 3.7 MB |
| 3 fps | 60 s | 12 KB (q=30) | **2.2 MB** |
| 2 fps | 30 s | 12 KB (q=30) | 0.7 MB |

12 MB of cellular data per button press is hard to justify. At 3 fps you can still see who
is at the door, and the use case here — checking the front door while away from home —
does not need smooth motion. A lower rate is arguably a *better* fit for this board than
the home panel's 10 fps.

**C — only enable the feed on the home LAN.** Simple and honest, but with `#define CAR`
the board is essentially never on the home network, so the feature would almost never be
available. Rejected unless the network situation changes.

### The image proxy blocks A1 as built

The remote path is not Synology → Node-RED directly. There is an `esp32-image-proxy`
container on the Pi that sits between them:

```text
Companion (cellular) -> Synology :9835 -> esp32-image-proxy (Pi:5000) -> Node-RED (Pi:1880)
                                          ^ validates API_TOKEN, rate limits
```

**This corrects an earlier note in this document.** The `?token=` *is* enforced — by the
proxy, not by Node-RED. An earlier probe returned 200 without a token only because it went
straight to Node-RED on 1880 and bypassed the proxy.

Probing the proxy without credentials reveals two blockers:

| Probe | Result | Meaning |
|-------------------------|----------|-------------------------------------|
| `/health` | `{"rate_limit":"10 requests/minute","status":"healthy"}` | Rate limit is per **minute** |
| `/esp32/latest` no token | 401 | Token enforced, path recognised |
| `/esp32/live` no token | **404** | Path **not whitelisted** |
| `/esp32/bogus` no token | 404 | Confirms whitelisting, not a typo |

**1. The path is not whitelisted.** `/esp32/live` returns 404 rather than 401, so the proxy
does not recognise it at all and the token never even gets checked. The Node-RED endpoint
is unreachable from outside until the proxy learns about it.

**2. The rate limit is 10 requests per minute.** A 3 fps feed is 180 requests per minute —
**18× over the limit.** The proxy was designed for occasional still images, where 10/min is
generous. Per-frame requests fight that design.

Both live in `app.py` at `/home/pi/appjpl/esp32-proxy/app.py`, mounted read-only into the
container.

### Resolving it — done

The updated proxy is version-controlled at `doc/proxy/app.py`. It previously existed only
on the Pi at `/home/pi/appjpl/esp32-proxy/app.py`.

Four changes, keeping the existing structure and per-endpoint style:

1. **`/esp32/live` route added**, forwarding only `height` and `quality` through to
   Node-RED, which clamps them.
2. **`RATE_LIMIT_LIVE`** (env var, default 400/minute) separate from the stills' 10/minute.
   3 fps is 180/minute sustained; 400 covers the panel running faster than expected or two
   sessions overlapping in the sliding window, while still bounding the endpoint.
3. **Separate rate-limit bucket.** `request_counts` is now keyed by `(ip, bucket)`. Without
   this the live traffic and the still endpoints shared one per-IP allowance, so a minute of
   video would lock out the Latest and Back buttons. **This was the subtle part** — a higher
   limit alone would not have fixed it.
4. **Per-request logging suppressed for the live bucket.** At a few frames per second the
   existing INFO lines would produce hundreds of entries a minute and bury everything else.
   Failures are still logged.

`require_token` now works both bare and with arguments, so the six existing endpoints are
untouched.

Verified: the file compiles, and the shipped `check_rate_limit` was exercised directly —
the default bucket blocks at 10 while the live bucket remains available, confirming the
separation.

### If the proxy could not be changed

`app.py` needs changing either way, since the path must be whitelisted. While there:

- **Whitelist `/esp32/live`** and ensure `height` and `quality` query parameters are
  forwarded, which the endpoint depends on.
- **Give the live path its own rate limit** — around 300/minute for 3 fps with headroom —
  while leaving the stills at 10/minute.

An alternative worth noting: Frigate's **MJPEG** endpoint would be a single request for a
whole viewing session, costing one unit against the rate limit instead of 180. It was
rejected for the home panel because `quality` is ignored there, locking frames at ~33 KB
and roughly tripling the data (5.9 MB versus 2.2 MB per 60 s view). Since `app.py` needs
editing regardless, per-frame polling with a raised limit remains preferable — but if
editing the proxy turns out to be undesirable, MJPEG is the fallback that needs no proxy
change beyond the whitelist.

### Server chain verified, and the data estimate corrected

The whole server path is proven working except the token leg:

| Leg | Result |
|-----------------------------|--------|
| `/health` | reports both limits, so the new `app.py` is running |
| proxy `/esp32/live` no token | **401** (was 404) - route registered, token checked |
| Node-RED -> Frigate | **200, real JPEG, 640x360**, in 33-66 ms |
| `height` / `quality` params | both take effect |

**Measured data cost at `height=360`** - this corrects an earlier estimate in this document
of 2.2 MB for 3 fps at quality 30, which was extrapolated from the home panel's smaller
frames and was roughly half the real figure:

| quality | bytes/frame | 3 fps / 60 s | 2 fps / 60 s |
|---------|-------------|--------------|--------------|
| 40 | 29.3 KB | 5.0 MB | 3.4 MB |
| 30 | 24.2 KB | **4.2 MB** | 2.8 MB |
| 25 | 21.4 KB | 3.7 MB | 2.4 MB |
| 20 | 18.3 KB | 3.1 MB | **2.1 MB** |
| 15 | 14.9 KB | 2.6 MB | 1.7 MB |

`height=360` is not freely adjustable: 640x360 is the smallest size that both satisfies
ESP32_JPEG's divisible-by-8 rotation constraint and still fills nearly the full 368 px
screen width after a 90 degree rotation. Going smaller would letterbox the width. **So
quality is the only real lever on data cost**, and both are runtime query parameters, so
tuning needs no server change.

At 368 px wide, JPEG artefacts at quality 20-25 are far less visible than they would be on
a large display. Starting at 25 rather than 30 is worth trying.

Note also that 230,400 pixels is 27% more than the home panel's 181,760, so decode will
cost proportionally more before any rotation overhead is counted.

### Additional considerations for a remote path

- **Latency** rises substantially over the internet. Prefetch exists precisely to hide
  latency, so this should absorb better than it sounds.
- **TLS per frame.** The connection is kept alive so there is no repeated handshake, only
  record encryption, and the ESP32-S3 has hardware AES. Needs measuring, not assuming.
- **`MAX_JPEG_SIZE` is already 128000** here, comfortably above any frame size considered.

## Hard constraint found: only one TLS client fits

Phase 1's first runs failed with `HTTP -1`. The diagnostics settled it:

```text
[VTEST] free heap 51288, largest free block 31732, free PSRAM 8310360
[VTEST] HTTP -1 | heap before 48884, now 48480, largest block 31732
        | TLS: SSL - Memory allocation failed
```

**Total free heap is not the constraint - contiguity is.** mbedTLS needs
contiguous ~16 KB in and out content buffers plus its context. With ~48 KB free
but a largest free block of only ~32 KB, a second `WiFiClientSecure` cannot get
what it needs.

The image fetcher succeeds on the identical host, port and certificate because
its client is a **global constructed at startup**, when the heap is still
unfragmented. Any client created later - global in another module or
function-local, both were tried - fails.

Two wrong diagnoses preceded this, worth recording so they are not repeated:

1. *Total heap exhaustion.* Disproved by the Latest and Back buttons continuing to
   work at the same ~48 KB.
2. *The image fetcher holding its connection open.* Disproved by
   `image_fetcher.cpp:437`, which closes the client after receiving an image.

Neither guess survived contact with evidence; the `lastError()` string did the
work that reasoning could not.

### Consequence for the design

`imageFetcherSecureClient()` now lends the image fetcher's client to the video
module. Safe because the two never fetch at once - each stands down while the
other is active.

**This constrains Phase 2.** The prefetching client cannot open its own TLS
connection; it must drive the shared one. That is a more significant difference
from the home panel than the display or the CPU clock, because the home panel's
prefetch design assumed a dedicated socket it could hold open. Options if sharing
proves awkward:

- Keep the shared client and serialise access, accepting that the still-image
  path cannot fetch during a feed. Already true, via the existing guards.
- Investigate reducing mbedTLS buffer sizes, which needs SDK configuration rather
  than an Arduino sketch change and may not be reachable from this toolchain.
- Reduce fragmentation by moving the 33 KB LVGL draw buffer to PSRAM. Rejected:
  it would slow the blit, which is the thing being optimised.

## Phase 1 results — COMPLETE, measured on hardware

Both orientation variants working, 15 frames each in one burst.

| | A: rotate in decode, ROT_NONE | B: LVGL rotates, ROT_90 |
|--------------|-------------------------------|-------------------------|
| decode | 47.0 ms | 42.7 ms |
| blit | 109.0 ms | 127.6 ms |
| decode+blit | **156.0 ms** | 170.3 ms |
| http | 352.3 ms | 340.7 ms |
| frame | 522.2 ms | 517.0 ms |
| rate | 1.9 fps | 1.9 fps |

**Orientation:** A needs `JPEG_ROTATE_270D`, not `90D`. At 90D the image came out
upside down for this panel. **Confirmed correct with 270D on a second run**, which
also reproduced the timings within noise (decode+blit 155.3 vs 156.0 ms). B renders
correctly as-is.

**Variant A is the decision.** Correct orientation, 15 ms cheaper, and displays at
`ROT_NONE` like the still images already do.

**Keep-alive is worth 550 ms per frame.** Before it, every frame opened a fresh TLS
connection and `http` measured 906 ms. With `setReuse(true)` and a persistent
HTTPClient it dropped to ~350 ms, with frame 1 still paying 879 ms for the one
handshake. This was by far the largest single win in the port.

### Two predictions in this document were wrong

**The blit is not cheaper here.** This document argued the companion's blit would
beat the home panel's 45 ms because it avoids software rotation and uses a small
internal-RAM draw buffer. Measured: **109 ms**, about 2.4x the home panel.
LVGL's rotation accounts for only ~19 ms of it (A 109 vs B 128); the cost is
elsewhere in the AMOLED draw path. The reasoning was plausible and wrong.

**Decode was never the problem.** 47 ms at 240 MHz, against a worry that 80 MHz
would make it prohibitive. Raising the clock was still the right call, but the
bottleneck was always the network.

### Where the time actually goes

```text
http    352 ms   67%   cellular round trip + ~21 KB
blit    109 ms   21%
decode   47 ms    9%
other    14 ms    3%
```

### Variant A is chosen, though the margin is nearly irrelevant

A is cheaper by 14.3 ms and displays at `ROT_NONE`, consistent with how the
still images already work. But note that **once prefetch hides the network, both
variants give the same frame rate**, because `http` (352 ms) exceeds decode+blit
either way. A is preferred for lower CPU use rather than for frame rate.

### The IMU starves during a burst

Sampling frequency collapses from ~50 Hz to **1.85-1.98 Hz** while the burst
runs. The blocking `HTTPClient::GET()` holds the main loop for ~350 ms per frame,
so the IMU barely gets scheduled.

This is a real side effect on a board whose main job is inclinometer and G-meter
work, and it is a second reason to build prefetch: a non-blocking fetch returns
the loop to the IMU instead of parking it. Worth re-measuring the sampling rate
once prefetch lands.

### Projected with prefetch

Frame period becomes roughly `max(http, decode+blit)` plus overhead. With http at
250-350 ms and decode+blit at 156 ms, that is **2.9-4 fps** — so the 3 fps target
is reachable, and the network is the limit rather than the panel. Lowering quality
from 25 to 15 would trim ~60 ms of transfer if more headroom is needed.

## Phase 2 results — COMPLETE, both paths measured

Full 60 second feed from the camera button:

```text
Video: 117 frames in 60.1s (1.9 fps) | http 352 | decode 46 | blit 109 | frame 513 ms
Video: free PSRAM 7849308, free heap 50864
```

Visually good. 117 frames matches the 1.9 fps projected from Phase 1, and IMU
sampling recovers to a full 50.00 Hz the moment the feed ends.

### The TLS session costs ~43 KB while open

```text
during a feed:  heap 50,188
after teardown: heap 93,792     +43 KB released
```

This also settles the memory confusion earlier in this document. The apparent
50 KB drop between Phase 0 and Phase 1 was not the video module's footprint and
not a leak - it was simply an open TLS session left over from an earlier image
fetch. Two wrong theories were chased before the `lastError()` string identified
fragmentation as the real handshake blocker.

Practical consequence: internal heap sits at ~94 KB idle and ~50 KB with a TLS
session open, and the largest contiguous block is ~32 KB. That is why only one
`WiFiClientSecure` fits, and it is worth remembering before adding anything else
that wants a large contiguous allocation.

### Motion chain verified

```text
Initiating HTTPS GET: .../esp32/latest?token=***
Image download complete (34328 bytes, 1131 ms since button press)
LVGL image source updated. Total 1268 ms from button press
... ~10 s ...
Motion still shown, starting live feed
Video: 127 frames in 60.4s (2.1 fps) | http 315 | decode 46 | blit 109 | frame 475 ms
```

The still appears, holds 10 seconds, hands over to the feed, and the panel returns
to the previous screen after 60 seconds.

**The chained feed runs slightly faster** than the button-triggered one - 2.1 fps
with http at 315 ms, against 1.9 fps and 352 ms - because the still fetch has
already warmed the TLS session, so the feed's first frame skips the handshake.

**329 KB more PSRAM is held during a chained feed** (7.52 MB free versus 7.85 MB).
That is image_fetcher's still buffer, 368x448x2 = 329,728 bytes, still allocated
because the chain goes still to video with no cleanup in between. Released on
Screen 2 unload. Harmless against 7.5 MB free, and the same behaviour the home
panel has.

### Accepted trade-off

IMU sampling drops to ~2 Hz for the duration of a feed, because the blocking
fetch holds the main loop for ~350 ms per frame. Accepted deliberately: the
inclinometer and G-meter are not being read while someone is watching the front
door. Prefetch would fix it and roughly halve the frame period, and remains
available as a later improvement on a working baseline.

## Phased plan

Same approach that worked on the home panel: **measure before building.**

### Phase 0 — toolchain — COMPLETE

Changed the CPU clock, added ESP32_JPEG, confirmed the project still builds and the
existing features still work. No video code.

**Verified on hardware at 240 MHz:**

```text
Sampling freq: 50.00 Hz | CPU 240 MHz | heap 98680 | PSRAM 8373372
```

- All screens work: images, G-meter, inclinometer
- Image buttons work end to end (35 KB fetch, decode, display, 2.2 s of a 20 s budget)
- IMU sampling is *better* at 240 MHz — around 47-50 Hz, up from before
- Power draw is higher as expected, board otherwise fine
- No I2C errors

**Memory baseline for the video work:** 8.37 MB PSRAM free, 98 KB internal heap free.
PSRAM is ample; internal heap is tighter than the home panel's ~180 KB, so **video buffers
must go in PSRAM**, which is where they belong anyway.

**Serial visibility.** The boot banner is impractical to catch on this board: USB CDC
attaches after it prints and the battery means the board is usually already running before
the monitor opens. The clock, heap and PSRAM therefore ride along on the periodic
`Sampling freq:` line so they can be read at any time.

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
