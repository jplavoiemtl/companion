# QSPI Video Clock Experiment

## Goal

Determine whether raising the SH8601 AMOLED QSPI clock improves live-feed FPS by
reducing the approximately 110 ms LVGL/display blit, without changing the JPEG,
network, proxy, Node-RED, or Frigate paths.

The production baseline is `height=432`, `quality=15`, prefetch enabled:

| ttfb | xfer | decode | blit | frame | fps |
|------|------|--------|------|-------|-----|
| 177-178 ms | 160 ms | 59 ms | 110-111 ms | 342-343 ms | 2.9 |

## Phase 1: 16 MHz

`DISPLAY_QSPI_HZ` in `companion.ino` is initially set to 16 MHz, twice the
Arduino_GFX default of 8 MHz. This is the only performance variable in the first
test firmware.

### Bench procedure

1. Run one 60-second feed using the current 8 MHz `main` firmware immediately
   before flashing the experiment. Save both `Video:` summary lines.
2. Build and flash this branch at 16 MHz.
3. Confirm the serial log says `Display: QSPI clock 16000000 Hz`.
4. Exercise every screen once before measuring. Look for bad pixels, shifted
   regions, wrong colours, flicker, failed refreshes, or touch/UI regressions.
5. Run two back-to-back 60-second feeds over the same phone hotspot and save both
   summary pairs.
6. Return to Screen 1 after each feed and confirm orientation and normal UI
   refresh are intact.
7. Let the board run for at least five minutes and repeat navigation once to catch
   intermittent display corruption.

Compare back-to-back runs from the same sitting. Cellular performance varies
between sessions, so historical network numbers are not a valid A/B control.

### Expected signature

Only `blit` and total `frame` should fall materially. JPEG size, `xfer`, and
`decode` should remain approximately unchanged. The prefetch `ttfb` metric spans
decode and blit, so it should also fall when blit gets faster; that is expected
and is not a network improvement.

Target for continuing:

- No visible display/UI regression.
- Blit below 85 ms in both runs.
- FPS consistently above the same-session 8 MHz control.

If FPS changes while blit does not, treat that as cellular variance rather than a
successful display-clock result.

### Measured result

Phase 1 passed on hardware. The 60-second feed was visually smoother, the video
and every other screen rendered correctly, and no UI regression was observed.

```text
Video: 193 frames in 60.3s (3.2 fps) | http 308 | decode 74 | blit 69 | frame 313 ms
Video: http = ttfb 157 + xfer 151 ms | frame 17.6 KB | 117 KB/s while transferring
Video: free PSRAM 7646792, free heap 50344
```

Against the documented 8 MHz baseline, blit fell from 110-111 ms to 69 ms
(-37%) and frame time fell from 342-343 ms to 313 ms, raising the displayed rate
from 2.9 to 3.2 fps. Transfer remained close to baseline, so the large blit change
is the decisive evidence that the improvement came from the display clock rather
than cellular variation.

The 69 ms result also matches the bus-cost model almost exactly: approximately
28 ms of fixed LVGL/copy work plus 41 ms to transfer the visible frame at 16 MHz.
Decode rose from 59 ms in the earlier prefetch run to 74 ms in this run; keep
watching that metric, but do not attribute it to the display clock from one sample.

**Decision:** retain 16 MHz as the proven checkpoint and proceed to a separate
20 MHz test with no other performance changes.

### 20 MHz measured result

Two back-to-back 60-second feeds completed with correct rendering on the video
screen and all other screens:

```text
Video: 191 frames in 60.2s (3.2 fps) | http 311 | decode 73 | blit 61 | frame 315 ms
Video: http = ttfb 143 + xfer 167 ms | frame 17.6 KB | 105 KB/s while transferring
Video: free PSRAM 7647008, free heap 50344

Video: 192 frames in 60.3s (3.2 fps) | http 306 | decode 73 | blit 61 | frame 314 ms
Video: http = ttfb 143 + xfer 163 ms | frame 17.5 KB | 107 KB/s while transferring
Video: free PSRAM 7646816, free heap 50316
```

Blit fell another 8 ms, from 69 to 61 ms, exactly matching the display-bus model.
FPS held at 3.2 because transfer rose from 151 ms in the 16 MHz run to 163-167 ms
in these runs. The display clock is no longer the dominant limiter; the next
meaningful experiment must address the per-frame network request gap.

**Decision:** accept 20 MHz as the project setting. Stop the clock sweep here and
preserve HTTP pipelining as a separate experimental branch.

### Subsequent production measurements

Four later 60-second runs used the unchanged accepted `main` firmware. They show
the range produced by cellular throughput and JPEG scene complexity, not four
additional code changes:

```text
Video: 204 frames in 60.2s (3.4 fps) | http 288 | decode 59 | blit 61 | frame 295 ms
Video: http = ttfb 129 + xfer 159 ms | frame 17.7 KB | 111 KB/s while transferring
Video: free PSRAM 7646768, free heap 50360

Video: 243 frames in 60.2s (4.0 fps) | http 239 | decode 56 | blit 61 | frame 248 ms
Video: http = ttfb 128 + xfer 111 ms | frame 14.2 KB | 128 KB/s while transferring
Video: free PSRAM 7647032, free heap 50332

Video: 248 frames in 60.2s (4.1 fps) | http 234 | decode 56 | blit 61 | frame 243 ms
Video: http = ttfb 126 + xfer 108 ms | frame 13.9 KB | 129 KB/s while transferring
Video: free PSRAM 7646804, free heap 50336

Video: 245 frames in 60.2s (4.1 fps) | http 237 | decode 56 | blit 61 | frame 246 ms
Video: http = ttfb 127 + xfer 110 ms | frame 13.9 KB | 126 KB/s while transferring
Video: free PSRAM 7646804, free heap 50336
```

Blit remained exactly 61 ms in all four runs and memory remained stable. The
3.4 fps run transferred a 17.7 KB JPEG at 111 KB/s; the 4.0-4.1 fps runs had
more compressible 13.9-14.2 KB frames and a faster 126-129 KB/s link, cutting
transfer to 108-111 ms. Do not attribute that variable portion to QSPI.

At 4.0-4.1 fps, observed `ttfb` was 126-128 ms against 56 ms decode plus
61 ms blit and 9-11 ms overhead. Prefetch was therefore hiding virtually the
entire server/round-trip wait. The remaining critical cost was JPEG transfer,
which cannot be reduced in the Node-RED flow without changing payload quality
or freshness.

**Production characterization:** the accepted firmware delivers approximately
3.2-4.1 fps depending on cellular conditions and scene compressibility. The
best repeated result is 41% above the original 2.9 fps baseline. Keep 20 MHz
and stop further display, pipeline, reader-task, and server-cache experiments.

## Phase 2: clock sweep - complete at 20 MHz

The remaining frequencies were not pursued because 20 MHz was stable and the
network already hid the additional display gain from the FPS result:

| Clock | Action |
|-------|--------|
| 20 MHz | Accepted project setting; stable 61 ms blit |
| 24 MHz | Optional only if 20 MHz remains clean and blit is still bus-limited |
| 40 MHz | Exploratory ceiling; do not keep without an extended stability run |

Change one clock value per firmware. Do not combine the sweep with JPEG height,
quality, socket draining, or HTTP pipelining changes.

Stop at the first frequency that produces any display corruption or fails to
improve blit meaningfully. The preferred result is the lowest frequency that
captures most of the gain, not the highest clock that happens to boot.

## Rollback

Set `DISPLAY_QSPI_HZ` to `8000000` or use pre-experiment commit `7c8d4ba`. `main`
now intentionally carries the accepted 20 MHz setting. No persistent device state
or server configuration is changed by this experiment.
