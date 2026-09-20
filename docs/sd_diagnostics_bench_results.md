# SD diagnostics bench results

Latest status and next tests: [end-of-day checkpoint, 2026-09-17](sd_diagnostics_checkpoint_2026-09-17.md).
The optional Live pacing experiment was reverted; retain archive 18 until its
remaining tests and safe serial deletion. Earlier sections are chronological evidence.

Project owner and bench tester: **JP**. All owner observations below are JP's.

## 2026-09-15 — Step 0 USB connection check

**Outcome:** the browser connection check passed for the tested board and Chrome
with explicit **DTR=true, RTS=false**, applied after opening at 115200 baud.
The owner reported normal touch and Live operation across disconnects and reconnects.
Proceed to Stage 0 probe validation using these settings.
The VS Code monitor round trip remains unresolved; this is not a pass for every serial client.

### Test context

- Existing firmware, before flashing Stage 0 probes. No SD logger or USB file protocol.
- Board family: Waveshare ESP32-S3-Touch-AMOLED-1.8.
- Browser: Chrome 152.0.0.0; user-agent platform Windows NT 10.0, Win64, x64.
- Source: owner-pasted browser console output and visual observations in this task.
- Times below are the browser's local PC timestamps, not device event timestamps.
- Exact flashed firmware revision, board identifier, battery presence and page URL
  were not recorded. Do not infer these from the current checkout.
- USB remained plugged in during the browser disconnect tests.

### Observations and evidence

| Test | Local time | Result |
|------|------------|--------|
| Driver defaults, no setSignals call | 13:32–13:35 and 14:40–14:41 | Owner saw a board reset when clicking Disconnect. Boot output arrived on reconnect. |
| VS Code monitor close | Initial report | Owner saw frozen UI and unresponsive buttons. Unplugging and reconnecting USB restored operation. Cause unresolved. |
| First explicit DTR=true, RTS=false trial | 14:43:17 connect; 14:43:28 disconnect | Owner reported no reset and normal board operation after disconnect. |
| Repeated explicit-signal cycles | 14:45:15–14:46:35 | Four successful connections and disconnects; owner reported normal operation throughout. Three status readings confirm continuous uptime across the measured reconnects. |
| Explicit-signal disconnect during Live | 14:48:28–14:49:32 | Live continued normally and completed its full cycle. No reset or visible pause reported. |

A port chooser cancellation at 14:45:53 produced “No port selected by the user.”
The next selection succeeded; this was not a board or transfer failure.

Selected uptime evidence from the repeated-cycle test:

```text
14:46:00.214 RX uptime=248372 ms
14:46:18.326 RX uptime=266485 ms
14:46:32.731 RX uptime=280890 ms
```

Uptime increased by 18.113 and 14.405 seconds, matching the elapsed PC time
between readings to within milliseconds. No reboot occurred between those readings.

Selected Live evidence:

```text
14:48:32.519 RX Live button clicked -> starting live feed
14:48:40.664 EVENT Disconnect clicked
14:48:40.670 EVENT Disconnected
14:48:55.997 EVENT Explicit signals applied. After open: DTR=true, RTS=false.
14:49:03.104 RX uptime=55199 ms
14:49:32.808 RX Video: 194 frames in 60.3s (3.2 fps) | http 302 | decode 59 | blit 61 | frame 311 ms
14:49:32.808 RX Video: http = ttfb 129 + xfer 173 ms | frame 17.5 KB | 102 KB/s while transferring
14:49:32.808 RX Video: free PSRAM 7647016, free heap 50560
14:49:32.810 RX Video: returning to previous screen
14:49:32.810 RX [ScreenMem] Returned to screen 1; no preference save needed
```

The owner reported uninterrupted video throughout the test. These are existing
firmware summary values, not Stage 0 minimum-memory or maximum-frame-gap measurements.
The earlier driver-default session also completed Latest in 1381 ms at 14:41:47.632.

### Interpretation and limits

- Explicit control-signal settings avoided the observed browser disconnect reset
  in these trials. This supports a control-line explanation without proving the
  exact Windows driver or chip reset sequence.
- Boot text received when opening a port may have been buffered. In the first
  session, status showed 70.204 seconds of uptime only about 31 seconds after
  opening the page. Boot text alone cannot date a reset.
- A separate reboot occurred between the repeated-cycle run and the Live run:
  the Live run's uptime was 55.199 seconds at 14:49:03.104. Its trigger was not
  recorded. It predates the Live run's initial connection and is not evidence
  of a reset during that disconnect test.
- Early page logs had a timestamp bug: an empty fragment after a newline could
  timestamp the next received message too early. The page fix preserves the
  arrival time of a real partial line but ignores an empty trailing fragment.
  The 13:32 status RX timestamp is affected; do not use it for precise latency.
- Exact DTR and RTS levels under driver-default mode are unknown.
- Secure-context and navigator.serial indicator values were not pasted.
  Successful use verifies availability in this session, not all disk or browser contexts.
- VS Code close behavior remains an open issue. Use the browser with the tested
  explicit settings for measurement; release its port before owner builds and flashes.

### Stage 0 handoff

At handoff, compilation and flashing were in progress. The first running Stage 0
probe result subsequently arrived at 15:03:46; see below.

First collect a normal-operation summary after about 70 seconds without media.
Then validate MQTT, Latest and Live probe windows one test at a time.
Record build identity, board and power source with the next measurements.

Keep these Step 0 results as connection evidence. Before the Stage 1 comparison,
remeasure the probe-only firmware immediately before logging-enabled firmware
in the same sitting, as required by CLAUDE.md. Today's Live figures are not a
substitute for that paired baseline.


## 2026-09-15 — Stage 0 first normal-operation measurement

**Status:** probe firmware is running on the board. The first normal window has
complete nominal periodic coverage. Latest, Live and MQTT connect coverage remain
pending; this does not pass the Stage 1 memory or performance gate.

Chrome 152 connected at 15:02:59.850 with explicit DTR=true, RTS=false.
Startup output was partial and may have been buffered. It showed CPU 240 MHz,
heap 93748 and PSRAM 8372552. No startup MQTT probe was captured in this excerpt.
The exact build identity, board identifier and power source remain unrecorded.

Owner-supplied measurement, with escaped underscores normalized:

```text
2026-09-15 15:03:46.270 RX [PROBE] window=normal run=1 ms=60000 heap_min_boot=88192 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10918 scan_max_us=1056 timer=on imu_n=2946 imu_min_hz=30.28 imu_avg_hz=49.19
```

| Measurement | Result |
|-------------|--------|
| Window duration | 60000 ms |
| Internal heap low-water mark since boot | 88192 bytes |
| Lowest sampled largest free internal block | 31732 bytes |
| Requested sampler interval and count | 10 ms; 6000 periodic samples |
| Longest sample gap | 10918 us (10.918 ms) |
| Slowest heap scan | 1056 us (1.056 ms) |
| Sampler state | on |
| Eligible IMU observations | 2946 |
| Minimum observed IMU rate | 30.28 Hz |
| Average observed IMU rate | 49.19 Hz |

The count matches the nominal 100 samples per second for 60 seconds, and the
longest gap is close to the requested interval. This establishes normal-window
coverage; short TLS allocation windows still need their own counts and gaps.
The largest-block result is above the proposed 20480-byte threshold in this
normal window, but does not establish the minimum during TLS connections.
The scan maximum records probe cost; it is not an average CPU overhead measurement.

The IMU minimum is the lowest individual eligible sampling_frequency observation,
not a sustained 30.28 Hz rate. Its arithmetic mean is 49.19 Hz. Retain both for
same-session comparison when logging is added; no IMU behavior change is indicated
by this single measurement.

**Next test:** request Latest once with Wi-Fi and real MQTT connected. Capture
the image_https probe, full Latest timing and any error. Return to the normal
screen afterward. Live and MQTT tests follow separately.


## 2026-09-15 — Stage 0 Latest HTTPS measurement

**Outcome:** Latest succeeded in 1291 ms. The HTTPS window collected 76 periodic
samples over 757 ms, with a maximum gap of 10.331 ms. The lowest sampled largest
internal block was 30708 bytes, above the proposed 20480-byte threshold.
This validates the captured HTTPS window; Live and MQTT windows remain pending.

Owner-supplied output, with escaped underscores normalized:

```text
2026-09-15 15:05:42.781 RX [PROBE] window=image_https run=1 ms=757 heap_min_boot=38644 largest_min=30708 interval_ms=10 samples=76 gap_max_us=10331 scan_max_us=186 timer=on
2026-09-15 15:05:42.781 RX Response received in 758 ms, Content-Length: 33012
2026-09-15 15:05:42.781 RX Starting to receive image data...
2026-09-15 15:05:43.107 RX Image download complete (33012 bytes, 1156 ms since button press). Starting decode...
2026-09-15 15:05:43.242 RX JPEG decoded successfully into PSRAM.
2026-09-15 15:05:43.242 RX LVGL image source updated. Total 1291 ms from button press (budget 20000 ms).
2026-09-15 15:05:46.937 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-15 15:05:46.937 RX [ScreenMem] Returned to screen 1; no preference save needed
```

| Measurement | Result |
|-------------|--------|
| HTTPS window duration | 757 ms |
| Internal heap low-water mark since boot | 38644 bytes |
| Lowest sampled largest free internal block | 30708 bytes |
| Requested sampler interval and count | 10 ms; 76 periodic samples |
| Longest sample gap | 10331 us |
| Slowest heap scan | 186 us |
| Sampler state | on |
| HTTP response time | 758 ms |
| Image size | 33012 bytes |
| Download completion from button press | 1156 ms |
| Total Latest time | 1291 ms (20000 ms budget) |

The HTTPS window includes GET through response headers, not only the TLS handshake.
The internal heap low-water mark fell from 88192 to 38644 bytes since boot.
This records a new historical minimum, not retained allocation or evidence of a leak.
No allocation, TLS or decode failure appears in the supplied excerpt.
Largest-block readings remain sampled minima. No logging-on performance comparison
has been made, and this result does not complete the Stage 1 gate.

**Next test:** run one full Live cycle with Wi-Fi and real MQTT connected.
Leave the browser connected and do not use off or on during this measurement.
Capture live_tls if a new TLS connection occurs, the live summary, and Video fps,
first_frame, max_gap and memory lines. Record visible pauses or errors.
If the shared connection is reused, live_tls may be absent; test that separately
rather than interpreting a missing TLS window as a successful connect measurement.


## 2026-09-15 — Stage 0 full Live cycle

**Outcome:** owner reported normal Live video throughout the complete cycle.
The full Live window and two actual secure-client connect calls were sampled.
Their lowest observed largest blocks exceeded the proposed 20480-byte threshold.
MQTT connect probe validation remains pending.

| Window | Duration ms | heap_min_boot bytes | largest_min bytes | Samples at 10 ms | gap_max_us | scan_max_us |
|--------|-------------|---------------------|-------------------|------------------|------------|-------------|
| normal run 7, ended at Live start | 11239 | 38644 | 31732 | 1124 | 10647 | 279 |
| live_tls run 1 | 867 | 38644 | 30708 | 87 | 10231 | 252 |
| live_tls run 2 | 623 | 38084 | 30708 | 62 | 10130 | 287 |
| live run 1 | 60327 | 38084 | 29684 | 6032 | 11347 | 981 |

All windows reported timer=on. The preceding normal window recorded 551 eligible
IMU observations, minimum 29.40 Hz and average 49.16 Hz. This normal window ended
at Live start; these IMU figures do not measure Live operation.

Owner-supplied probe and timing evidence, with escaped underscores normalized:

```text
2026-09-15 15:08:54.489 RX Live button clicked -> starting live feed
2026-09-15 15:08:54.490 RX [PROBE] window=normal run=7 ms=11239 heap_min_boot=38644 largest_min=31732 interval_ms=10 samples=1124 gap_max_us=10647 scan_max_us=279 timer=on imu_n=551 imu_min_hz=29.40 imu_avg_hz=49.16
2026-09-15 15:08:54.561 RX Video: endpoint parsed, port 9835, path /esp32/live
2026-09-15 15:08:55.431 RX [PROBE] window=live_tls run=1 ms=867 heap_min_boot=38644 largest_min=30708 interval_ms=10 samples=87 gap_max_us=10231 scan_max_us=252 timer=on
2026-09-15 15:08:55.715 RX Video: frame 432x768, panel 368x448 -> gap x=-64 y=-320, pan x=-15 y=32
2026-09-15 15:09:15.349 RX TX motion MQTT: Moving (periodic)
2026-09-15 15:09:15.382 RX Movement Stopped.
2026-09-15 15:09:28.085 RX [PROBE] window=live_tls run=2 ms=623 heap_min_boot=38084 largest_min=30708 interval_ms=10 samples=62 gap_max_us=10130 scan_max_us=287 timer=on
2026-09-15 15:09:54.814 RX Video: 180 frames in 60.3s (3.0 fps) | http 326 | decode 74 | blit 61 | frame 335 ms | first_frame 1361 | max_gap 943 ms
2026-09-15 15:09:54.814 RX Video: http = ttfb 145 + xfer 180 ms | frame 18.0 KB | 100 KB/s while transferring
2026-09-15 15:09:54.814 RX Video: free PSRAM 7647048, free heap 49824
2026-09-15 15:09:54.816 RX [PROBE] window=live run=1 ms=60327 heap_min_boot=38084 largest_min=29684 interval_ms=10 samples=6032 gap_max_us=11347 scan_max_us=981 timer=on
2026-09-15 15:09:54.816 RX Video: returning to previous screen
2026-09-15 15:09:54.816 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-15 15:09:54.816 RX [ScreenMem] Returned to screen 1; no preference save needed
```

First-frame time was 1361 ms; the longest interval between displayed frames was
943 ms. The owner noticed no interruption. A second secure-client connection
occurred during the feed, but this excerpt does not establish why it was needed
or locate the maximum frame gap relative to that connection. No fetch failure
or allocation error appears in the supplied output.

The largest sampled internal block stayed at least 29684 bytes across Live.
The heap low-water mark since boot reached 38084 bytes. These are baseline
observations, not a leak diagnosis or a completed Stage 1 acceptance gate.

The prior Step 0 run reported 3.2 fps versus 3.0 here. Its frame content and
transfer metrics differed (17.5 versus 18.0 KB; 102 versus 100 KB/s; decode 59
versus 74 ms). These runs do not isolate probe overhead. Preserve both records;
do not attribute that difference to the probes or apply the Stage 1 logging
tolerance to this unpaired comparison.

**Next test:** on the normal screen, keep Wi-Fi connected and send off. Let one
test-endpoint MQTT attempt finish, then send on and wait for the green remote
message. Capture both mqtt_connect windows and TEST attempt timings. Do not start
media during this test. A brief UI freeze during the existing blocking retry is
expected; confirm recovery afterward.


## 2026-09-15 — Stage 0 MQTT outage and recovery

**Outcome:** the sampler continued through a blocking test-endpoint failure.
The real broker then reconnected successfully. This completes the requested
Stage 0 measurement-window coverage: normal, Latest HTTPS, Live TLS, full Live,
and failed and successful MQTT connects. Stage 1 has not started.

Owner-supplied key evidence, with escaped underscores normalized:

```text
2026-09-15 15:12:54.827 RX [PROBE] window=normal run=10 ms=60004 heap_min_boot=38084 largest_min=31732 interval_ms=10 samples=6001 gap_max_us=10599 scan_max_us=502 timer=on imu_n=2946 imu_min_hz=36.88 imu_avg_hz=49.20
2026-09-15 15:12:58.630 TX off [CRLF]
2026-09-15 15:12:58.637 RX [TEST] OFF: MQTT -> 192.0.2.1:9735 until you send on; WiFi/HTTPS unchanged.
2026-09-15 15:12:58.637 RX [TEST] First attempt in 5000 ms; later attempts use the existing 15-second retry interval.
2026-09-15 15:12:58.637 RX [TEST] No automatic restore. Commands wait while a connection attempt blocks.
2026-09-15 15:12:59.396 RX [NET] WiFi=CONNECTED | MQTT=DISCONNECTED
2026-09-15 15:13:03.738 RX [TEST] MQTT attempt 1 BEGIN -> TEST endpoint | uptime=625847 ms
2026-09-15 15:13:03.738 RX [PROBE] window=normal run=11 ms=8911 heap_min_boot=38084 largest_min=31732 interval_ms=10 samples=891 gap_max_us=10807 scan_max_us=312 timer=on imu_n=433 imu_min_hz=38.28 imu_avg_hz=49.34
2026-09-15 15:13:08.741 RX [PROBE] window=mqtt_connect run=2 ms=5002 heap_min_boot=38084 largest_min=34804 interval_ms=10 samples=500 gap_max_us=10046 scan_max_us=110 timer=on
2026-09-15 15:13:08.742 RX [TEST] MQTT attempt 1 END -> TEST endpoint | FAILED | elapsed=5004 ms | state=-2
2026-09-15 15:13:20.030 TX on [CRLF]
2026-09-15 15:13:20.032 RX [TEST] ON: restoring real broker (serial on), 21396 ms after off.
2026-09-15 15:13:20.135 RX [TEST] MQTT attempt 2 BEGIN -> REAL broker | uptime=642244 ms
2026-09-15 15:13:20.135 RX [PROBE] window=normal run=12 ms=11391 heap_min_boot=38084 largest_min=34804 interval_ms=10 samples=1139 gap_max_us=10234 scan_max_us=351 timer=on imu_n=555 imu_min_hz=37.27 imu_avg_hz=49.23
2026-09-15 15:13:20.647 RX [PROBE] window=mqtt_connect run=3 ms=511 heap_min_boot=38084 largest_min=31732 interval_ms=10 samples=51 gap_max_us=10226 scan_max_us=275 timer=on
2026-09-15 15:13:20.647 RX [TEST] MQTT attempt 2 END -> REAL broker | CONNECTED | elapsed=513 ms | state=0
2026-09-15 15:13:20.653 RX [TEST] Real broker connected. Ready for another off.
2026-09-15 15:13:20.773 RX [NET] WiFi=CONNECTED | MQTT=REMOTE CONNECTED
```

A calibration publish was also logged at 15:13:20.653; its payload is omitted here.

| MQTT window | Probe duration | TEST elapsed | Largest block minimum | Samples | Maximum gap | Maximum scan |
|-------------|----------------|--------------|-----------------------|---------|-------------|--------------|
| Failed test endpoint | 5002 ms | 5004 ms | 34804 bytes | 500 | 10046 us | 110 us |
| Successful real broker | 511 ms | 513 ms | 31732 bytes | 51 | 10226 us | 275 us |

Both windows reported interval_ms=10, timer=on and heap_min_boot=38084.
The 2 ms differences between probe and TEST elapsed durations reflect their
different measurement boundaries and surrounding instrumentation; they are not
additional connection attempts. The failed endpoint produced the intended state=-2.
The successful connection produced state=0, followed by REMOTE CONNECTED.

The counts and approximately 10 ms maximum gaps show that periodic sampling
continued during the five-second blocking call. No new since-boot heap minimum
was reached in these attempts. Both sampled largest-block minima exceeded 20480 bytes.
No separate visual observation accompanied this MQTT excerpt; successful network
recovery is established by the log, but no new claim about touch response is made.

The additional full normal window averaged 49.20 Hz (minimum observation 36.88 Hz).
Its 6001 samples over 60004 ms are consistent with the nominal 10 ms cadence.
The shorter normal windows ended before the connect calls; their IMU rates do not
include the blocked calls.

### Stage 0 checkpoint

The initial probe validation sequence is complete from the supplied logs.
No new allocation, media-fetch or decode failure appeared; the intentional MQTT
failure recovered. The owner separately reported smooth full-cycle Live operation.
The smallest sampled largest internal block across the captured windows was
29684 bytes, and the lowest reported since-boot internal heap value was 38084 bytes.

Retain this probe-only version unchanged for the Stage 1 comparison. Before
accepting logging performance, run the probe-only baseline and logging-enabled
build back-to-back in the same sitting. Stage 0 does not exercise the SD queue,
writer stack, card errors, rotation, clocks, breadcrumbs or shutdown close.

No further fault test is proposed before owner review of this checkpoint.
Next proposed implementation is Stage 1 basics only, followed by its bench gate;
the current request stops at Step 0 and Stage 0. No Stage 1 implementation,
firmware edit or commit was made while recording these results.


### Checkpoint review and continuation

JP requested that the tested Step 0 and Stage 0 work and results be committed
and pushed on 2026-09-15. This checkpoint preserves the probe-only baseline.

Review conclusion: the captured measurements support proceeding to Stage 1 basics.
Periodic sampling covered the blocking MQTT timeout, successful MQTT connection,
HTTPS request, both Live TLS connects and full Live operation. Largest sampled
internal blocks exceeded 20480 bytes in all captured windows. JP reported normal
Latest and Live operation. The normal IMU averages were approximately 49.2 Hz.

Open points carried forward:

- Use explicit DTR=true, RTS=false in the tested browser. The VS Code monitor
  disconnect freeze remains unresolved and is not silently accepted as fixed.
- Largest-block minima are sampled. Keep the same probes, interval and coverage
  reporting in Stage 1; the current results do not establish SD writer costs.
- Confirm the Stage 1 proposed acceptance limits with JP before paired measurements:
  no new allocation or TLS failures, largest internal block at least 20480 bytes
  during TLS connects, Live fps within about 5% of the same-session baseline,
  and zero queue drops in normal use. Record writer stack margin as well.
- Record board identifier and power source with the next bench run.
- Remeasure this committed probe-only firmware immediately before the logging
  build in the same sitting. Historical measurements remain useful context.

The next implementation requires JP's instruction to start Stage 1. Implement
only the basics listed in the main plan: SD mounting with format disabled, queue
and writer, append headers, rotation and pruning, boot identity, clock states,
reset records and RTC breadcrumbs, bounded shutdown close, health and status.
Include its planned test hooks. Stop for the Stage 1 bench gate before USB file
retrieval or network and media event hooks. JP builds and flashes each version.


## 2026-09-15 — JP accepts the checkpoint and authorizes Stage 1

JP stated: "I accept. Let's continue." The Step 0 and Stage 0 checkpoint was
committed and pushed as f406063 on sd-diagnostics before Stage 1 work began.

Stage 1 basics have now been prepared, with the original probes unchanged.
No compilation or flashing was performed by the assistant and no new firmware
measurements are available. The Stage 1 gate remains pending.
See [the implementation handoff](../src/diagnostics/STAGE1.md) for choices,
default-off fault hooks and the first bench check.

The first check is a normal build with a prepared FAT32 card, fault hooks off,
startup output and log status. Preserve or refresh the same-sitting probe-only
baseline before flashing for the performance comparison. JP remains the builder,
flasher and bench tester. Stage 1B does not start until JP accepts Stage 1 results.


## 2026-09-15 — Stage 1 first startup and normal-operation check

**Outcome:** JP's flashed Stage 1 build reports ready, synced, hooks off and no
logger errors. Current grew by 554 bytes and writes increased from 7 to 8
between status requests. This is consistent with one minute health record.
The copied file contents and power-loss durability have not yet been inspected.
This passes the initial readiness check, not the complete Stage 1 gate.

Chrome 152 connected at 16:51:00.266 with explicit DTR=true, RTS=false.
The detected card capacity is 15931539456 bytes (about 15.93 decimal GB,
consistent with a nominal 16 GB card). Board identifier and battery presence
remain unreported. The exact build revision is the uncommitted Stage 1 work;
its compiled timestamp was not in the supplied excerpt.

Selected owner-supplied evidence, with escaped underscores normalized:

```text
2026-09-15 16:51:15.857 TX log status [CRLF]
2026-09-15 16:51:15.862 RX [LOG] state=ready boot=2 session=boot-2 up_ms=39419 clock=synced setup=1 hooks=0 file_bytes=2098 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15923052544 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-15 16:51:15.862 RX [LOG] measured=1 stack_min=1992 internal_min=78984 internal_largest=31732 dma_min=71488 dma_largest=31732 writes=7 slow=0 write_max_us=2236 flush_max_us=5020 sd_max_us=55202 rotations=0 pruned=0 oversized=0
2026-09-15 16:51:44.828 RX [PROBE] window=normal run=1 ms=60002 heap_min_boot=78984 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10986 scan_max_us=1116 timer=on imu_n=2948 imu_min_hz=27.78 imu_avg_hz=49.22
2026-09-15 16:52:17.571 TX log status [CRLF]
2026-09-15 16:52:17.575 RX [LOG] state=ready boot=2 session=boot-2 up_ms=101134 clock=synced setup=1 hooks=0 file_bytes=2652 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15923052544 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-15 16:52:17.576 RX [LOG] measured=1 stack_min=1992 internal_min=78984 internal_largest=31732 dma_min=71488 dma_largest=31732 writes=8 slow=0 write_max_us=2236 flush_max_us=5020 sd_max_us=55202 rotations=0 pruned=0 oversized=0
```

| Measurement | Result |
|-------------|--------|
| Logger | ready; clock synced; setup complete; fault hooks off |
| Current size | 2098 to 2652 bytes, +554 |
| File generation and archives | generation 1; zero archives, rotations or pruning |
| Queue | empty at both readings; high-water 1 of 16 |
| Lost or shortened records | drops 0; suppressed 0; truncated 0 |
| Writer stack minimum unused margin | 1992 bytes out of 6144 allocated |
| Internal heap minimum since boot | 78984 bytes |
| Writer boundary minimum largest internal block | 31732 bytes |
| Internal DMA heap minimum and largest block | 71488 and 31732 bytes |
| Maximum write and flush time | 2236 us (2.236 ms); 5020 us (5.020 ms) |
| Maximum instrumented SD operation time | 55202 us (55.202 ms) |
| Slow write or flush count | 0, using the 100 ms threshold |
| Normal periodic sampling | 6000 samples over 60002 ms; interval 10 ms |
| Maximum normal sample gap and scan time | 10986 us; 1116 us |
| Normal IMU | 2948 observations; minimum 27.78 Hz; average 49.22 Hz |

The 1992-byte stack margin covers paths exercised so far, not future rotation,
recovery or shutdown peaks. No conclusion about the full writer stack gate yet.

Compared with the first probe-only normal window, the reported since-boot internal
heap minimum is 9208 bytes lower (88192 to 78984). The normal sampled largest block
remains 31732 bytes. This is consistent with the new logger's memory cost, not
evidence of a leak. The previous 38084-byte low after media is a different workload
and must not be compared as though it were an idle reading.

The normal IMU average of 49.22 Hz is close to the earlier 49.19 Hz baseline.
The 27.78 Hz minimum is one observed interval, not a sustained sampling rate.
Keep same-sitting pairing and equivalent workloads for the final performance gate;
TLS and full Live measurements with logging are still required.

The boot counter value 2 alone does not establish repeated-boot append correctness.
That storage test still requires before/after file evidence. Free bytes remaining
unchanged while the file grows is not a demonstrated error; byte-level file growth
and filesystem allocation accounting differ.

**Next test:** with logging ready, Wi-Fi and MQTT connected, request Latest once.
Return to the normal screen and send log status. Capture image_https, the complete
image timing and both logger status lines. This checks HTTPS memory and image
operation with the logger present before the full Live test.


## 2026-09-15 — Stage 1 Latest: memory gate failure

JP confirmed this board has a **battery and a 16 GB card**.

**Outcome:** Latest displayed successfully in 1372 ms, but the HTTPS largest-block
measurement was **14836 bytes**. This is **5644 bytes below** the agreed proposed
20480-byte floor. Sampling coverage was adequate for this observed failure:
86 readings over 853 ms, maximum gap 10079 us. Do not accept the memory gate
or advance to the planned full Live test until this reduction is investigated.

Owner-supplied key evidence, with escaped underscores normalized:

```text
2026-09-15 16:55:08.933 RX Latest button clicked
2026-09-15 16:55:09.007 RX [PROBE] window=normal run=5 ms=24172 heap_min_boot=78984 largest_min=31732 interval_ms=10 samples=2417 gap_max_us=10794 scan_max_us=335 timer=on imu_n=1184 imu_min_hz=32.22 imu_avg_hz=49.25
2026-09-15 16:55:09.861 RX [PROBE] window=image_https run=1 ms=853 heap_min_boot=30756 largest_min=14836 interval_ms=10 samples=86 gap_max_us=10079 scan_max_us=164 timer=on
2026-09-15 16:55:09.861 RX Response received in 855 ms, Content-Length: 33689
2026-09-15 16:55:10.170 RX Image download complete (33689 bytes, 1236 ms since button press). Starting decode...
2026-09-15 16:55:10.306 RX JPEG decoded successfully into PSRAM.
2026-09-15 16:55:10.306 RX LVGL image source updated. Total 1372 ms from button press (budget 20000 ms).
2026-09-15 16:55:13.660 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-15 16:55:16.506 RX [LOG] state=ready boot=2 session=boot-2 up_ms=280069 clock=synced setup=1 hooks=0 file_bytes=4322 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15923052544 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-15 16:55:16.508 RX [LOG] measured=1 stack_min=1992 internal_min=30756 internal_largest=14836 dma_min=23260 dma_largest=14836 writes=11 slow=0 write_max_us=2236 flush_max_us=5020 sd_max_us=55202 rotations=0 pruned=0 oversized=0
```

| Metric | Probe-only Latest, 15:05 | Stage 1 Latest, 16:55 |
|--------|-------------------------|----------------------|
| HTTPS duration | 757 ms | 853 ms |
| Sampled largest internal block minimum | 30708 bytes | 14836 bytes |
| Internal heap minimum since boot | 38644 bytes | 30756 bytes |
| Periodic samples | 76 | 86 |
| Maximum sample gap | 10331 us | 10079 us |
| Total Latest time | 1291 ms | 1372 ms |
| Image bytes | 33012 | 33689 |

The largest-block minimum decreased by 15872 bytes, while total internal
low-water memory decreased by 7888 bytes. Different heap allocation layout
may explain the larger loss of contiguous space. This is not yet a leak diagnosis.
The absolute 20480-byte gate fails independently of any uncertainty in the
earlier comparison or changed image content. The roughly 5% limit applies to
Live fps; it is not a separate pass/fail limit for Latest time.

Logger writes continued, with no drops, truncation, slow writes or storage errors.
The status values are historical minima, not a fresh measurement of free memory
at the moment status was requested.

Source review confirms the event queue and 1024-byte formatter use explicit PSRAM,
while the writer has a 6144-byte internal stack. SDMMC, VFS and FATFS also have
their own allocations. The exact allocation responsible for the 14836-byte
largest-block result is unverified. Do not attribute it to the possible temporary
DMA bounce buffer without further evidence. Do not shrink the stack blindly:
1992 bytes remain on the paths exercised, but recovery and rotation peaks are
not yet measured.

**Next diagnostic test:** run the same Stage 1 firmware without the card.
Fully power down the battery-equipped board before removing the card; unplugging
USB alone does not turn it off. Boot, capture log status (logging should be
disabled after the mount failure), request Latest once, then capture log status
again. This removes successful SD mounting and the continuing writer task from
the run, while retaining the new code, queue and clock setup. Compare image_https
coverage and largest_min. It does not isolate an individual allocation, and
historical startup minima in logger status must not be mistaken for the new HTTPS
probe measurement. No firmware change or threshold change was made for this diagnosis.


## 2026-09-15 — Stage 1 no-card comparison

**Outcome:** no-card startup disabled logging as designed, and Latest succeeded.
The HTTPS largest-block minimum recovered to **28660 bytes**, above the 20480-byte
floor. The mounted-card result remains a failed gate; no-card success does not
accept logging-enabled operation.

Same Stage 1 firmware, battery-equipped board, card removed while powered off.
This was boot 3. Wi-Fi and MQTT operation continued with the logger disabled.

Owner-supplied evidence, with escaped underscores normalized:

```text
2026-09-15 17:01:25.273 RX [LOG] state=disabled boot=3 session=boot-3 up_ms=28785 clock=synced setup=1 hooks=0 file_bytes=0 generation=0 newest=0 archives=0 card_bytes=0 free_bytes=0 queue=0/16 high=0 drops=0 suppressed=0 truncated=0 error=mount_failed_or_no_card errno=5
2026-09-15 17:01:25.274 RX [LOG] measured=1 stack_min=2264 internal_min=196316 internal_largest=131060 dma_min=188820 dma_largest=131060 writes=0 slow=0 write_max_us=0 flush_max_us=0 sd_max_us=28504 rotations=0 pruned=0 oversized=0
2026-09-15 17:01:34.749 RX Latest button clicked
2026-09-15 17:01:34.822 RX [PROBE] window=normal run=1 ms=29745 heap_min_boot=86640 largest_min=31732 interval_ms=10 samples=2974 gap_max_us=10635 scan_max_us=408 timer=on imu_n=1457 imu_min_hz=36.99 imu_avg_hz=49.20
2026-09-15 17:01:35.544 RX [PROBE] window=image_https run=1 ms=721 heap_min_boot=36584 largest_min=28660 interval_ms=10 samples=72 gap_max_us=10294 scan_max_us=140 timer=on
2026-09-15 17:01:35.544 RX Response received in 722 ms, Content-Length: 33689
2026-09-15 17:01:35.895 RX Image download complete (33689 bytes, 1145 ms since button press). Starting decode...
2026-09-15 17:01:36.030 RX JPEG decoded successfully into PSRAM.
2026-09-15 17:01:36.031 RX LVGL image source updated. Total 1281 ms from button press (budget 20000 ms).
2026-09-15 17:01:39.406 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-15 17:01:43.672 RX [LOG] state=disabled boot=3 session=boot-3 up_ms=47185 clock=synced setup=1 hooks=0 file_bytes=0 generation=0 newest=0 archives=0 card_bytes=0 free_bytes=0 queue=0/16 high=0 drops=0 suppressed=0 truncated=0 error=mount_failed_or_no_card errno=5
2026-09-15 17:01:43.673 RX [LOG] measured=1 stack_min=2264 internal_min=196316 internal_largest=131060 dma_min=188820 dma_largest=131060 writes=0 slow=0 write_max_us=0 flush_max_us=0 sd_max_us=28504 rotations=0 pruned=0 oversized=0
```

| HTTPS measurement | Card installed, boot 2 | No card, boot 3 |
|-------------------|------------------------|----------------|
| Image bytes | 33689 | 33689 |
| Total Latest time | 1372 ms | 1281 ms |
| Probe duration | 853 ms | 721 ms |
| Largest sampled internal block | 14836 bytes | 28660 bytes |
| Internal heap minimum since boot | 30756 bytes | 36584 bytes |
| Samples at 10 ms | 86 | 72 |
| Longest sample gap | 10079 us | 10294 us |
| Slowest heap scan | 164 us | 140 us |

Removing the mounted SD subsystem and continuing writer increased the HTTPS
largest-block minimum by 13824 bytes and the total internal low-water mark by
5828 bytes. This implicates the combined mounted filesystem and live writer
footprint or allocation layout. It does not identify a specific allocation,
prove a leak, or verify the hypothesized temporary SD DMA copy.

The writer exits after its mount failure and stops updating its status memory
snapshot. Thus internal_min=196316 and internal_largest=131060 are frozen early
startup observations. They are not the current heap or the HTTPS minima.
Use the image_https probe for this comparison. The task's stack is reclaimed
after exit; the PSRAM queue and clock support remain.

### First corrective experiment — writer start order

Moved only the diagnosticsStart() call in companion.ino. It now runs after
display, UI, media-module, IMU and battery initialization, immediately before
initWiFi(). Previously it ran immediately after initPMIC().
The event queue, boot counter and RTC capture remain early. The SD writer still
starts without waiting for network connection, including on battery-only boots
that may later shut down after Wi-Fi failure.

This tests whether allowing long-lived hardware allocations to settle first
improves contiguous internal memory. It does not reduce total logger memory.
The 6144-byte internal writer stack, sampling probes, SD limits, network
timeouts and fault-hook defaults are unchanged. Success is unverified.

**Next test:** fully power down, reinstall the same 16 GB card, and have JP
build and flash this revised version. Keep fault hooks off. Request Latest
once after Wi-Fi and MQTT reconnect, then send log status. Capture the HTTPS
probe, image result and both logger status lines. Require logging ready and
largest_min at least 20480 bytes before considering the experiment successful.
No compilation, flashing or commit was performed by the assistant.


## 2026-09-15 — Writer start-order experiment: no memory improvement

**Outcome:** JP tested the card-installed startup-order version after checkpoint
fe3b5da. Latest succeeded, but largest_min was again **14836 bytes**, identical
to the earlier failing run. The 20480-byte memory gate still fails.
Moving writer startup after hardware initialization did not solve this case.

Battery-equipped board and same 16 GB card. Logger ready, clock synced, hooks off.
This is boot 6; intermediate boot numbers were not supplied and are not interpreted.

Owner-supplied evidence, with escaped underscores normalized:

```text
2026-09-15 17:10:14.054 RX [LOG] state=ready boot=6 session=boot-6 up_ms=23170 clock=synced setup=1 hooks=0 file_bytes=11830 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15923052544 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-15 17:10:14.055 RX [LOG] measured=1 stack_min=1988 internal_min=78988 internal_largest=31732 dma_min=71492 dma_largest=31732 writes=7 slow=0 write_max_us=2317 flush_max_us=4719 sd_max_us=98439 rotations=0 pruned=0 oversized=0
2026-09-15 17:10:28.594 RX Latest button clicked
2026-09-15 17:10:28.667 RX [PROBE] window=normal run=1 ms=29306 heap_min_boot=78988 largest_min=31732 interval_ms=10 samples=2931 gap_max_us=10666 scan_max_us=432 timer=on imu_n=1434 imu_min_hz=34.47 imu_avg_hz=49.16
2026-09-15 17:10:29.430 RX [PROBE] window=image_https run=1 ms=762 heap_min_boot=30736 largest_min=14836 interval_ms=10 samples=76 gap_max_us=10081 scan_max_us=149 timer=on
2026-09-15 17:10:29.430 RX Response received in 763 ms, Content-Length: 33689
2026-09-15 17:10:29.914 RX Image download complete (33689 bytes, 1320 ms since button press). Starting decode...
2026-09-15 17:10:30.050 RX JPEG decoded successfully into PSRAM.
2026-09-15 17:10:30.051 RX LVGL image source updated. Total 1457 ms from button press (budget 20000 ms).
2026-09-15 17:10:34.373 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-15 17:10:37.676 RX [LOG] state=ready boot=6 session=boot-6 up_ms=46792 clock=synced setup=1 hooks=0 file_bytes=11830 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15923052544 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-15 17:10:37.676 RX [LOG] measured=1 stack_min=1988 internal_min=30736 internal_largest=14836 dma_min=23240 dma_largest=14836 writes=7 slow=0 write_max_us=2317 flush_max_us=4719 sd_max_us=98439 rotations=0 pruned=0 oversized=0
```

| Metric | Initial card run | Later writer start |
|--------|------------------|--------------------|
| HTTPS largest-block minimum | 14836 bytes | 14836 bytes |
| HTTPS internal heap minimum since boot | 30756 bytes | 30736 bytes |
| HTTPS probe duration | 853 ms | 762 ms |
| Periodic samples and maximum gap | 86; 10079 us | 76; 10081 us |
| Latest total time, same 33689-byte image size | 1372 ms | 1457 ms |
| Writer stack minimum unused bytes | 1992 | 1988 |

The failure was adequately sampled and did not improve with the changed order.
The file size stayed 11830 bytes and writes stayed 7 across the entire request.
Thus this observed dip did not require a new logger write during Latest.
It does not support attributing the problem to simultaneous SD write latency
or a temporary write-time DMA buffer. Previously allocated resources and their
heap layout remain candidates; the exact allocation is not yet identified.

The SDK configuration was rechecked: CONFIG_MBEDTLS_INTERNAL_MEM_ALLOC=y,
CONFIG_FATFS_ALLOC_PREFER_EXTRAM=y, CONFIG_FATFS_PER_FILE_CACHE=y,
CONFIG_FATFS_LFN_STACK=y, CONFIG_FATFS_SECTOR_4096=y, and
CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096. TLS allocation competes for internal
RAM. FATFS preference for PSRAM does not make every SD allocation external.
The writer retains a 6144-byte internal stack. The reported 1988-byte margin
implies at least 4156 bytes were used on measured paths; dropping directly to
a 4096-byte stack is not a justified fix, even before recovery and rotation tests.

**Recommended next investigation:** measure allocation phases (writer task and
stack, SD mount, and opening current.log) before choosing another memory change.
Capture current internal free bytes and largest block for each phase; keep
the existing Stage 0 probes unchanged. Account for concurrent startup allocations
when interpreting differences. Do not claim the 20 KiB gate is passed, reduce
its threshold, or move the writer stack into PSRAM without the plan's required
SDK, ROM and cache-off audit and bench validation.

No firmware changes or commits were made while recording this failed experiment.
The later writer start remains in fe3b5da for reproducibility; it is not a proven
memory fix. No additional run of the same Latest test is requested at this point.

## Retained allocation snapshots prepared - awaiting JP's next build

Following JP's observation that startup finishes before the web console connects,
the temporary diagnostic now retains ten startup memory snapshots until reboot.
log status retrieves them after attachment; no startup console capture is needed.
See [the handoff](../src/diagnostics/STAGE1.md#next-test-retained-startup-memory)
for fields and the single Latest test.

Only sd_diagnostics.cpp changes firmware in this follow-up. Snapshots cover clock
startup, task creation and entry, formatter allocation, mount, first writable
current.log open, and storage-setup completion. The fixed array is at most 240
internal bytes. Concurrent Wi-Fi allocations remain possible; no synchronization
barrier, stack relocation, timeout or storage-policy change was added.
Existing Stage 0 probes are unchanged. No compile, flash, commit or new board
measurement has been performed by the assistant. Stage 1 acceptance remains on hold.

## Retained startup snapshot test - 2026-09-15 17:29, boot 8

JP flashed the retained-snapshot build and attached the browser after setup.
Both log status replies contain identical ten-phase snapshots, including their
original boot-relative timestamps. Late attachment and repeat retrieval passed.
The 240-byte snapshot array is reported by the running firmware.

| Interval | Elapsed microseconds | Net internal free reduction, bytes | Largest-block reduction, bytes |
|----------|----------------------|------------------------------------|--------------------------------|
| Clock setup | 695 | 5580 | 8192 |
| Writer creation to task entry | 80 | 6528 | 4096 |
| PSRAM formatter allocation | 62 | 0 | 0 |
| Mount interval, overlaps Wi-Fi | 56757 | 22304 | 24576 |
| Post-mount preparation, overlaps Wi-Fi | 49223 | 14964 | 12288 |
| First writable current.log open | 1417 | 0 | 0 |

These are observed whole-system differences, not allocator ownership traces.
The three heap queries in each snapshot are sequential. Wi-Fi initialization
runs concurrently with the writer; the 22304-byte mount-interval drop and
14964-byte post-mount drop cannot be assigned wholly to SD.

Pinned core evidence also matters for the 5580-byte clock interval:
esp32-hal-time.c configTzTime calls esp_netif_init before configuring SNTP.
It can initialize shared network infrastructure, so the whole observed drop
must not be treated as removable clock-only overhead.

Writer creation shows a 6528-byte net reduction over 80 microseconds, consistent
with the configured 6144-byte internal stack plus task bookkeeping and any
concurrent changes. This is the clearest identified resident allocation.
Neither explicit PSRAM formatter allocation nor the first writable current.log
open showed a net internal reduction at their boundaries. That does not rule out
temporary allocations or resources retained from earlier metadata reads.

Latest succeeded: 33689 bytes, HTTPS window 743 ms, total display time 1301 ms.
The HTTPS largest-block minimum is still 14836 bytes, below the 20480-byte gate.
Coverage: 74 samples, maximum gap 10217 microseconds; internal minimum 30472 bytes.
Logger writes stayed 7 and file size stayed 24250 across the request.
There were no errors, drops, slow writes or truncations. Writer stack margin
remained 1988 bytes. Normal IMU averaged 49.13 Hz; its minimum was 27.02 Hz.

Stage 1 acceptance remains on hold. Do not shrink the writer to 4096 bytes:
the measured 6144 - 1988 = 4156 bytes used already exceeds that size.
Next work should inspect persistent allocation placement and the external-stack
safety constraints before selecting a correction. The mount interval alone is
not evidence to change SD buffers, and these results do not justify lowering
the gate. No additional flash or repeat of the identical test is requested yet.

### Captured evidence

```text
2026-09-15 17:29:42.416 RX [LOG] state=ready boot=8 session=boot-8 up_ms=26508 clock=synced setup=1 hooks=0 file_bytes=24250 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15923052544 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-15 17:29:42.418 RX [LOG] measured=1 stack_min=1988 internal_min=78776 internal_largest=31732 dma_min=71280 dma_largest=31732 writes=7 slow=0 write_max_us=2339 flush_max_us=4748 sd_max_us=64375 rotations=0 pruned=0 oversized=0
2026-09-15 17:29:42.418 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-15 17:29:42.418 RX [LOG MEM] phase=before_clock up_us=1783940 free=173212 largest=110580 heap_min_boot=173212
2026-09-15 17:29:42.418 RX [LOG MEM] phase=after_clock up_us=1784635 free=167632 largest=102388 heap_min_boot=167524
2026-09-15 17:29:42.418 RX [LOG MEM] phase=before_writer up_us=1784685 free=167632 largest=102388 heap_min_boot=167524
2026-09-15 17:29:42.418 RX [LOG MEM] phase=writer_entry up_us=1784765 free=161104 largest=98292 heap_min_boot=161104
2026-09-15 17:29:42.418 RX [LOG MEM] phase=after_formatter up_us=1784827 free=161104 largest=98292 heap_min_boot=161104
2026-09-15 17:29:42.420 RX [LOG MEM] phase=before_mount up_us=1785254 free=161104 largest=98292 heap_min_boot=161104
2026-09-15 17:29:42.420 RX [LOG MEM] phase=after_mount up_us=1842011 free=138800 largest=73716 heap_min_boot=127172
2026-09-15 17:29:42.420 RX [LOG MEM] phase=before_current_open up_us=1891234 free=123836 largest=61428 heap_min_boot=121872
2026-09-15 17:29:42.420 RX [LOG MEM] phase=after_current_open up_us=1892651 free=123836 largest=61428 heap_min_boot=121872
2026-09-15 17:29:42.420 RX [LOG MEM] phase=storage_done up_us=1905682 free=123836 largest=61428 heap_min_boot=121872
2026-09-15 17:29:47.040 RX [PROBE] window=normal run=1 ms=22758 heap_min_boot=78776 largest_min=31732 interval_ms=10 samples=2276 gap_max_us=10657 scan_max_us=700 timer=on imu_n=1112 imu_min_hz=27.02 imu_avg_hz=49.13
2026-09-15 17:29:47.784 RX [PROBE] window=image_https run=1 ms=743 heap_min_boot=30472 largest_min=14836 interval_ms=10 samples=74 gap_max_us=10217 scan_max_us=278 timer=on
2026-09-15 17:29:47.784 RX Response received in 744 ms, Content-Length: 33689
2026-09-15 17:29:48.269 RX LVGL image source updated. Total 1301 ms from button press (budget 20000 ms).
2026-09-15 17:29:56.157 RX [LOG] state=ready boot=8 session=boot-8 up_ms=40249 clock=synced setup=1 hooks=0 file_bytes=24250 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15923052544 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-15 17:29:56.157 RX [LOG] measured=1 stack_min=1988 internal_min=30472 internal_largest=14836 dma_min=22976 dma_largest=14836 writes=7 slow=0 write_max_us=2339 flush_max_us=4748 sd_max_us=64375 rotations=0 pruned=0 oversized=0
2026-09-15 17:29:56.160 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-15 17:29:56.160 RX [LOG MEM] phase=before_clock up_us=1783940 free=173212 largest=110580 heap_min_boot=173212
2026-09-15 17:29:56.160 RX [LOG MEM] phase=after_clock up_us=1784635 free=167632 largest=102388 heap_min_boot=167524
2026-09-15 17:29:56.160 RX [LOG MEM] phase=before_writer up_us=1784685 free=167632 largest=102388 heap_min_boot=167524
2026-09-15 17:29:56.160 RX [LOG MEM] phase=writer_entry up_us=1784765 free=161104 largest=98292 heap_min_boot=161104
2026-09-15 17:29:56.160 RX [LOG MEM] phase=after_formatter up_us=1784827 free=161104 largest=98292 heap_min_boot=161104
2026-09-15 17:29:56.160 RX [LOG MEM] phase=before_mount up_us=1785254 free=161104 largest=98292 heap_min_boot=161104
2026-09-15 17:29:56.160 RX [LOG MEM] phase=after_mount up_us=1842011 free=138800 largest=73716 heap_min_boot=127172
2026-09-15 17:29:56.160 RX [LOG MEM] phase=before_current_open up_us=1891234 free=123836 largest=61428 heap_min_boot=121872
2026-09-15 17:29:56.160 RX [LOG MEM] phase=after_current_open up_us=1892651 free=123836 largest=61428 heap_min_boot=121872
2026-09-15 17:29:56.160 RX [LOG MEM] phase=storage_done up_us=1905682 free=123836 largest=61428 heap_min_boot=121872
```


## Accepted PSRAM-stack A/B implementation prepared - results pending

JP accepted Claude's revised experiment review on 2026-09-15 and authorized
implementation, without build, flash or commit. All seven changes are implemented.
DIAG_WRITER_STACK_PSRAM defaults to 0; hooks also default to 0.

A retains the internal 6144-byte dynamic task. B uses an 8192-byte PSRAM stack,
file-scope internal StaticTask_t, no fallback, and cleanup followed by permanent
suspension. Placement, lifecycle, used bytes and final margin are retained for
log status. Stage 0 probe source/windows and the 20480-byte gate are unchanged.

The later-phase, hooks-only NVS stress commits a dummy key from the main task
for at most 30 seconds and 300 writes while the writer writes/flushes test
records. Actual counts and timestamps are reported. Nothing is auto-armed.

Source inspection found no contradiction to the seven accepted changes.
Build and hardware validation are pending with JP. No passing memory result or
Stage 1 acceptance is inferred. See [the exact test](../src/diagnostics/STAGE1.md#next-bench-test-accepted-writer-stack-ab-experiment).


Preparation validation: source-only branch checks covered stack mode 0/1 with
hooks 0/1, default flags, disabled logger creation paths, placement capture,
terminal cleanup ordering, unavailable readings and command lengths. Stage 0
probe files, companion.ino and existing network/media files were checked against
HEAD; existing startup snapshot and memory sampling functions are unchanged.
git diff --check passed. No compilation or hardware validation was performed.


## Writer-stack A/B Latest results - 2026-09-15 18:28 and 19:52

JP supplied both captures in one attachment. Both builds used the card, hooks=0,
ready logging, synced clock and an active writer. JP compiled and flashed them.
No firmware changes were made while recording these results.

| Measurement | A: internal 6144 bytes, boot 10 | B: PSRAM 8192 bytes, boot 12 |
|-------------|--------------------------------|----------------------------|
| Latest HTTPS lowest largest internal block | 14836 bytes | 26612 bytes |
| 20480-byte memory floor, this run | Fail, 5644 below | Pass, 6132 above |
| HTTPS heap_min_boot | 30468 bytes | 34964 bytes |
| Writer stack used maximum | 4204 bytes | 4204 bytes |
| Writer stack minimum margin | 1940 bytes | 3988 bytes |
| Stack start and local variable external | 0 and 0 | 1 and 1 |
| TCB internal, size | 1, 352 bytes | 1, 352 bytes |
| Placement valid | 1 | 1 |
| HTTPS window and periodic samples | 770 ms, 77 | 767 ms, 76 |
| Requested interval, maximum sample gap | 10 ms, 10121 us | 10 ms, 10271 us |
| Latest image size and total time | 33689 bytes, 1295 ms | 27437 bytes, 1273 ms |
| Normal IMU minimum and average | 35.55 Hz, 49.21 Hz | 32.20 Hz, 49.17 Hz |
| Queue drops, logger error, slow writes | 0, none, 0 | 0, none, 0 |

The sampled block improves by 11776 bytes in B. Sampling remained active across
both HTTPS windows. Neither capture reports allocation or TLS failure, a reset
during the request, or a logger error. Both images decoded and returned normally.

The measured stack use is identical. B's extra 2048 bytes appear entirely as
additional stack margin. stack_final_margin=-1 is expected while the writer is
active; terminal cleanup has not been tested.

The retained creation snapshots show A falling from 167616 to 161088 free internal
bytes, a 6528-byte change. B stays at 167264 bytes across creation. Its static
352-byte TCB exists before these snapshots: before_clock free space is 352 bytes
lower in B (172844 versus 173196). These observations support the intended
allocation change. They do not isolate every concurrent allocation or measure
the complete SD cost. The 4496-byte difference in heap_min_boot is not a resident
cost: that API sums heap-region low watermarks reached at potentially different
times. No inference about a specific 9344-byte split is needed.

The captures are about 84 minutes apart and the images differ in size. Record
their timing, but do not claim B is faster or use them to pass a paired performance
gate. Normal IMU averages remain near 49.2 Hz in these short windows.
The writer's write count stays at 7 across each request; this pair does not
exercise sustained SD writes concurrent with TLS.

Decision: B passes the initial Latest memory check; A reproduces the prior
failure. The 20480-byte gate is unchanged. Stage 1 acceptance, production use of
a PSRAM stack, Live performance, NVS stress, rotation/pruning and cleanup remain
pending.

Next single test: keep B with hooks off. Send log status, run a complete 60-second
Live cycle, then send log status again. Retain all live_tls and live probes and
Video timing lines, and report visual behavior. This checks B memory and stability;
the performance gate still requires a same-session comparison.
JP's local DIAG_WRITER_STACK_PSRAM=1 test override is preserved. The intended
default is still 0 pending acceptance; no commit or push was requested here.

### Captured evidence

Only diagnostic, command and image-result lines are retained below. Connection
chatter and the HTTPS URL are omitted. Timestamps and measured values are unchanged.

#### Build A

```text
2026-09-15 18:28:10.985 TX log status [CRLF]
2026-09-15 18:28:10.987 RX [LOG] state=ready boot=10 session=boot-10 up_ms=34927 clock=synced setup=1 hooks=0 file_bytes=58415 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15923019776 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-15 18:28:10.988 RX [LOG] measured=1 stack_min=1940 internal_min=78720 internal_largest=31732 dma_min=71224 dma_largest=31732 writes=7 slow=0 write_max_us=2666 flush_max_us=4732 sd_max_us=101491 rotations=0 pruned=0 oversized=0
2026-09-15 18:28:10.990 RX [LOG STACK] stack_mode=internal stack_bytes=6144 placement_valid=1 stack_start=0x3fccf7b8 stack_external=0 stack_local_external=0 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-15 18:28:10.990 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-15 18:28:10.990 RX [LOG MEM] phase=before_clock up_us=1782933 free=173196 largest=110580 heap_min_boot=173196
2026-09-15 18:28:10.990 RX [LOG MEM] phase=after_clock up_us=1783627 free=167616 largest=102388 heap_min_boot=167508
2026-09-15 18:28:10.990 RX [LOG MEM] phase=before_writer up_us=1783675 free=167616 largest=102388 heap_min_boot=167508
2026-09-15 18:28:10.991 RX [LOG MEM] phase=writer_entry up_us=1783753 free=161088 largest=98292 heap_min_boot=161088
2026-09-15 18:28:10.991 RX [LOG MEM] phase=after_formatter up_us=1783838 free=161088 largest=98292 heap_min_boot=161088
2026-09-15 18:28:10.991 RX [LOG MEM] phase=before_mount up_us=1784315 free=161088 largest=98292 heap_min_boot=161088
2026-09-15 18:28:10.991 RX [LOG MEM] phase=after_mount up_us=1885692 free=123680 largest=61428 heap_min_boot=121908
2026-09-15 18:28:10.991 RX [LOG MEM] phase=before_current_open up_us=1892860 free=123680 largest=61428 heap_min_boot=121908
2026-09-15 18:28:10.991 RX [LOG MEM] phase=after_current_open up_us=1894220 free=123680 largest=61428 heap_min_boot=121908
2026-09-15 18:28:10.991 RX [LOG MEM] phase=storage_done up_us=1907030 free=123680 largest=61428 heap_min_boot=121908
2026-09-15 18:28:15.729 RX [PROBE] window=normal run=1 ms=31264 heap_min_boot=78720 largest_min=31732 interval_ms=10 samples=3126 gap_max_us=10688 scan_max_us=457 timer=on imu_n=1532 imu_min_hz=35.55 imu_avg_hz=49.21
2026-09-15 18:28:16.500 RX [PROBE] window=image_https run=1 ms=770 heap_min_boot=30468 largest_min=14836 interval_ms=10 samples=77 gap_max_us=10121 scan_max_us=126 timer=on
2026-09-15 18:28:16.500 RX Response received in 771 ms, Content-Length: 33689
2026-09-15 18:28:16.815 RX Image download complete (33689 bytes, 1158 ms since button press). Starting decode...
2026-09-15 18:28:16.951 RX LVGL image source updated. Total 1295 ms from button press (budget 20000 ms).
2026-09-15 18:28:22.538 TX log status [CRLF]
2026-09-15 18:28:22.540 RX [LOG] state=ready boot=10 session=boot-10 up_ms=46480 clock=synced setup=1 hooks=0 file_bytes=58415 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15923019776 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-15 18:28:22.540 RX [LOG] measured=1 stack_min=1940 internal_min=30468 internal_largest=14836 dma_min=22972 dma_largest=14836 writes=7 slow=0 write_max_us=2666 flush_max_us=4732 sd_max_us=101491 rotations=0 pruned=0 oversized=0
2026-09-15 18:28:22.541 RX [LOG STACK] stack_mode=internal stack_bytes=6144 placement_valid=1 stack_start=0x3fccf7b8 stack_external=0 stack_local_external=0 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-15 18:28:22.542 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-15 18:28:22.543 RX [LOG MEM] phase=before_clock up_us=1782933 free=173196 largest=110580 heap_min_boot=173196
2026-09-15 18:28:22.543 RX [LOG MEM] phase=after_clock up_us=1783627 free=167616 largest=102388 heap_min_boot=167508
2026-09-15 18:28:22.543 RX [LOG MEM] phase=before_writer up_us=1783675 free=167616 largest=102388 heap_min_boot=167508
2026-09-15 18:28:22.543 RX [LOG MEM] phase=writer_entry up_us=1783753 free=161088 largest=98292 heap_min_boot=161088
2026-09-15 18:28:22.543 RX [LOG MEM] phase=after_formatter up_us=1783838 free=161088 largest=98292 heap_min_boot=161088
2026-09-15 18:28:22.543 RX [LOG MEM] phase=before_mount up_us=1784315 free=161088 largest=98292 heap_min_boot=161088
2026-09-15 18:28:22.544 RX [LOG MEM] phase=after_mount up_us=1885692 free=123680 largest=61428 heap_min_boot=121908
2026-09-15 18:28:22.544 RX [LOG MEM] phase=before_current_open up_us=1892860 free=123680 largest=61428 heap_min_boot=121908
2026-09-15 18:28:22.544 RX [LOG MEM] phase=after_current_open up_us=1894220 free=123680 largest=61428 heap_min_boot=121908
2026-09-15 18:28:22.544 RX [LOG MEM] phase=storage_done up_us=1907030 free=123680 largest=61428 heap_min_boot=121908
```

#### Build B

```text
2026-09-15 19:52:29.548 TX log status [CRLF]
2026-09-15 19:52:29.554 RX [LOG] state=ready boot=12 session=boot-12 up_ms=28136 clock=synced setup=1 hooks=0 file_bytes=106812 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922954240 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-15 19:52:29.554 RX [LOG] measured=1 stack_min=3988 internal_min=84920 internal_largest=31732 dma_min=77424 dma_largest=31732 writes=7 slow=0 write_max_us=1853 flush_max_us=4590 sd_max_us=98211 rotations=0 pruned=0 oversized=0
2026-09-15 19:52:29.556 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-15 19:52:29.556 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-15 19:52:29.556 RX [LOG MEM] phase=before_clock up_us=1778914 free=172844 largest=110580 heap_min_boot=172844
2026-09-15 19:52:29.557 RX [LOG MEM] phase=after_clock up_us=1779621 free=167264 largest=102388 heap_min_boot=167156
2026-09-15 19:52:29.557 RX [LOG MEM] phase=before_writer up_us=1779669 free=167264 largest=102388 heap_min_boot=167156
2026-09-15 19:52:29.558 RX [LOG MEM] phase=writer_entry up_us=1779875 free=167264 largest=102388 heap_min_boot=167156
2026-09-15 19:52:29.558 RX [LOG MEM] phase=after_formatter up_us=1779955 free=167264 largest=102388 heap_min_boot=167156
2026-09-15 19:52:29.558 RX [LOG MEM] phase=before_mount up_us=1780480 free=167264 largest=102388 heap_min_boot=167156
2026-09-15 19:52:29.558 RX [LOG MEM] phase=after_mount up_us=1878567 free=129992 largest=65524 heap_min_boot=128784
2026-09-15 19:52:29.558 RX [LOG MEM] phase=before_current_open up_us=1885996 free=129992 largest=65524 heap_min_boot=128784
2026-09-15 19:52:29.558 RX [LOG MEM] phase=after_current_open up_us=1887426 free=129992 largest=65524 heap_min_boot=128784
2026-09-15 19:52:29.558 RX [LOG MEM] phase=storage_done up_us=1900600 free=129992 largest=65524 heap_min_boot=128784
2026-09-15 19:52:35.068 RX [PROBE] window=normal run=1 ms=24667 heap_min_boot=84920 largest_min=31732 interval_ms=10 samples=2467 gap_max_us=10866 scan_max_us=981 timer=on imu_n=1207 imu_min_hz=32.20 imu_avg_hz=49.17
2026-09-15 19:52:35.836 RX [PROBE] window=image_https run=1 ms=767 heap_min_boot=34964 largest_min=26612 interval_ms=10 samples=76 gap_max_us=10271 scan_max_us=228 timer=on
2026-09-15 19:52:35.836 RX Response received in 768 ms, Content-Length: 27437
2026-09-15 19:52:36.138 RX Image download complete (27437 bytes, 1143 ms since button press). Starting decode...
2026-09-15 19:52:36.268 RX LVGL image source updated. Total 1273 ms from button press (budget 20000 ms).
2026-09-15 19:52:40.321 TX log status [CRLF]
2026-09-15 19:52:40.324 RX [LOG] state=ready boot=12 session=boot-12 up_ms=38906 clock=synced setup=1 hooks=0 file_bytes=106812 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922954240 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-15 19:52:40.324 RX [LOG] measured=1 stack_min=3988 internal_min=34964 internal_largest=26612 dma_min=27468 dma_largest=26612 writes=7 slow=0 write_max_us=1853 flush_max_us=4590 sd_max_us=98211 rotations=0 pruned=0 oversized=0
2026-09-15 19:52:40.325 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-15 19:52:40.326 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-15 19:52:40.326 RX [LOG MEM] phase=before_clock up_us=1778914 free=172844 largest=110580 heap_min_boot=172844
2026-09-15 19:52:40.326 RX [LOG MEM] phase=after_clock up_us=1779621 free=167264 largest=102388 heap_min_boot=167156
2026-09-15 19:52:40.327 RX [LOG MEM] phase=before_writer up_us=1779669 free=167264 largest=102388 heap_min_boot=167156
2026-09-15 19:52:40.327 RX [LOG MEM] phase=writer_entry up_us=1779875 free=167264 largest=102388 heap_min_boot=167156
2026-09-15 19:52:40.327 RX [LOG MEM] phase=after_formatter up_us=1779955 free=167264 largest=102388 heap_min_boot=167156
2026-09-15 19:52:40.327 RX [LOG MEM] phase=before_mount up_us=1780480 free=167264 largest=102388 heap_min_boot=167156
2026-09-15 19:52:40.327 RX [LOG MEM] phase=after_mount up_us=1878567 free=129992 largest=65524 heap_min_boot=128784
2026-09-15 19:52:40.327 RX [LOG MEM] phase=before_current_open up_us=1885996 free=129992 largest=65524 heap_min_boot=128784
2026-09-15 19:52:40.327 RX [LOG MEM] phase=after_current_open up_us=1887426 free=129992 largest=65524 heap_min_boot=128784
2026-09-15 19:52:40.328 RX [LOG MEM] phase=storage_done up_us=1900600 free=129992 largest=65524 heap_min_boot=128784
```


## PSRAM-stack Live after overnight uptime - 2026-09-16 07:34

JP reports normal video for the complete cycle. This is still B, boot 12, with
the same retained startup snapshots and stack address as the previous evening.
Before Live, uptime was 42154049 ms: about 11 hours 43 minutes. Hooks were off.

| Measurement | Result |
|-------------|--------|
| Full cycle | 187 frames in 60.4 seconds, 3.1 fps |
| Frame timing | 323 ms average, 1138 ms first frame, 1021 ms maximum gap |
| HTTP / decode / blit | 315 / 60 / 62 ms |
| Frame size and transfer rate | 18.4 KB, 102 KB/s |
| Live TLS run 1 | 653 ms, largest_min=14324, 65 samples, max gap 10668 us |
| Live TLS run 2 | 653 ms, largest_min=14324, 65 samples, max gap 10235 us |
| Full Live probe | 60363 ms, largest_min=14324, 6037 samples, max gap 11310 us |
| Sampling interval and timer | 10 ms, on in all windows |
| Preceding normal window | 38071 ms, largest_min=16372, 3807 samples |
| Normal IMU minimum and average | 33.05 Hz, 49.21 Hz, 1870 observations |
| Logger internal_largest, before and after | 14324 bytes |
| heap_min_boot, before and through Live | 22676 bytes |
| Writer stack used / margin | 4204 / 3988 bytes, unchanged |
| Placement | Valid, stack and local variable external, TCB internal |
| Logger state and errors | ready, error=none, errno=0 |
| Queue / high water / drops | 0/16, 1, 0 before and after |
| Writes and file size | 710 to 711; 503263 to 503827 bytes |
| Slow writes / maximum write / maximum flush | 0 / 5682 us / 6969 us |

Result: functional Live completion, but memory gate FAIL. 14324 is 6156 bytes
below the unchanged 20480-byte floor. The failure is confirmed independently in
both window-local TLS probes; it is not merely an old value in log status.
No TLS or allocation failure is reported, and the stack placement and usage
remain consistent with the intended B experiment.

The low memory condition predates this Live cycle. Before the button press,
log status already reported internal_largest=14324. In sd_diagnostics.cpp,
memorySample retains the minimum of all writer observations; diagnosticsPrintStatus
prints that history, not an instantaneous largest block. The preceding normal
probe also observed only 16372 bytes at its lowest point. These readings do not
tell us when the first reduction happened or which operation caused it.
The current.log HEALTH field named internal_largest is different: writeHealth
queries its current value when constructing the record.

The previous evening's B Latest result (26612 bytes) remains a valid result for
that request, not evidence that the whole boot meets the gate. The new capture
does not distinguish a persistent allocation, fragmentation, a leak or a
temporary demand during earlier activity. Uptime alone is not a cause.
Do not compare today's 3.1 fps against a previous-day performance baseline.

The write count advanced once between status captures; this is ordinary logger
activity, not the sustained overlap exercised by the later NVS stress.
No reset is indicated across the captured Live cycle. Continuous error-free
operation during every intervening overnight minute is not established.

Decision: keep Stage 1 acceptance on hold. Pause paired performance and hooks
stress tests. Next run the same B firmware after a normal shutdown and restart:
log status, Live as the first media request, then log status. Keep hooks=0 and
send the complete capture. This separates fresh-boot behavior from elapsed time
and prior operations without changing code. A recovery after restart would not
by itself prove a leak; a repeat failure would direct inspection to fresh Live
allocation behavior.

Documentation only in this turn. No firmware changes, builds, commits or pushes.
JP's local switch=1 remains unchanged.

### Captured evidence

```text
2026-09-16 07:34:25.988 EVENT Console cleared.
2026-09-16 07:34:34.003 TX log status [CRLF]
2026-09-16 07:34:34.007 RX [LOG] state=ready boot=12 session=boot-12 up_ms=42154049 clock=synced setup=1 hooks=0 file_bytes=503263 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922561024 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 07:34:34.007 RX [LOG] measured=1 stack_min=3988 internal_min=22676 internal_largest=14324 dma_min=15180 dma_largest=14324 writes=710 slow=0 write_max_us=5682 flush_max_us=6969 sd_max_us=98211 rotations=0 pruned=0 oversized=0
2026-09-16 07:34:34.009 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 07:34:34.009 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 07:34:34.009 RX [LOG MEM] phase=before_clock up_us=1778914 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 07:34:34.009 RX [LOG MEM] phase=after_clock up_us=1779621 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:34:34.010 RX [LOG MEM] phase=before_writer up_us=1779669 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:34:34.010 RX [LOG MEM] phase=writer_entry up_us=1779875 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:34:34.010 RX [LOG MEM] phase=after_formatter up_us=1779955 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:34:34.010 RX [LOG MEM] phase=before_mount up_us=1780480 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:34:34.010 RX [LOG MEM] phase=after_mount up_us=1878567 free=129992 largest=65524 heap_min_boot=128784
2026-09-16 07:34:34.010 RX [LOG MEM] phase=before_current_open up_us=1885996 free=129992 largest=65524 heap_min_boot=128784
2026-09-16 07:34:34.010 RX [LOG MEM] phase=after_current_open up_us=1887426 free=129992 largest=65524 heap_min_boot=128784
2026-09-16 07:34:34.010 RX [LOG MEM] phase=storage_done up_us=1900600 free=129992 largest=65524 heap_min_boot=128784
2026-09-16 07:34:36.583 RX Screen touched, resetting inactivity timer.
2026-09-16 07:34:36.583 RX Live button clicked -> starting live feed
2026-09-16 07:34:36.584 RX [PROBE] window=normal run=705 ms=38071 heap_min_boot=22676 largest_min=16372 interval_ms=10 samples=3807 gap_max_us=10786 scan_max_us=439 timer=on imu_n=1870 imu_min_hz=33.05 imu_avg_hz=49.21
2026-09-16 07:34:36.584 RX Screen 2 Loaded.
2026-09-16 07:34:36.655 RX Video: endpoint parsed, port 9835, path /esp32/live
2026-09-16 07:34:37.310 RX [PROBE] window=live_tls run=1 ms=653 heap_min_boot=22676 largest_min=14324 interval_ms=10 samples=65 gap_max_us=10668 scan_max_us=136 timer=on
2026-09-16 07:34:37.601 RX Video: frame 432x768, panel 368x448 -> gap x=-64 y=-320, pan x=-15 y=32
2026-09-16 07:35:09.769 RX [PROBE] window=live_tls run=2 ms=653 heap_min_boot=22676 largest_min=14324 interval_ms=10 samples=65 gap_max_us=10235 scan_max_us=154 timer=on
2026-09-16 07:35:36.944 RX Video: 187 frames in 60.4s (3.1 fps) | http 315 | decode 60 | blit 62 | frame 323 ms | first_frame 1138 | max_gap 1021 ms
2026-09-16 07:35:36.944 RX Video: http = ttfb 134 + xfer 180 ms | frame 18.4 KB | 102 KB/s while transferring
2026-09-16 07:35:36.944 RX Video: free PSRAM 7610540, free heap 46548
2026-09-16 07:35:36.946 RX [PROBE] window=live run=1 ms=60363 heap_min_boot=22676 largest_min=14324 interval_ms=10 samples=6037 gap_max_us=11310 scan_max_us=1032 timer=on
2026-09-16 07:35:36.946 RX Video: returning to previous screen
2026-09-16 07:35:36.946 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 07:35:36.947 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 07:35:39.804 TX log status [CRLF]
2026-09-16 07:35:39.811 RX [LOG] state=ready boot=12 session=boot-12 up_ms=42219854 clock=synced setup=1 hooks=0 file_bytes=503827 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922561024 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 07:35:39.812 RX [LOG] measured=1 stack_min=3988 internal_min=22676 internal_largest=14324 dma_min=15180 dma_largest=14324 writes=711 slow=0 write_max_us=5682 flush_max_us=6969 sd_max_us=98211 rotations=0 pruned=0 oversized=0
2026-09-16 07:35:39.813 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 07:35:39.814 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 07:35:39.814 RX [LOG MEM] phase=before_clock up_us=1778914 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 07:35:39.814 RX [LOG MEM] phase=after_clock up_us=1779621 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:35:39.814 RX [LOG MEM] phase=before_writer up_us=1779669 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:35:39.814 RX [LOG MEM] phase=writer_entry up_us=1779875 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:35:39.814 RX [LOG MEM] phase=after_formatter up_us=1779955 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:35:39.814 RX [LOG MEM] phase=before_mount up_us=1780480 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:35:39.814 RX [LOG MEM] phase=after_mount up_us=1878567 free=129992 largest=65524 heap_min_boot=128784
2026-09-16 07:35:39.814 RX [LOG MEM] phase=before_current_open up_us=1885996 free=129992 largest=65524 heap_min_boot=128784
2026-09-16 07:35:39.814 RX [LOG MEM] phase=after_current_open up_us=1887426 free=129992 largest=65524 heap_min_boot=128784
2026-09-16 07:35:39.814 RX [LOG MEM] phase=storage_done up_us=1900600 free=129992 largest=65524 heap_min_boot=128784
```


## PSRAM-stack fresh-boot Live repeat - 2026-09-16 07:39

JP powered down and repeated the test, reporting normal video for the complete
cycle. Boot=13 and the pre-Live uptime of 39894 ms confirm a new boot.
B remains active: 8192-byte PSRAM stack, valid placement, internal 352-byte TCB,
hooks=0. No firmware change or reflash was requested for this repeat.

| Measurement | Fresh-boot B result |
|-------------|---------------------|
| Pre-Live logger largest historical minimum | 31732 bytes |
| Pre-Live normal window | 34634 ms, largest_min=31732, 3464 samples |
| Normal IMU minimum and average | 29.37 Hz, 49.22 Hz, 1701 observations |
| Live TLS run 1 | 777 ms, largest_min=25588, 77 samples, max gap 10486 us |
| Live TLS run 2 | 644 ms, largest_min=27636, 64 samples, max gap 11126 us |
| Full Live probe | 60428 ms, largest_min=25588, 6042 samples, max gap 11803 us |
| Requested sampler interval / state | 10 ms / on |
| Full cycle | 188 frames in 60.4 seconds, 3.1 fps |
| Frame timing | 321 ms average, 1333 ms first frame, 930 ms maximum gap |
| HTTP / decode / blit | 313 / 60 / 62 ms |
| Frame size and transfer rate | 18.8 KB, 104 KB/s |
| heap_min_boot after Live | 35540 bytes |
| Writer stack used / minimum margin | 4204 / 3988 bytes, unchanged |
| Logger state / errors / drops | ready / none / zero, before and after |
| Writes / file size | 7 to 8 / 506808 to 507366 bytes |
| Slow writes / maximum write / flush | 0 / 2809 us / 6114 us |
| Maximum recorded SD operation | 169424 us |
| Rotations / pruning | 0 / 0 |

Result: this fresh-boot Live memory check PASSES. The lowest observed largest
internal block is 25588 bytes, 5108 above the unchanged 20480-byte floor.
Both TLS windows pass, and adequate periodic sampling continued throughout.
Video completed and returned to the previous screen without a reported error.

The overnight boot measured 14324 bytes in both TLS windows and full Live.
Fresh-boot Live is 11264 bytes higher. This shows the earlier failure is not
unavoidable on the first Live cycle in B. It does not identify its cause:
elapsed time, intervening activity and allocation history differ.
It does not prove a leak, or that logging caused the earlier low headroom.

Both morning runs display 3.1 fps and JP reports normal video. They are both B,
so this does not satisfy the logging-off versus logging-on performance gate.
The frame-description serial line arrived with the second TLS summary, but the
device reports first_frame=1333 and max_gap=930 ms. Host receipt timing does not
establish a 32-second video stall; JP observed none.

Stage 1 acceptance remains on hold because the overnight memory failure is
unresolved. Keep fault hooks and performance-comparison work paused for now.
Next single test: without rebooting B or introducing Latest or MQTT fault commands,
run three more full Live cycles. Capture log status before the set and after
each cycle, with about 10 seconds on the normal screen between cycles.
Compare the per-window probe minima, not only the historical status minimum.
Stop remaining cycles if a window falls below 20480 or any functional error occurs.
Stable repeats would not rule out longer-term or other-operation effects.

The local B switch is preserved. This turn changed documentation only; no
firmware edits, builds, commits or pushes.

### Captured evidence

```text
2026-09-16 07:39:00.277 EVENT Console cleared.
2026-09-16 07:39:01.160 EVENT Connect requested. After open: DTR=true, RTS=false.
2026-09-16 07:39:03.879 EVENT Explicit signals applied. After open: DTR=true, RTS=false.
2026-09-16 07:39:03.879 EVENT Connected. Send status; compare with the previous reading if available.
2026-09-16 07:39:03.879 RX Initial MQTT connection successful!
2026-09-16 07:39:03.879 RX [NET] WiFi=CONNECTED | MQTT=REMOTE CONNECTED
2026-09-16 07:39:03.879 RX --- Setup complete: CPU 240 MHz | heap 90480 | PSRAM 8336064 ---
2026-09-16 07:39:03.879 RX
2026-09-16 07:39:03.879 RX [TEST] Serial bench commands: off, on, status, log status. Send with CR or LF.
2026-09-16 07:39:11.202 TX log status [CRLF]
2026-09-16 07:39:11.208 RX [LOG] state=ready boot=13 session=boot-13 up_ms=39894 clock=synced setup=1 hooks=0 file_bytes=506808 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922561024 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 07:39:11.209 RX [LOG] measured=1 stack_min=3988 internal_min=84916 internal_largest=31732 dma_min=77420 dma_largest=31732 writes=7 slow=0 write_max_us=2809 flush_max_us=6114 sd_max_us=169424 rotations=0 pruned=0 oversized=0
2026-09-16 07:39:11.210 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 07:39:11.210 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 07:39:11.210 RX [LOG MEM] phase=before_clock up_us=1781915 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 07:39:11.210 RX [LOG MEM] phase=after_clock up_us=1782603 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:39:11.211 RX [LOG MEM] phase=before_writer up_us=1782651 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:39:11.211 RX [LOG MEM] phase=writer_entry up_us=1782857 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:39:11.211 RX [LOG MEM] phase=after_formatter up_us=1782938 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:39:11.211 RX [LOG MEM] phase=before_mount up_us=1783508 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:39:11.211 RX [LOG MEM] phase=after_mount up_us=1952818 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:39:11.211 RX [LOG MEM] phase=before_current_open up_us=1960112 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:39:11.211 RX [LOG MEM] phase=after_current_open up_us=1961500 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:39:11.211 RX [LOG MEM] phase=storage_done up_us=1975173 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:39:15.330 RX Screen touched, resetting inactivity timer.
2026-09-16 07:39:15.330 RX Live button clicked -> starting live feed
2026-09-16 07:39:15.331 RX [PROBE] window=normal run=1 ms=34634 heap_min_boot=84916 largest_min=31732 interval_ms=10 samples=3464 gap_max_us=10647 scan_max_us=411 timer=on imu_n=1701 imu_min_hz=29.37 imu_avg_hz=49.22
2026-09-16 07:39:15.331 RX Screen 2 Loaded.
2026-09-16 07:39:15.402 RX Video: endpoint parsed, port 9835, path /esp32/live
2026-09-16 07:39:16.180 RX [PROBE] window=live_tls run=1 ms=777 heap_min_boot=36200 largest_min=25588 interval_ms=10 samples=77 gap_max_us=10486 scan_max_us=535 timer=on
2026-09-16 07:39:47.568 RX Video: frame 432x768, panel 368x448 -> gap x=-64 y=-320, pan x=-15 y=32
2026-09-16 07:39:47.568 RX [PROBE] window=live_tls run=2 ms=644 heap_min_boot=35540 largest_min=27636 interval_ms=10 samples=64 gap_max_us=11126 scan_max_us=377 timer=on
2026-09-16 07:40:15.757 RX Video: 188 frames in 60.4s (3.1 fps) | http 313 | decode 60 | blit 62 | frame 321 ms | first_frame 1333 | max_gap 930 ms
2026-09-16 07:40:15.757 RX Video: http = ttfb 132 + xfer 181 ms | frame 18.8 KB | 104 KB/s while transferring
2026-09-16 07:40:15.757 RX Video: free PSRAM 7610528, free heap 46824
2026-09-16 07:40:15.759 RX [PROBE] window=live run=1 ms=60428 heap_min_boot=35540 largest_min=25588 interval_ms=10 samples=6042 gap_max_us=11803 scan_max_us=1435 timer=on
2026-09-16 07:40:15.759 RX Video: returning to previous screen
2026-09-16 07:40:15.759 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 07:40:15.759 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 07:40:19.905 TX log status [CRLF]
2026-09-16 07:40:19.912 RX [LOG] state=ready boot=13 session=boot-13 up_ms=108600 clock=synced setup=1 hooks=0 file_bytes=507366 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922561024 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 07:40:19.913 RX [LOG] measured=1 stack_min=3988 internal_min=35540 internal_largest=25588 dma_min=28044 dma_largest=25588 writes=8 slow=0 write_max_us=2809 flush_max_us=6114 sd_max_us=169424 rotations=0 pruned=0 oversized=0
2026-09-16 07:40:19.914 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 07:40:19.914 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 07:40:19.914 RX [LOG MEM] phase=before_clock up_us=1781915 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 07:40:19.916 RX [LOG MEM] phase=after_clock up_us=1782603 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:40:19.916 RX [LOG MEM] phase=before_writer up_us=1782651 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:40:19.916 RX [LOG MEM] phase=writer_entry up_us=1782857 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:40:19.916 RX [LOG MEM] phase=after_formatter up_us=1782938 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:40:19.916 RX [LOG MEM] phase=before_mount up_us=1783508 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:40:19.916 RX [LOG MEM] phase=after_mount up_us=1952818 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:40:19.916 RX [LOG MEM] phase=before_current_open up_us=1960112 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:40:19.916 RX [LOG MEM] phase=after_current_open up_us=1961500 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:40:19.916 RX [LOG MEM] phase=storage_done up_us=1975173 free=129996 largest=65524 heap_min_boot=128864
```


## PSRAM-stack three repeated Live cycles - 2026-09-16 07:43

JP reports normal video throughout all three cycles. This remains boot 13, B,
hooks=0, with valid PSRAM stack placement and an internal TCB. Including the
earlier fresh-boot run, four full Live cycles have now passed in this boot.
The web console reconnected before this set; boot identity and uptime remained
consistent with no board restart.

| Measurement | Repeat 1, Live run 2 | Repeat 2, Live run 3 | Repeat 3, Live run 4 |
|-------------|----------------------|----------------------|----------------------|
| Frames / seconds / reported fps | 194 / 60.3 / 3.2 | 196 / 60.3 / 3.2 | 194 / 60.4 / 3.2 |
| Full Live largest_min | 25588 | 25588 | 26612 |
| TLS largest_min, first / second | 25588 / 25588 | 27636 / 25588 | 27636 / 27636 |
| Full Live periodic samples | 6025 | 6032 | 6042 |
| Full Live maximum sample gap, us | 11290 | 11077 | 11272 |
| First frame / maximum gap, ms | 1139 / 987 | 1187 / 959 | 1145 / 895 |
| HTTP / decode / blit, ms | 303 / 60 / 62 | 299 / 60 / 62 | 304 / 60 / 62 |
| Mean frame, ms | 311 | 308 | 311 |
| Frame KB / transfer KB per second | 18.9 / 110 | 18.9 / 112 | 18.9 / 109 |
| Video end free heap | 46796 | 46796 | 46796 |
| Video end free PSRAM | 7610596 | 7610336 | 7610336 |
| heap_min_boot after cycle | 35140 | 35016 | 34840 |
| Stack used / margin, bytes | 4204 / 3988 | 4204 / 3988 | 4204 / 3988 |

All three memory checks PASS the unchanged 20480-byte floor. The smallest
window-local result is 25588, leaving 5108 bytes above the floor. Timer sampling
was on at 10 ms throughout. All six TLS windows had 61 to 74 samples and maximum
sample gaps between 10038 and 10698 us.

The normal windows immediately before the cycles all measured largest_min=31732.
Their IMU minimum/average pairs were 32.21/49.17, 10.54/49.12 and 10.32/49.26 Hz.
The short between-cycle windows include low individual IMU readings, while the
averages remain near 49.2 Hz. Record those minima without assigning an unmeasured
cause or declaring the IMU performance gate complete.

Logger state stayed ready with no errors, drops, truncation or suppressed records.
Queue high water remained 1/16. Writes advanced from 12 to 15 and file size from
509611 to 511298 bytes. Slow writes stayed zero; write_max_us=2809,
flush_max_us=6114 and sd_max_us=169424 were unchanged. No rotation or pruning
occurred. The historical logger internal_largest remained 25588 even when the
last Live window measured 26612, as expected for a retained minimum.

This test did not reproduce the overnight 14324-byte condition. There is no
progressive reduction in the largest-block minima over these three Live cycles.
Video-end free heap also stayed at 46796 bytes. This does not prove no leak exists:
heap_min_boot fell by 700 bytes from the preceding fresh-boot value of 35540 to
34840. That field sums historical per-region minima and cannot alone establish
persistent allocation growth. The small PSRAM change also needs longer-term
evidence before being assigned a cause.

Decision: repeated Live passes, overnight cause unresolved, Stage 1 not accepted.
The next single test is Latest followed by Live without rebooting B: log status,
Latest, return to dashboard, log status, full Live, log status.
The prior overnight boot had a Latest request; the supplied boot-13 sequence
contains Live only. This tests media-operation order, not a known fault.
Stop remaining operations if the memory floor or functional checks fail.
Keep hooks off and defer NVS stress and paired performance acceptance.

Documentation only in this turn. No firmware changes, builds, commits or pushes.
The local B switch remains unchanged.

### Captured evidence

```text
2026-09-16 07:41:21.864 EVENT Console cleared.
2026-09-16 07:43:38.188 EVENT Connect requested. After open: DTR=true, RTS=false.
2026-09-16 07:43:40.925 EVENT Explicit signals applied. After open: DTR=true, RTS=false.
2026-09-16 07:43:40.925 EVENT Connected. Send status; compare with the previous reading if available.
2026-09-16 07:43:40.925 RX max_us=1014 timer=on imu_n=2947 imu_min_hz=27.78 imu_avg_hz=49.22
2026-09-16 07:43:40.925 RX [PROBE] window=normal run=4 ms=60001 heap_min_boot=35540 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10970 scan_max_us=636 timer=on imu_n=2948 imu_min_hz=29.40 imu_avg_hz=49.23
2026-09-16 07:43:42.779 TX log status [CRLF]
2026-09-16 07:43:42.783 RX [LOG] state=ready boot=13 session=boot-13 up_ms=311475 clock=synced setup=1 hooks=0 file_bytes=509611 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922561024 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 07:43:42.784 RX [LOG] measured=1 stack_min=3988 internal_min=35540 internal_largest=25588 dma_min=28044 dma_largest=25588 writes=12 slow=0 write_max_us=2809 flush_max_us=6114 sd_max_us=169424 rotations=0 pruned=0 oversized=0
2026-09-16 07:43:42.784 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 07:43:42.785 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 07:43:42.785 RX [LOG MEM] phase=before_clock up_us=1781915 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 07:43:42.785 RX [LOG MEM] phase=after_clock up_us=1782603 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:43:42.785 RX [LOG MEM] phase=before_writer up_us=1782651 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:43:42.785 RX [LOG MEM] phase=writer_entry up_us=1782857 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:43:42.786 RX [LOG MEM] phase=after_formatter up_us=1782938 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:43:42.786 RX [LOG MEM] phase=before_mount up_us=1783508 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:43:42.786 RX [LOG MEM] phase=after_mount up_us=1952818 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:43:42.786 RX [LOG MEM] phase=before_current_open up_us=1960112 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:43:42.786 RX [LOG MEM] phase=after_current_open up_us=1961500 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:43:42.786 RX [LOG MEM] phase=storage_done up_us=1975173 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:43:45.706 RX Screen touched, resetting inactivity timer.
2026-09-16 07:43:45.706 RX Live button clicked -> starting live feed
2026-09-16 07:43:45.707 RX [PROBE] window=normal run=5 ms=29836 heap_min_boot=35540 largest_min=31732 interval_ms=10 samples=2984 gap_max_us=10954 scan_max_us=1085 timer=on imu_n=1464 imu_min_hz=32.21 imu_avg_hz=49.17
2026-09-16 07:43:45.707 RX Screen 2 Loaded.
2026-09-16 07:43:45.779 RX Video: endpoint parsed, port 9835, path /esp32/live
2026-09-16 07:44:17.459 RX [PROBE] window=live_tls run=3 ms=653 heap_min_boot=35540 largest_min=25588 interval_ms=10 samples=65 gap_max_us=10043 scan_max_us=135 timer=on
2026-09-16 07:44:17.459 RX [PROBE] window=live_tls run=4 ms=670 heap_min_boot=35412 largest_min=25588 interval_ms=10 samples=67 gap_max_us=10038 scan_max_us=138 timer=on
2026-09-16 07:44:45.966 RX Video: 194 frames in 60.3s (3.2 fps) | http 303 | decode 60 | blit 62 | frame 311 ms | first_frame 1139 | max_gap 987 ms
2026-09-16 07:44:45.966 RX Video: http = ttfb 130 + xfer 173 ms | frame 18.9 KB | 110 KB/s while transferring
2026-09-16 07:44:45.966 RX Video: free PSRAM 7610596, free heap 46796
2026-09-16 07:44:45.968 RX [PROBE] window=live run=2 ms=60253 heap_min_boot=35140 largest_min=25588 interval_ms=10 samples=6025 gap_max_us=11290 scan_max_us=1406 timer=on
2026-09-16 07:44:45.968 RX Video: returning to previous screen
2026-09-16 07:44:45.968 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 07:44:45.969 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 07:44:48.016 TX log status [CRLF]
2026-09-16 07:44:48.019 RX [LOG] state=ready boot=13 session=boot-13 up_ms=376704 clock=synced setup=1 hooks=0 file_bytes=510173 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922561024 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 07:44:48.021 RX [LOG] measured=1 stack_min=3988 internal_min=35140 internal_largest=25588 dma_min=27644 dma_largest=25588 writes=13 slow=0 write_max_us=2809 flush_max_us=6114 sd_max_us=169424 rotations=0 pruned=0 oversized=0
2026-09-16 07:44:48.021 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 07:44:48.021 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 07:44:48.022 RX [LOG MEM] phase=before_clock up_us=1781915 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 07:44:48.022 RX [LOG MEM] phase=after_clock up_us=1782603 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:44:48.022 RX [LOG MEM] phase=before_writer up_us=1782651 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:44:48.022 RX [LOG MEM] phase=writer_entry up_us=1782857 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:44:48.022 RX [LOG MEM] phase=after_formatter up_us=1782938 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:44:48.022 RX [LOG MEM] phase=before_mount up_us=1783508 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:44:48.023 RX [LOG MEM] phase=after_mount up_us=1952818 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:44:48.023 RX [LOG MEM] phase=before_current_open up_us=1960112 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:44:48.023 RX [LOG MEM] phase=after_current_open up_us=1961500 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:44:48.023 RX [LOG MEM] phase=storage_done up_us=1975173 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:44:58.471 RX Screen touched, resetting inactivity timer.
2026-09-16 07:44:58.471 RX Live button clicked -> starting live feed
2026-09-16 07:44:58.472 RX [PROBE] window=normal run=6 ms=12502 heap_min_boot=35140 largest_min=31732 interval_ms=10 samples=1251 gap_max_us=10236 scan_max_us=352 timer=on imu_n=610 imu_min_hz=10.54 imu_avg_hz=49.12
2026-09-16 07:44:58.472 RX Screen 2 Loaded.
2026-09-16 07:44:58.544 RX Video: endpoint parsed, port 9835, path /esp32/live
2026-09-16 07:44:59.290 RX [PROBE] window=live_tls run=5 ms=746 heap_min_boot=35016 largest_min=27636 interval_ms=10 samples=74 gap_max_us=10650 scan_max_us=346 timer=on
2026-09-16 07:45:58.802 RX [PROBE] window=live_tls run=6 ms=665 heap_min_boot=35016 largest_min=25588 interval_ms=10 samples=67 gap_max_us=10698 scan_max_us=583 timer=on
2026-09-16 07:45:58.802 RX Video: 196 frames in 60.3s (3.2 fps) | http 299 | decode 60 | blit 62 | frame 308 ms | first_frame 1187 | max_gap 959 ms
2026-09-16 07:45:58.803 RX Video: http = ttfb 130 + xfer 170 ms | frame 18.9 KB | 112 KB/s while transferring
2026-09-16 07:45:58.803 RX Video: free PSRAM 7610336, free heap 46796
2026-09-16 07:45:58.805 RX [PROBE] window=live run=3 ms=60322 heap_min_boot=35016 largest_min=25588 interval_ms=10 samples=6032 gap_max_us=11077 scan_max_us=989 timer=on
2026-09-16 07:45:58.805 RX Video: returning to previous screen
2026-09-16 07:45:58.805 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 07:45:58.805 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 07:46:02.024 TX log status [CRLF]
2026-09-16 07:46:02.025 RX [LOG] state=ready boot=13 session=boot-13 up_ms=450701 clock=synced setup=1 hooks=0 file_bytes=510735 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922561024 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 07:46:02.026 RX [LOG] measured=1 stack_min=3988 internal_min=35016 internal_largest=25588 dma_min=27520 dma_largest=25588 writes=14 slow=0 write_max_us=2809 flush_max_us=6114 sd_max_us=169424 rotations=0 pruned=0 oversized=0
2026-09-16 07:46:02.027 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 07:46:02.028 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 07:46:02.028 RX [LOG MEM] phase=before_clock up_us=1781915 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 07:46:02.028 RX [LOG MEM] phase=after_clock up_us=1782603 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:46:02.028 RX [LOG MEM] phase=before_writer up_us=1782651 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:46:02.029 RX [LOG MEM] phase=writer_entry up_us=1782857 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:46:02.029 RX [LOG MEM] phase=after_formatter up_us=1782938 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:46:02.029 RX [LOG MEM] phase=before_mount up_us=1783508 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:46:02.029 RX [LOG MEM] phase=after_mount up_us=1952818 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:46:02.029 RX [LOG MEM] phase=before_current_open up_us=1960112 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:46:02.029 RX [LOG MEM] phase=after_current_open up_us=1961500 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:46:02.029 RX [LOG MEM] phase=storage_done up_us=1975173 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:46:12.062 RX Screen touched, resetting inactivity timer.
2026-09-16 07:46:12.063 RX Live button clicked -> starting live feed
2026-09-16 07:46:12.063 RX [PROBE] window=normal run=7 ms=13255 heap_min_boot=35016 largest_min=31732 interval_ms=10 samples=1326 gap_max_us=10661 scan_max_us=417 timer=on imu_n=649 imu_min_hz=10.32 imu_avg_hz=49.26
2026-09-16 07:46:12.063 RX Screen 2 Loaded.
2026-09-16 07:46:12.135 RX Video: endpoint parsed, port 9835, path /esp32/live
2026-09-16 07:46:12.779 RX [PROBE] window=live_tls run=7 ms=643 heap_min_boot=35000 largest_min=27636 interval_ms=10 samples=64 gap_max_us=10323 scan_max_us=157 timer=on
2026-09-16 07:46:43.466 RX [PROBE] window=live_tls run=8 ms=614 heap_min_boot=34840 largest_min=27636 interval_ms=10 samples=61 gap_max_us=10573 scan_max_us=131 timer=on
2026-09-16 07:47:12.490 RX Video: 194 frames in 60.4s (3.2 fps) | http 304 | decode 60 | blit 62 | frame 311 ms | first_frame 1145 | max_gap 895 ms
2026-09-16 07:47:12.490 RX Video: http = ttfb 130 + xfer 174 ms | frame 18.9 KB | 109 KB/s while transferring
2026-09-16 07:47:12.490 RX Video: free PSRAM 7610336, free heap 46796
2026-09-16 07:47:12.492 RX [PROBE] window=live run=4 ms=60418 heap_min_boot=34840 largest_min=26612 interval_ms=10 samples=6042 gap_max_us=11272 scan_max_us=946 timer=on
2026-09-16 07:47:12.492 RX Video: returning to previous screen
2026-09-16 07:47:12.492 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 07:47:12.492 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 07:47:15.542 TX log status [CRLF]
2026-09-16 07:47:15.544 RX [LOG] state=ready boot=13 session=boot-13 up_ms=524211 clock=synced setup=1 hooks=0 file_bytes=511298 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922561024 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 07:47:15.545 RX [LOG] measured=1 stack_min=3988 internal_min=34840 internal_largest=25588 dma_min=27344 dma_largest=25588 writes=15 slow=0 write_max_us=2809 flush_max_us=6114 sd_max_us=169424 rotations=0 pruned=0 oversized=0
2026-09-16 07:47:15.547 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 07:47:15.547 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 07:47:15.547 RX [LOG MEM] phase=before_clock up_us=1781915 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 07:47:15.547 RX [LOG MEM] phase=after_clock up_us=1782603 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:47:15.547 RX [LOG MEM] phase=before_writer up_us=1782651 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:47:15.548 RX [LOG MEM] phase=writer_entry up_us=1782857 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:47:15.548 RX [LOG MEM] phase=after_formatter up_us=1782938 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:47:15.548 RX [LOG MEM] phase=before_mount up_us=1783508 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:47:15.548 RX [LOG MEM] phase=after_mount up_us=1952818 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:47:15.548 RX [LOG MEM] phase=before_current_open up_us=1960112 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:47:15.548 RX [LOG MEM] phase=after_current_open up_us=1961500 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:47:15.548 RX [LOG MEM] phase=storage_done up_us=1975173 free=129996 largest=65524 heap_min_boot=128864
```


### JP follow-up: screen selection test takes priority

JP suggested testing the G-meter and inclinometer. Move these ahead of the
proposed Latest-to-Live check, one screen at a time. The next test is G-meter:
log status on the dashboard, select G-meter for at least 35 seconds, log status,
return to dashboard for at least 35 seconds, log status, then full Live and
log status if no failure has occurred. Keep the same B boot and hooks off.

Source check: screen_memory.cpp uses a 30000 ms debounce and writes Preferences
only when the pending selected screen differs from the saved one. Keep ScreenMem
messages to distinguish an actual NVS save from an already-saved selection.
This is normal screen and save behavior, not the later controlled NVS overlap
stress. Test inclinometer separately after review; defer Latest-to-Live.
No firmware changes are needed for the screen test.


## G-meter selection, NVS saves and Live - 2026-09-16 07:52

JP reports that the G-meter dot followed movement properly and video was normal.
This is still B, boot 13, hooks=0, with valid PSRAM stack placement and an internal
TCB. The following Live cycle is run 5 of this boot.

Observed sequence:
- 07:52:03.392: G-meter selected; UI objects created and screen-save timer armed.
- 07:52:33.407: screen ID 2 actually saved to NVS.
- 07:52:48.478: G-meter objects cleaned up; dashboard selected.
- 07:53:05: dashboard log status captured before its save completed.
- 07:53:18.486: screen ID 1 actually saved to NVS.
- 07:53:20.539: Live started after both saves.
- 07:54:20.959: return to dashboard without scheduling another preference save.

The dashboard dwell was about 32 seconds rather than the requested 35, but its
save is explicitly confirmed before Live, so no repeat is needed for that detail.
Screen-memory IDs use their own enum: ID 2 is G-meter (ui_Screen3), ID 1 is dashboard.
This was not an image-viewer preference save.

| Measurement | Result |
|-------------|--------|
| Normal-window largest_min around screen changes | 31732 bytes in runs 12, 13 and 14 |
| Live TLS first / second largest_min | 26612 / 27636 bytes |
| TLS durations / periodic samples | 631 ms / 63; 654 ms / 66 |
| TLS maximum sampler gaps | 10598 / 10191 us |
| Full Live largest_min | 26612 bytes |
| Full Live duration / samples / maximum sampler gap | 60410 ms / 6041 / 11161 us |
| Requested sampler interval / state | 10 ms / on |
| Video | 194 frames in 60.4 seconds, 3.2 fps |
| First frame / maximum frame gap | 1141 / 958 ms |
| HTTP / decode / blit / mean frame | 304 / 60 / 62 / 311 ms |
| Frame size / transfer rate | 18.6 KB / 107 KB per second |
| Video-end free heap / PSRAM | 46812 / 7610568 bytes |
| Writer stack used / margin | 4204 / 3988 bytes, unchanged |
| Historical internal_largest | 25588 bytes, unchanged throughout |
| heap_min_boot | 34840 bytes, unchanged throughout |
| Logger state / errors / drops | ready / none / zero throughout |
| Writes / file size | 20 to 22 / 514107 to 515231 bytes |

This sequence PASSES the 20480-byte memory check with 6132 bytes of margin in
the lowest Live window. No allocation/TLS error, reset or logger error is reported.
There were no suppressed or truncated records, and queue high water remained 1.
Slow writes stayed zero; maximum write 2809 us, flush 6114 us and SD operation
169424 us were unchanged. No rotation or pruning occurred.

The preceding normal probes report IMU minimum/average pairs of 7.52/49.11,
10.31/48.32 and 37.02/49.25 Hz. Record the isolated lower readings and the
48.32 Hz window rather than claiming unchanged IMU performance. These windows
span screen and motion activity; the capture does not isolate the cause of
each minimum. The ordinary screen-save test does not replace the later
controlled NVS/SD overlap stress or the same-session IMU performance comparison.

A partial merged normal-probe fragment appeared immediately after console
reconnection. Do not reconstruct missing run-10 values from it. Subsequent
complete probe and status lines support the conclusions above.

Decision: G-meter behavior, its ordinary NVS save, dashboard save and following
Live passed in this capture. This does not reproduce or explain the earlier
overnight failure, so Stage 1 remains unaccepted and hooks stress stays paused.
Next single test: same B boot, log status, inclinometer for at least 35 seconds
with gentle tilts, log status, dashboard for at least 35 seconds, log status,
then one full Live cycle and log status. Capture screen-ID-3 and ID-1 saves.
Stop on a sub-20480 probe or functional error before proceeding.

Documentation only; no firmware changes, builds, commits or pushes. The local B
switch remains unchanged.

### Captured evidence

```text
2026-09-16 07:48:20.564 EVENT Console cleared.
2026-09-16 07:51:46.572 EVENT Connect requested. After open: DTR=true, RTS=false.
2026-09-16 07:51:50.168 EVENT Explicit signals applied. After open: DTR=true, RTS=false.
2026-09-16 07:51:50.168 EVENT Connected. Send status; compare with the previous reading if available.
2026-09-16 07:51:50.168 RX PROBE] window=normal run=10 ms=60001 heap_min_boot=34840 largest_[PROBE] window=normal run=11 ms=60000 heap_min_boot=34840 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10800 scan_max_us=433 timer=on imu_n=2950 imu_min_hz=32.20 imu_avg_hz=49.25
2026-09-16 07:51:58.604 TX log status [CRLF]
2026-09-16 07:51:58.610 RX [LOG] state=ready boot=13 session=boot-13 up_ms=807244 clock=synced setup=1 hooks=0 file_bytes=514107 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922561024 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 07:51:58.610 RX [LOG] measured=1 stack_min=3988 internal_min=34840 internal_largest=25588 dma_min=27344 dma_largest=25588 writes=20 slow=0 write_max_us=2809 flush_max_us=6114 sd_max_us=169424 rotations=0 pruned=0 oversized=0
2026-09-16 07:51:58.611 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 07:51:58.612 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 07:51:58.612 RX [LOG MEM] phase=before_clock up_us=1781915 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 07:51:58.612 RX [LOG MEM] phase=after_clock up_us=1782603 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:51:58.612 RX [LOG MEM] phase=before_writer up_us=1782651 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:51:58.612 RX [LOG MEM] phase=writer_entry up_us=1782857 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:51:58.612 RX [LOG MEM] phase=after_formatter up_us=1782938 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:51:58.613 RX [LOG MEM] phase=before_mount up_us=1783508 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:51:58.613 RX [LOG MEM] phase=after_mount up_us=1952818 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:51:58.613 RX [LOG MEM] phase=before_current_open up_us=1960112 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:51:58.613 RX [LOG MEM] phase=after_current_open up_us=1961500 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:51:58.613 RX [LOG MEM] phase=storage_done up_us=1975173 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:52:03.392 RX G-meter button clicked
2026-09-16 07:52:03.392 RX Creating G-meter UI elements...
2026-09-16 07:52:03.400 RX G-meter display elements created
2026-09-16 07:52:03.400 RX Free heap: 90428 bytes, Free PSRAM: 7610924 bytes
2026-09-16 07:52:03.400 RX [ScreenMem] Screen 2 selected, save scheduled in 30 seconds
2026-09-16 07:52:03.400 RX Screen touched, resetting inactivity timer.
2026-09-16 07:52:05.114 RX Movement Detected! (Accel: 0.00, Gyro: 6.44)
2026-09-16 07:52:05.115 RX TX motion MQTT: Moving (immediate)
2026-09-16 07:52:12.638 RX [PROBE] window=normal run=12 ms=60004 heap_min_boot=34840 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10673 scan_max_us=669 timer=on imu_n=2935 imu_min_hz=7.52 imu_avg_hz=49.11
2026-09-16 07:52:33.407 RX [ScreenMem] Saved screen ID 2 to NVS
2026-09-16 07:52:35.118 RX TX motion MQTT: Moving (periodic)
2026-09-16 07:52:40.428 TX log status [CRLF]
2026-09-16 07:52:40.429 RX [LOG] state=ready boot=13 session=boot-13 up_ms=849065 clock=synced setup=1 hooks=0 file_bytes=514669 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922561024 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 07:52:40.430 RX [LOG] measured=1 stack_min=3988 internal_min=34840 internal_largest=25588 dma_min=27344 dma_largest=25588 writes=21 slow=0 write_max_us=2809 flush_max_us=6114 sd_max_us=169424 rotations=0 pruned=0 oversized=0
2026-09-16 07:52:40.431 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 07:52:40.431 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 07:52:40.431 RX [LOG MEM] phase=before_clock up_us=1781915 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 07:52:40.431 RX [LOG MEM] phase=after_clock up_us=1782603 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:52:40.432 RX [LOG MEM] phase=before_writer up_us=1782651 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:52:40.433 RX [LOG MEM] phase=writer_entry up_us=1782857 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:52:40.433 RX [LOG MEM] phase=after_formatter up_us=1782938 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:52:40.433 RX [LOG MEM] phase=before_mount up_us=1783508 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:52:40.433 RX [LOG MEM] phase=after_mount up_us=1952818 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:52:40.433 RX [LOG MEM] phase=before_current_open up_us=1960112 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:52:40.433 RX [LOG MEM] phase=after_current_open up_us=1961500 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:52:40.433 RX [LOG MEM] phase=storage_done up_us=1975173 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:52:48.478 RX Screen 3 Unloading: Cleaning up G-meter objects and resetting pointers
2026-09-16 07:52:48.479 RX [ScreenMem] Screen 1 selected, save scheduled in 30 seconds
2026-09-16 07:52:48.479 RX Screen touched, resetting inactivity timer.
2026-09-16 07:52:50.448 RX Movement Stopped.
2026-09-16 07:53:05.133 TX log status [CRLF]
2026-09-16 07:53:05.138 RX [LOG] state=ready boot=13 session=boot-13 up_ms=873765 clock=synced setup=1 hooks=0 file_bytes=514669 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922561024 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 07:53:05.140 RX [LOG] measured=1 stack_min=3988 internal_min=34840 internal_largest=25588 dma_min=27344 dma_largest=25588 writes=21 slow=0 write_max_us=2809 flush_max_us=6114 sd_max_us=169424 rotations=0 pruned=0 oversized=0
2026-09-16 07:53:05.140 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 07:53:05.142 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 07:53:05.142 RX [LOG MEM] phase=before_clock up_us=1781915 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 07:53:05.142 RX [LOG MEM] phase=after_clock up_us=1782603 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:53:05.142 RX [LOG MEM] phase=before_writer up_us=1782651 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:53:05.142 RX [LOG MEM] phase=writer_entry up_us=1782857 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:53:05.142 RX [LOG MEM] phase=after_formatter up_us=1782938 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:53:05.142 RX [LOG MEM] phase=before_mount up_us=1783508 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:53:05.142 RX [LOG MEM] phase=after_mount up_us=1952818 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:53:05.142 RX [LOG MEM] phase=before_current_open up_us=1960112 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:53:05.143 RX [LOG MEM] phase=after_current_open up_us=1961500 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:53:05.143 RX [LOG MEM] phase=storage_done up_us=1975173 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:53:18.486 RX [PROBE] window=normal run=13 ms=60002 heap_min_boot=34840 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=11015 scan_max_us=1143 timer=on imu_n=2886 imu_min_hz=10.31 imu_avg_hz=48.32
2026-09-16 07:53:18.486 RX [ScreenMem] Saved screen ID 1 to NVS
2026-09-16 07:53:20.539 RX Screen touched, resetting inactivity timer.
2026-09-16 07:53:20.539 RX Live button clicked -> starting live feed
2026-09-16 07:53:20.540 RX [PROBE] window=normal run=14 ms=7890 heap_min_boot=34840 largest_min=31732 interval_ms=10 samples=789 gap_max_us=10233 scan_max_us=289 timer=on imu_n=387 imu_min_hz=37.02 imu_avg_hz=49.25
2026-09-16 07:53:20.540 RX Screen 2 Loaded.
2026-09-16 07:53:20.611 RX Video: endpoint parsed, port 9835, path /esp32/live
2026-09-16 07:53:21.243 RX [PROBE] window=live_tls run=9 ms=631 heap_min_boot=34840 largest_min=26612 interval_ms=10 samples=63 gap_max_us=10598 scan_max_us=150 timer=on
2026-09-16 07:53:52.751 RX [PROBE] window=live_tls run=10 ms=654 heap_min_boot=34840 largest_min=27636 interval_ms=10 samples=66 gap_max_us=10191 scan_max_us=159 timer=on
2026-09-16 07:54:20.957 RX Video: 194 frames in 60.4s (3.2 fps) | http 304 | decode 60 | blit 62 | frame 311 ms | first_frame 1141 | max_gap 958 ms
2026-09-16 07:54:20.957 RX Video: http = ttfb 130 + xfer 173 ms | frame 18.6 KB | 107 KB/s while transferring
2026-09-16 07:54:20.957 RX Video: free PSRAM 7610568, free heap 46812
2026-09-16 07:54:20.958 RX [PROBE] window=live run=5 ms=60410 heap_min_boot=34840 largest_min=26612 interval_ms=10 samples=6041 gap_max_us=11161 scan_max_us=1023 timer=on
2026-09-16 07:54:20.959 RX Video: returning to previous screen
2026-09-16 07:54:20.959 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 07:54:20.959 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 07:54:26.396 TX log status [CRLF]
2026-09-16 07:54:26.400 RX [LOG] state=ready boot=13 session=boot-13 up_ms=955019 clock=synced setup=1 hooks=0 file_bytes=515231 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922561024 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 07:54:26.401 RX [LOG] measured=1 stack_min=3988 internal_min=34840 internal_largest=25588 dma_min=27344 dma_largest=25588 writes=22 slow=0 write_max_us=2809 flush_max_us=6114 sd_max_us=169424 rotations=0 pruned=0 oversized=0
2026-09-16 07:54:26.402 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 07:54:26.402 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 07:54:26.402 RX [LOG MEM] phase=before_clock up_us=1781915 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 07:54:26.402 RX [LOG MEM] phase=after_clock up_us=1782603 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:54:26.403 RX [LOG MEM] phase=before_writer up_us=1782651 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:54:26.403 RX [LOG MEM] phase=writer_entry up_us=1782857 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:54:26.403 RX [LOG MEM] phase=after_formatter up_us=1782938 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:54:26.403 RX [LOG MEM] phase=before_mount up_us=1783508 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:54:26.403 RX [LOG MEM] phase=after_mount up_us=1952818 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:54:26.403 RX [LOG MEM] phase=before_current_open up_us=1960112 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:54:26.403 RX [LOG MEM] phase=after_current_open up_us=1961500 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:54:26.403 RX [LOG MEM] phase=storage_done up_us=1975173 free=129996 largest=65524 heap_min_boot=128864
```


## Inclinometer navigation, NVS saves and Live - 2026-09-16 07:57

JP reports normal inclinometer and video behavior. The capture stays on boot 13,
B with hooks=0 and valid PSRAM stack/internal TCB placement. This is Live run 6.

Screen sequence:
- Brief G-meter entry at 07:57:49, with UI creation and pending ID-2 save.
- At 07:57:50, G-meter cleanup, screen-load UI-reset message and selection of ID 3.
- 07:58:16 status preceded the save; the additional status at 07:58:25 followed it.
- 07:58:20.193: saved screen ID 3 to NVS, confirming the inclinometer preference.
- Brief G-meter transit at 07:58:27, then dashboard selected.
- 07:58:58.013: saved ID 1 to NVS; dashboard status and Live followed.
- Return from Live reported no preference save needed.

The brief G-meter visits did not last for the 30-second debounce; no ID-2 save
is reported in this capture. The screen-memory enum identifies ID 3 as
inclinometer, independent of the text "Calibration Screen Loaded: Resetting UI
elements". That message describes resetting calibration UI elements and does
not establish that a calibration operation was performed.

| Measurement | Result |
|-------------|--------|
| Normal-window largest_min | 31732 bytes in runs 18 and 19 |
| Normal IMU minimum / average | 7.52 / 48.79 Hz; 7.42 / 49.14 Hz |
| TLS first window | 784 ms, largest_min=27636, 79 samples |
| TLS second window | 699 ms, largest_min=25588, 70 samples |
| TLS maximum sample gaps | 10211 and 10053 us |
| Full Live probe | 60216 ms, largest_min=25588, 6022 samples |
| Full Live maximum sample gap / scan | 11051 / 1324 us |
| Sampler interval / state | 10 ms / on |
| Video | 191 frames in 60.2 seconds, 3.2 fps |
| First frame / maximum gap | 1243 / 1015 ms |
| HTTP / decode / blit / mean frame | 306 / 60 / 62 / 315 ms |
| Frame size / transfer rate | 18.8 KB / 108 KB per second |
| Video-end free heap / PSRAM | 46812 / 7610560 bytes |
| Historical heap_min_boot / internal_largest | 34840 / 25588 bytes, unchanged |
| Writer stack used / margin | 4204 / 3988 bytes, unchanged |
| Logger state / errors / drops | ready / none / zero throughout |
| Writes / file size | 26 to 28 / 517485 to 518612 bytes |

Result: this screen-route and following Live memory check PASSES. The minimum
25588 bytes is 5108 above the unchanged 20480-byte floor. Both intended NVS saves
are explicitly confirmed. No logger or TLS/allocation error or reset is reported.
Queue high water stayed 1/16, with no suppressed or truncated records.
Slow writes remained zero; maximum write 2809 us, flush 6114 us and SD operation
169424 us were unchanged. No rotation or pruning occurred.
Motion MQTT transmissions occurred normally, including during Live.

Record the lower individual IMU readings without attributing them to a particular
operation. These active-screen windows do not complete the controlled
same-session logging-on/off IMU performance comparison.

Six complete Live cycles in boot 13 now pass, including cycles after G-meter and
inclinometer navigation and ordinary preference saves. The low-headroom overnight
condition has not recurred. This does not explain the earlier failure or justify
Stage 1 acceptance; controlled stress and other gates remain pending.

Next: same B boot, hooks off, log status, one Latest request, return to dashboard,
log status, then one full Live cycle and log status. Capture all media probes and
timings. Stop on a sub-20480 window or functional error before the next operation.
This resumes the deferred media-order test, because the previous overnight boot
included Latest while the supplied boot-13 sequence so far did not.
No assertion is made that Latest is the cause.

Documentation only in this turn. No firmware changes, builds, commits or pushes.
JP's local B setting is preserved.

### Captured evidence

```text
2026-09-16 07:57:40.674 EVENT Console cleared.
2026-09-16 07:57:46.861 TX log status [CRLF]
2026-09-16 07:57:46.869 RX [LOG] state=ready boot=13 session=boot-13 up_ms=1155461 clock=synced setup=1 hooks=0 file_bytes=517485 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922561024 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 07:57:46.870 RX [LOG] measured=1 stack_min=3988 internal_min=34840 internal_largest=25588 dma_min=27344 dma_largest=25588 writes=26 slow=0 write_max_us=2809 flush_max_us=6114 sd_max_us=169424 rotations=0 pruned=0 oversized=0
2026-09-16 07:57:46.870 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 07:57:46.871 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 07:57:46.871 RX [LOG MEM] phase=before_clock up_us=1781915 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 07:57:46.871 RX [LOG MEM] phase=after_clock up_us=1782603 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:57:46.872 RX [LOG MEM] phase=before_writer up_us=1782651 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:57:46.872 RX [LOG MEM] phase=writer_entry up_us=1782857 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:57:46.872 RX [LOG MEM] phase=after_formatter up_us=1782938 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:57:46.872 RX [LOG MEM] phase=before_mount up_us=1783508 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:57:46.872 RX [LOG MEM] phase=after_mount up_us=1952818 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:57:46.872 RX [LOG MEM] phase=before_current_open up_us=1960112 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:57:46.872 RX [LOG MEM] phase=after_current_open up_us=1961500 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:57:46.872 RX [LOG MEM] phase=storage_done up_us=1975173 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:57:49.409 RX G-meter button clicked
2026-09-16 07:57:49.409 RX Creating G-meter UI elements...
2026-09-16 07:57:49.417 RX G-meter display elements created
2026-09-16 07:57:49.417 RX Free heap: 90428 bytes, Free PSRAM: 7610924 bytes
2026-09-16 07:57:49.417 RX [ScreenMem] Screen 2 selected, save scheduled in 30 seconds
2026-09-16 07:57:49.417 RX Screen touched, resetting inactivity timer.
2026-09-16 07:57:50.189 RX Screen 3 Unloading: Cleaning up G-meter objects and resetting pointers
2026-09-16 07:57:50.190 RX Calibration Screen Loaded: Resetting UI elements
2026-09-16 07:57:50.190 RX [ScreenMem] Screen 3 selected, save scheduled in 30 seconds
2026-09-16 07:57:50.190 RX Screen touched, resetting inactivity timer.
2026-09-16 07:57:52.149 RX Movement Detected! (Accel: 0.01, Gyro: 31.20)
2026-09-16 07:57:52.150 RX TX motion MQTT: Moving (immediate)
2026-09-16 07:58:16.027 TX log status [CRLF]
2026-09-16 07:58:16.029 RX [LOG] state=ready boot=13 session=boot-13 up_ms=1184622 clock=synced setup=1 hooks=0 file_bytes=517485 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922561024 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 07:58:16.030 RX [LOG] measured=1 stack_min=3988 internal_min=34840 internal_largest=25588 dma_min=27344 dma_largest=25588 writes=26 slow=0 write_max_us=2809 flush_max_us=6114 sd_max_us=169424 rotations=0 pruned=0 oversized=0
2026-09-16 07:58:16.031 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 07:58:16.032 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 07:58:16.032 RX [LOG MEM] phase=before_clock up_us=1781915 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 07:58:16.032 RX [LOG MEM] phase=after_clock up_us=1782603 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:58:16.032 RX [LOG MEM] phase=before_writer up_us=1782651 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:58:16.032 RX [LOG MEM] phase=writer_entry up_us=1782857 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:58:16.033 RX [LOG MEM] phase=after_formatter up_us=1782938 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:58:16.033 RX [LOG MEM] phase=before_mount up_us=1783508 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:58:16.033 RX [LOG MEM] phase=after_mount up_us=1952818 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:58:16.033 RX [LOG MEM] phase=before_current_open up_us=1960112 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:58:16.033 RX [LOG MEM] phase=after_current_open up_us=1961500 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:58:16.033 RX [LOG MEM] phase=storage_done up_us=1975173 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:58:20.193 RX [ScreenMem] Saved screen ID 3 to NVS
2026-09-16 07:58:21.097 RX [PROBE] window=normal run=18 ms=60001 heap_min_boot=34840 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10735 scan_max_us=425 timer=on imu_n=2892 imu_min_hz=7.52 imu_avg_hz=48.79
2026-09-16 07:58:22.153 RX TX motion MQTT: Moving (periodic)
2026-09-16 07:58:25.333 TX log status [CRLF]
2026-09-16 07:58:25.335 RX [LOG] state=ready boot=13 session=boot-13 up_ms=1193929 clock=synced setup=1 hooks=0 file_bytes=517485 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922561024 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 07:58:25.337 RX [LOG] measured=1 stack_min=3988 internal_min=34840 internal_largest=25588 dma_min=27344 dma_largest=25588 writes=26 slow=0 write_max_us=2809 flush_max_us=6114 sd_max_us=169424 rotations=0 pruned=0 oversized=0
2026-09-16 07:58:25.337 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 07:58:25.338 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 07:58:25.338 RX [LOG MEM] phase=before_clock up_us=1781915 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 07:58:25.339 RX [LOG MEM] phase=after_clock up_us=1782603 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:58:25.339 RX [LOG MEM] phase=before_writer up_us=1782651 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:58:25.339 RX [LOG MEM] phase=writer_entry up_us=1782857 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:58:25.339 RX [LOG MEM] phase=after_formatter up_us=1782938 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:58:25.339 RX [LOG MEM] phase=before_mount up_us=1783508 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:58:25.339 RX [LOG MEM] phase=after_mount up_us=1952818 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:58:25.339 RX [LOG MEM] phase=before_current_open up_us=1960112 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:58:25.339 RX [LOG MEM] phase=after_current_open up_us=1961500 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:58:25.339 RX [LOG MEM] phase=storage_done up_us=1975173 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:58:27.364 RX Screen touched, resetting inactivity timer.
2026-09-16 07:58:27.364 RX G-meter button clicked
2026-09-16 07:58:27.364 RX Creating G-meter UI elements...
2026-09-16 07:58:27.372 RX G-meter display elements created
2026-09-16 07:58:27.372 RX Free heap: 90428 bytes, Free PSRAM: 7610924 bytes
2026-09-16 07:58:27.372 RX [ScreenMem] Screen 2 selected, save scheduled in 30 seconds
2026-09-16 07:58:27.996 RX Screen 3 Unloading: Cleaning up G-meter objects and resetting pointers
2026-09-16 07:58:27.997 RX [ScreenMem] Screen 1 selected, save scheduled in 30 seconds
2026-09-16 07:58:27.997 RX Screen touched, resetting inactivity timer.
2026-09-16 07:58:33.241 RX Movement Stopped.
2026-09-16 07:58:58.013 RX [ScreenMem] Saved screen ID 1 to NVS
2026-09-16 07:59:02.085 TX log status [CRLF]
2026-09-16 07:59:02.086 RX [LOG] state=ready boot=13 session=boot-13 up_ms=1230668 clock=synced setup=1 hooks=0 file_bytes=518049 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922561024 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 07:59:02.086 RX [LOG] measured=1 stack_min=3988 internal_min=34840 internal_largest=25588 dma_min=27344 dma_largest=25588 writes=27 slow=0 write_max_us=2809 flush_max_us=6114 sd_max_us=169424 rotations=0 pruned=0 oversized=0
2026-09-16 07:59:02.088 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 07:59:02.088 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 07:59:02.089 RX [LOG MEM] phase=before_clock up_us=1781915 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 07:59:02.089 RX [LOG MEM] phase=after_clock up_us=1782603 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:59:02.089 RX [LOG MEM] phase=before_writer up_us=1782651 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:59:02.090 RX [LOG MEM] phase=writer_entry up_us=1782857 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:59:02.090 RX [LOG MEM] phase=after_formatter up_us=1782938 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:59:02.091 RX [LOG MEM] phase=before_mount up_us=1783508 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 07:59:02.091 RX [LOG MEM] phase=after_mount up_us=1952818 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:59:02.091 RX [LOG MEM] phase=before_current_open up_us=1960112 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:59:02.091 RX [LOG MEM] phase=after_current_open up_us=1961500 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:59:02.091 RX [LOG MEM] phase=storage_done up_us=1975173 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 07:59:04.939 RX Screen touched, resetting inactivity timer.
2026-09-16 07:59:04.940 RX Live button clicked -> starting live feed
2026-09-16 07:59:04.940 RX [PROBE] window=normal run=19 ms=43831 heap_min_boot=34840 largest_min=31732 interval_ms=10 samples=4383 gap_max_us=10818 scan_max_us=448 timer=on imu_n=2138 imu_min_hz=7.42 imu_avg_hz=49.14
2026-09-16 07:59:04.940 RX Screen 2 Loaded.
2026-09-16 07:59:05.011 RX Video: endpoint parsed, port 9835, path /esp32/live
2026-09-16 07:59:05.797 RX [PROBE] window=live_tls run=11 ms=784 heap_min_boot=34840 largest_min=27636 interval_ms=10 samples=79 gap_max_us=10211 scan_max_us=156 timer=on
2026-09-16 07:59:08.964 RX Movement Detected! (Accel: 0.00, Gyro: 15.19)
2026-09-16 07:59:08.965 RX TX motion MQTT: Moving (immediate)
2026-09-16 07:59:37.252 RX [PROBE] window=live_tls run=12 ms=699 heap_min_boot=34840 largest_min=25588 interval_ms=10 samples=70 gap_max_us=10053 scan_max_us=128 timer=on
2026-09-16 07:59:38.967 RX TX motion MQTT: Moving (periodic)
2026-09-16 07:59:39.038 RX Movement Stopped.
2026-09-16 08:00:05.163 RX Video: 191 frames in 60.2s (3.2 fps) | http 306 | decode 60 | blit 62 | frame 315 ms | first_frame 1243 | max_gap 1015 ms
2026-09-16 08:00:05.164 RX Video: http = ttfb 131 + xfer 175 ms | frame 18.8 KB | 108 KB/s while transferring
2026-09-16 08:00:05.164 RX Video: free PSRAM 7610560, free heap 46812
2026-09-16 08:00:05.165 RX [PROBE] window=live run=6 ms=60216 heap_min_boot=34840 largest_min=25588 interval_ms=10 samples=6022 gap_max_us=11051 scan_max_us=1324 timer=on
2026-09-16 08:00:05.165 RX Video: returning to previous screen
2026-09-16 08:00:05.166 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 08:00:05.166 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 08:00:08.331 TX log status [CRLF]
2026-09-16 08:00:08.337 RX [LOG] state=ready boot=13 session=boot-13 up_ms=1296912 clock=synced setup=1 hooks=0 file_bytes=518612 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922561024 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 08:00:08.338 RX [LOG] measured=1 stack_min=3988 internal_min=34840 internal_largest=25588 dma_min=27344 dma_largest=25588 writes=28 slow=0 write_max_us=2809 flush_max_us=6114 sd_max_us=169424 rotations=0 pruned=0 oversized=0
2026-09-16 08:00:08.339 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 08:00:08.339 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 08:00:08.340 RX [LOG MEM] phase=before_clock up_us=1781915 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 08:00:08.340 RX [LOG MEM] phase=after_clock up_us=1782603 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:00:08.340 RX [LOG MEM] phase=before_writer up_us=1782651 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:00:08.341 RX [LOG MEM] phase=writer_entry up_us=1782857 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:00:08.341 RX [LOG MEM] phase=after_formatter up_us=1782938 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:00:08.341 RX [LOG MEM] phase=before_mount up_us=1783508 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:00:08.341 RX [LOG MEM] phase=after_mount up_us=1952818 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:00:08.341 RX [LOG MEM] phase=before_current_open up_us=1960112 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:00:08.341 RX [LOG MEM] phase=after_current_open up_us=1961500 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:00:08.341 RX [LOG MEM] phase=storage_done up_us=1975173 free=129996 largest=65524 heap_min_boot=128864
```


## Latest followed by Live - 2026-09-16 08:02

JP reports normal operation. B remains on boot 13 with hooks=0, valid PSRAM stack
placement and internal TCB. The capture includes the first image_https window
and seventh complete Live window in this boot.

| Measurement | Result |
|-------------|--------|
| Latest HTTPS | 770 ms, largest_min=26612, 77 samples |
| Latest HTTPS maximum sample gap / scan | 10141 / 137 us |
| Image size / total display time | 35085 bytes / 1343 ms |
| Response / download completion | 771 ms / 1204 ms since button press |
| Normal window before Latest | largest_min=31732, 4914 samples |
| Normal window between HTTPS and Live | largest_min=30708, 1091 samples |
| Live TLS run 13 | 642 ms, largest_min=26612, 64 samples |
| Live TLS maximum sample gap / scan | 11200 / 947 us |
| Full Live probe | 60407 ms, largest_min=26612, 6041 samples |
| Full Live maximum sample gap / scan | 11200 / 1196 us |
| Sampler interval / state | 10 ms / on |
| Video | 197 frames in 60.4 seconds, 3.3 fps |
| First frame / maximum gap | 565 / 957 ms |
| HTTP / decode / blit / mean frame | 302 / 60 / 62 / 307 ms |
| Frame size / transfer rate | 18.9 KB / 110 KB per second |
| Video-end free heap / PSRAM | 46544 / 7610560 bytes |
| Historical heap_min_boot / internal_largest | 34840 / 25588, unchanged |
| Writer stack used / margin | 4204 / 3988, unchanged |
| Logger state / errors / drops | ready / none / zero throughout |
| Writes / file size | 31 to 32 / 520304 to 520868 bytes |

Result: Latest and following Live both PASS the unchanged 20480-byte floor,
with 6132 bytes margin. Both return to the dashboard without a preference save.
No reported reset, allocation/TLS failure or logger error occurred.
Queue high water stayed 1/16; slow writes, suppression and truncation remained zero.
Write, flush and SD-operation maxima stayed 2809, 6114 and 169424 us.

Only one actual Live TLS-connect probe appears, during the feed. Do not treat
its absence at initial Live start as a sampling failure: the window instruments
actual connect calls, and the still and Live paths share the secure client.
This capture alone does not establish a new handshake at Live start.
The short first-frame time is recorded without claiming a code performance gain.

The normal IMU minimum/average values were 37.36/49.25 and 10.42/49.03 Hz.
The second normal probe starts after HTTPS headers, so it includes remaining
image processing as well as the return to the dashboard. Its 30708-byte minimum
does not by itself establish persistent loss of idle headroom.

Seven full Live cycles in this boot have now passed, covering repeats, G-meter,
inclinometer, ordinary preference saves and Latest-to-Live. This morning's
sequences have not reproduced the prior boot's 14324-byte overnight result.
The historical memory minima have stayed at 34840/25588 since earlier tests.
These observations do not resolve the overnight cause or pass the full Stage 1 gate.

Next step: use existing diagnostics before prescribing more button tests.
Request the complete /logs/current.log from the card through a computer reader,
after normal board shutdown. Copy it without deleting the original.
The final status shows generation 1, zero archives, rotations and pruning;
boot-12 overnight records should still be in that file, to be verified.
The current content size is about 509 KiB (520868 bytes).

Analysis should locate the first recorded overnight reduction using boot and
uptime plus minute HEALTH records. Compare instantaneous internal_free,
internal_largest, psram_free, and Wi-Fi/MQTT/screen/operation fields with
snapshot_age_ms. File HEALTH internal_largest is queried when composing that
record, unlike the historical minimum in serial log status. The minute cadence
and mixed capture times limit causal attribution; transient allocations may be
missed. Use observed changes to select a targeted follow-up.

Stage 1 acceptance and hooks stress remain on hold. No firmware edit, build,
flash, commit or push occurred in this turn. Local B configuration is preserved.

### Captured evidence

The HTTPS URL line is omitted; other serial values are unchanged.

```text
2026-09-16 08:02:39.710 EVENT Console cleared.
2026-09-16 08:02:52.145 TX log status [CRLF]
2026-09-16 08:02:52.149 RX [LOG] state=ready boot=13 session=boot-13 up_ms=1460710 clock=synced setup=1 hooks=0 file_bytes=520304 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922561024 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 08:02:52.149 RX [LOG] measured=1 stack_min=3988 internal_min=34840 internal_largest=25588 dma_min=27344 dma_largest=25588 writes=31 slow=0 write_max_us=2809 flush_max_us=6114 sd_max_us=169424 rotations=0 pruned=0 oversized=0
2026-09-16 08:02:52.150 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 08:02:52.151 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 08:02:52.151 RX [LOG MEM] phase=before_clock up_us=1781915 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 08:02:52.151 RX [LOG MEM] phase=after_clock up_us=1782603 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:02:52.151 RX [LOG MEM] phase=before_writer up_us=1782651 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:02:52.152 RX [LOG MEM] phase=writer_entry up_us=1782857 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:02:52.152 RX [LOG MEM] phase=after_formatter up_us=1782938 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:02:52.152 RX [LOG MEM] phase=before_mount up_us=1783508 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:02:52.152 RX [LOG MEM] phase=after_mount up_us=1952818 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:02:52.152 RX [LOG MEM] phase=before_current_open up_us=1960112 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:02:52.152 RX [LOG MEM] phase=after_current_open up_us=1961500 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:02:52.152 RX [LOG MEM] phase=storage_done up_us=1975173 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:02:54.254 RX Screen touched, resetting inactivity timer.
2026-09-16 08:02:54.254 RX Latest button clicked
2026-09-16 08:02:54.254 RX Initiating async latest image request...
2026-09-16 08:02:54.254 RX Preparing UI for new image request...
2026-09-16 08:02:54.254 RX Cleaning up image fetcher state...
2026-09-16 08:02:54.254 RX Screen 2 Loaded.
2026-09-16 08:02:54.326 RX === requestImage('latest') START ===
2026-09-16 08:02:54.327 RX Sending HTTP GET...
2026-09-16 08:02:54.327 RX [PROBE] window=normal run=22 ms=49139 heap_min_boot=34840 largest_min=31732 interval_ms=10 samples=4914 gap_max_us=10981 scan_max_us=1072 timer=on imu_n=2412 imu_min_hz=37.36 imu_avg_hz=49.25
2026-09-16 08:02:55.098 RX [PROBE] window=image_https run=1 ms=770 heap_min_boot=34840 largest_min=26612 interval_ms=10 samples=77 gap_max_us=10141 scan_max_us=137 timer=on
2026-09-16 08:02:55.098 RX Response received in 771 ms, Content-Length: 35085
2026-09-16 08:02:55.098 RX Starting to receive image data...
2026-09-16 08:02:55.459 RX Image download complete (35085 bytes, 1204 ms since button press). Starting decode...
2026-09-16 08:02:55.597 RX JPEG decoded successfully into PSRAM.
2026-09-16 08:02:55.597 RX LVGL image source updated. Total 1343 ms from button press (budget 20000 ms).
2026-09-16 08:02:58.943 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 08:02:58.943 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 08:03:00.032 TX log status [CRLF]
2026-09-16 08:03:00.033 RX [LOG] state=ready boot=13 session=boot-13 up_ms=1468595 clock=synced setup=1 hooks=0 file_bytes=520304 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922561024 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 08:03:00.035 RX [LOG] measured=1 stack_min=3988 internal_min=34840 internal_largest=25588 dma_min=27344 dma_largest=25588 writes=31 slow=0 write_max_us=2809 flush_max_us=6114 sd_max_us=169424 rotations=0 pruned=0 oversized=0
2026-09-16 08:03:00.035 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 08:03:00.036 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 08:03:00.036 RX [LOG MEM] phase=before_clock up_us=1781915 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 08:03:00.036 RX [LOG MEM] phase=after_clock up_us=1782603 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:03:00.036 RX [LOG MEM] phase=before_writer up_us=1782651 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:03:00.036 RX [LOG MEM] phase=writer_entry up_us=1782857 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:03:00.037 RX [LOG MEM] phase=after_formatter up_us=1782938 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:03:00.037 RX [LOG MEM] phase=before_mount up_us=1783508 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:03:00.037 RX [LOG MEM] phase=after_mount up_us=1952818 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:03:00.037 RX [LOG MEM] phase=before_current_open up_us=1960112 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:03:00.037 RX [LOG MEM] phase=after_current_open up_us=1961500 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:03:00.037 RX [LOG MEM] phase=storage_done up_us=1975173 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:03:06.510 RX Screen touched, resetting inactivity timer.
2026-09-16 08:03:06.510 RX Live button clicked -> starting live feed
2026-09-16 08:03:06.510 RX [PROBE] window=normal run=23 ms=10911 heap_min_boot=34840 largest_min=30708 interval_ms=10 samples=1091 gap_max_us=10288 scan_max_us=391 timer=on imu_n=528 imu_min_hz=10.42 imu_avg_hz=49.03
2026-09-16 08:03:06.511 RX Screen 2 Loaded.
2026-09-16 08:03:06.582 RX Video: endpoint parsed, port 9835, path /esp32/live
2026-09-16 08:03:36.850 RX [PROBE] window=live_tls run=13 ms=642 heap_min_boot=34840 largest_min=26612 interval_ms=10 samples=64 gap_max_us=11200 scan_max_us=947 timer=on
2026-09-16 08:04:06.922 RX Video: 197 frames in 60.4s (3.3 fps) | http 302 | decode 60 | blit 62 | frame 307 ms | first_frame 565 | max_gap 957 ms
2026-09-16 08:04:06.922 RX Video: http = ttfb 130 + xfer 172 ms | frame 18.9 KB | 110 KB/s while transferring
2026-09-16 08:04:06.922 RX Video: free PSRAM 7610560, free heap 46544
2026-09-16 08:04:06.923 RX [PROBE] window=live run=7 ms=60407 heap_min_boot=34840 largest_min=26612 interval_ms=10 samples=6041 gap_max_us=11200 scan_max_us=1196 timer=on
2026-09-16 08:04:06.924 RX Video: returning to previous screen
2026-09-16 08:04:06.924 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 08:04:06.924 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 08:04:09.967 TX log status [CRLF]
2026-09-16 08:04:09.974 RX [LOG] state=ready boot=13 session=boot-13 up_ms=1538531 clock=synced setup=1 hooks=0 file_bytes=520868 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922561024 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 08:04:09.975 RX [LOG] measured=1 stack_min=3988 internal_min=34840 internal_largest=25588 dma_min=27344 dma_largest=25588 writes=32 slow=0 write_max_us=2809 flush_max_us=6114 sd_max_us=169424 rotations=0 pruned=0 oversized=0
2026-09-16 08:04:09.976 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 08:04:09.977 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 08:04:09.977 RX [LOG MEM] phase=before_clock up_us=1781915 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 08:04:09.977 RX [LOG MEM] phase=after_clock up_us=1782603 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:04:09.978 RX [LOG MEM] phase=before_writer up_us=1782651 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:04:09.978 RX [LOG MEM] phase=writer_entry up_us=1782857 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:04:09.978 RX [LOG MEM] phase=after_formatter up_us=1782938 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:04:09.978 RX [LOG MEM] phase=before_mount up_us=1783508 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:04:09.978 RX [LOG MEM] phase=after_mount up_us=1952818 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:04:09.978 RX [LOG MEM] phase=before_current_open up_us=1960112 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:04:09.978 RX [LOG MEM] phase=after_current_open up_us=1961500 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:04:09.978 RX [LOG MEM] phase=storage_done up_us=1975173 free=129996 largest=65524 heap_min_boot=128864
```


## Supplied SD log analysis - 2026-09-16

JP supplied F:/current.log, 523255 bytes, through the card reader after shutdown.
The original bytes are preserved in
[bench_data/sd_current_2026-09-16_0807.txt](bench_data/sd_current_2026-09-16_0807.txt).
Full findings, source checks, hash and next-test procedure are in
[sd_diagnostics_overnight_analysis.md](sd_diagnostics_overnight_analysis.md).

The important correction to the initial interpretation is that there are two
separate observations. A historical low first appears between 07:07:15 and
07:08:15 in boot 12: internal_min 34964 to 22676 and dma_largest 26612 to 14324.
Both surrounding snapshots are offline, with current free 85348 and largest
32756, so the transient had recovered by the next minute snapshot.
At 07:34:15, Wi-Fi/MQTT are recorded connected and current largest is only 16372,
although current free is 46544, the same as before the outage. Live then sees
14324. Reconnection is a useful hypothesis, not the proven start or cause of
the first event.

All 551 consecutive offline snapshots (22:23 to 07:33) show current largest
32756. There is no steady decline in the sampled current free total.
Boot 12 has 706 HEALTH records, no reboot within the session, zero drops,
suppression, truncation and slow writes, and fixed 3988-byte writer stack margin.
Boots 12 and 13 both end with SESSION_END reason=shutdown pending=0.
Older BOOT records 2, 8, 10 and 12 report task_watchdog; their cause is unknown
and must not be assigned to logging or to the overnight interval without evidence.

JP was asked about activity around 07:07-07:08. Next targeted test: same B firmware,
card reinserted with board off, start, Latest and return, actual hotspot off about
90 seconds then on, then full Live if no failure. Capture status and probes at
each step. Do not substitute serial MQTT off/on. Keep the 20480 gate unchanged,
Stage 1 unaccepted and hooks stress paused. A short-outage pass cannot clear the
long-outage case. No firmware changes, build, flash, commit or push.


JP's recollection: he returned from a ride using the car companion, came downstairs
near the bench unit, and turned the iPhone hotspot back on to connect it. He does
not remember the exact time or specific activity around 07:07-07:08. Record the
hotspot recovery context without attributing that minute's transient to it.
The bench log does not describe the separate car unit's trip.


## Short hotspot outage after Latest reproduces low headroom - 2026-09-16 08:20

JP reports normal operation throughout. This is boot 14, same B variant,
hooks=0, ready logger, valid PSRAM stack placement and internal TCB.

| Checkpoint | Measurement |
|------------|-------------|
| Startup status | internal_min=84888, historical internal_largest=31732 |
| Latest HTTPS | 798 ms, largest_min=26612, 79 samples, max sample gap 10647 us |
| Latest image / total time | 35085 bytes / 1401 ms |
| After Latest | internal_min=34944, historical internal_largest=26612 |
| Offline notification | 08:21:12.992 |
| Normal window before reconnect | largest_min=32756, 39322 ms, 3932 samples |
| MQTT reconnect | 618 ms, largest_min=14324, 62 samples, max gap 10273 us |
| Green connected notification | 08:22:33.425 |
| Status after reconnect | internal_min=22656, historical internal_largest=14324 |
| Normal window before Live | largest_min=14324, 9227 ms, 923 samples |
| Live TLS run 1 | 704 ms, largest_min=14324, 70 samples, max gap 10325 us |
| Live TLS run 2 | 668 ms, largest_min=14324, 67 samples, max gap 10055 us |
| Full Live probe | 60341 ms, largest_min=14324, 6034 samples, max gap 11147 us |
| Video | 170 frames in 60.3 seconds, 2.8 fps |
| First frame / maximum gap | 1332 / 1043 ms |
| HTTP / decode / blit / mean frame | 346 / 60 / 61 / 355 ms |
| Frame size / transfer rate | 18.9 KB / 89 KB per second |
| Transfer / time to first byte | 212 / 133 ms |
| Video-end free heap / PSRAM | 46588 / 7610520 bytes |
| Stack used / minimum margin | 4204 / 3988, unchanged |
| Logger state / errors / drops | ready / none / zero |
| Writes / file size | 7 to 10 / 524389 to 526065 bytes |

The measured offline-to-connected notification interval is 80.433 seconds;
hotspot toggle times are not captured, so do not call it an exact 90-second
outage. This short sequence was sufficient to reproduce the observed symptom.

Result: memory gate FAIL, functional operation successful. The first low block
is directly observed in mqtt_connect run 2, before Live. 14324 is 6156 below the
unchanged 20480 floor. The 12288-byte fall in historical internal_min matches
the earlier overnight arithmetic, but does not identify a 12 KiB allocation.
Timer sampling was on at 10 ms throughout the measured windows.
The capture continued into Live despite the requested stop at a low reading;
retain that evidence without treating the continued operation as a passing gate.

No TLS/allocation failure, reset, logger error, suppression or truncation is
reported. Queue high water stayed 1/16, slow writes zero, stack watermark fixed.
Maximum write/flush/SD operation were 2767/9899/174391 us, unchanged across this
capture. No rotation or pruning occurred.
Normal IMU minimum/average pairs were 29.41/49.20, 10.10/48.91,
36.79/49.00 and 32.21/49.19 Hz.

Video fps was lower than earlier morning captures, alongside slower measured
transfer (212 ms at 89 KB/s, compared with 172 ms at 110 KB/s in the prior test).
Do not infer memory fragmentation caused the fps change. JP observed normal video.

Source follow-up: netCheckMqtt wraps the actual PubSubClient connect in the
probe and disconnects stale MQTT state first. It does not clean the image client.
image_fetcher.cpp calls HTTPClient::end at successful body completion; screen
unload frees image buffers without stopping its secure client.
Installed core 3.1.3 HTTPClient.h defaults _reuse=true; HTTPClient.cpp::end calls
disconnect(false), which preserves the client if reuse is permitted.
A retained image TLS connection is a plausible interaction to test, not a
verified allocation owner or a proven cleanup bug. No firmware was changed.

Next single control: normal shutdown and fresh B boot, hooks off, no Latest,
Live, Back or screen changes. After green MQTT, capture log status; turn the
actual hotspot off about 90 seconds, capture offline status, turn on and capture
green status plus mqtt_connect probes. Stop there without Live.
This tests whether MQTT reconnect alone is enough or prior media history matters.
Stage 1 acceptance and hooks stress remain on hold; keep the 20480 floor.

Documentation only. No firmware edits, builds, commits or pushes. Local B remains
selected. The pinned-core observations are local-source evidence, not a newly
validated allocation trace.

### Captured evidence

The request URL and calibration payload are omitted; diagnostic values are unchanged.

```text
2026-09-16 08:20:35.456 EVENT Console cleared.
2026-09-16 08:20:44.689 TX log status [CRLF]
2026-09-16 08:20:44.691 RX [LOG] state=ready boot=14 session=boot-14 up_ms=28673 clock=synced setup=1 hooks=0 file_bytes=524389 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922561024 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 08:20:44.692 RX [LOG] measured=1 stack_min=3988 internal_min=84888 internal_largest=31732 dma_min=77392 dma_largest=31732 writes=7 slow=0 write_max_us=2767 flush_max_us=9899 sd_max_us=174391 rotations=0 pruned=0 oversized=0
2026-09-16 08:20:44.693 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 08:20:44.694 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 08:20:44.694 RX [LOG MEM] phase=before_clock up_us=1881911 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 08:20:44.694 RX [LOG MEM] phase=after_clock up_us=1882598 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:20:44.694 RX [LOG MEM] phase=before_writer up_us=1882646 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:20:44.696 RX [LOG MEM] phase=writer_entry up_us=1882852 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:20:44.696 RX [LOG MEM] phase=after_formatter up_us=1882928 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:20:44.696 RX [LOG MEM] phase=before_mount up_us=1883496 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:20:44.696 RX [LOG MEM] phase=after_mount up_us=2057772 free=129996 largest=65524 heap_min_boot=128800
2026-09-16 08:20:44.696 RX [LOG MEM] phase=before_current_open up_us=2065089 free=129996 largest=65524 heap_min_boot=128800
2026-09-16 08:20:44.696 RX [LOG MEM] phase=after_current_open up_us=2066512 free=129996 largest=65524 heap_min_boot=128800
2026-09-16 08:20:44.696 RX [LOG MEM] phase=storage_done up_us=2078616 free=129996 largest=65524 heap_min_boot=128800
2026-09-16 08:20:52.063 RX Screen touched, resetting inactivity timer.
2026-09-16 08:20:52.063 RX Latest button clicked
2026-09-16 08:20:52.063 RX Initiating async latest image request...
2026-09-16 08:20:52.063 RX Preparing UI for new image request...
2026-09-16 08:20:52.063 RX Cleaning up image fetcher state...
2026-09-16 08:20:52.063 RX Screen 2 Loaded.
2026-09-16 08:20:52.135 RX === requestImage('latest') START ===
2026-09-16 08:20:52.135 RX Sending HTTP GET...
2026-09-16 08:20:52.136 RX [PROBE] window=normal run=1 ms=24123 heap_min_boot=84888 largest_min=31732 interval_ms=10 samples=2412 gap_max_us=10578 scan_max_us=436 timer=on imu_n=1181 imu_min_hz=29.41 imu_avg_hz=49.20
2026-09-16 08:20:52.935 RX [PROBE] window=image_https run=1 ms=798 heap_min_boot=34944 largest_min=26612 interval_ms=10 samples=79 gap_max_us=10647 scan_max_us=344 timer=on
2026-09-16 08:20:52.935 RX Response received in 799 ms, Content-Length: 35085
2026-09-16 08:20:52.935 RX Starting to receive image data...
2026-09-16 08:20:53.325 RX Image download complete (35085 bytes, 1262 ms since button press). Starting decode...
2026-09-16 08:20:53.463 RX JPEG decoded successfully into PSRAM.
2026-09-16 08:20:53.464 RX LVGL image source updated. Total 1401 ms from button press (budget 20000 ms).
2026-09-16 08:20:56.483 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 08:20:56.484 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 08:20:57.768 TX log status [CRLF]
2026-09-16 08:20:57.769 RX [LOG] state=ready boot=14 session=boot-14 up_ms=41751 clock=synced setup=1 hooks=0 file_bytes=524389 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922561024 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 08:20:57.769 RX [LOG] measured=1 stack_min=3988 internal_min=34944 internal_largest=26612 dma_min=27448 dma_largest=26612 writes=7 slow=0 write_max_us=2767 flush_max_us=9899 sd_max_us=174391 rotations=0 pruned=0 oversized=0
2026-09-16 08:20:57.771 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 08:20:57.771 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 08:20:57.771 RX [LOG MEM] phase=before_clock up_us=1881911 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 08:20:57.771 RX [LOG MEM] phase=after_clock up_us=1882598 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:20:57.772 RX [LOG MEM] phase=before_writer up_us=1882646 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:20:57.772 RX [LOG MEM] phase=writer_entry up_us=1882852 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:20:57.772 RX [LOG MEM] phase=after_formatter up_us=1882928 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:20:57.772 RX [LOG MEM] phase=before_mount up_us=1883496 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:20:57.772 RX [LOG MEM] phase=after_mount up_us=2057772 free=129996 largest=65524 heap_min_boot=128800
2026-09-16 08:20:57.773 RX [LOG MEM] phase=before_current_open up_us=2065089 free=129996 largest=65524 heap_min_boot=128800
2026-09-16 08:20:57.773 RX [LOG MEM] phase=after_current_open up_us=2066512 free=129996 largest=65524 heap_min_boot=128800
2026-09-16 08:20:57.773 RX [LOG MEM] phase=storage_done up_us=2078616 free=129996 largest=65524 heap_min_boot=128800
2026-09-16 08:21:12.992 RX [NET] WiFi=OFFLINE | MQTT=DISCONNECTED
2026-09-16 08:21:38.299 TX log status [CRLF]
2026-09-16 08:21:38.302 RX [LOG] state=ready boot=14 session=boot-14 up_ms=82285 clock=synced setup=1 hooks=0 file_bytes=524945 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922528256 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 08:21:38.303 RX [LOG] measured=1 stack_min=3988 internal_min=34944 internal_largest=26612 dma_min=27448 dma_largest=26612 writes=8 slow=0 write_max_us=2767 flush_max_us=9899 sd_max_us=174391 rotations=0 pruned=0 oversized=0
2026-09-16 08:21:38.303 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 08:21:38.304 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 08:21:38.304 RX [LOG MEM] phase=before_clock up_us=1881911 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 08:21:38.304 RX [LOG MEM] phase=after_clock up_us=1882598 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:21:38.304 RX [LOG MEM] phase=before_writer up_us=1882646 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:21:38.304 RX [LOG MEM] phase=writer_entry up_us=1882852 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:21:38.305 RX [LOG MEM] phase=after_formatter up_us=1882928 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:21:38.305 RX [LOG MEM] phase=before_mount up_us=1883496 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:21:38.305 RX [LOG MEM] phase=after_mount up_us=2057772 free=129996 largest=65524 heap_min_boot=128800
2026-09-16 08:21:38.305 RX [LOG MEM] phase=before_current_open up_us=2065089 free=129996 largest=65524 heap_min_boot=128800
2026-09-16 08:21:38.305 RX [LOG MEM] phase=after_current_open up_us=2066512 free=129996 largest=65524 heap_min_boot=128800
2026-09-16 08:21:38.305 RX [LOG MEM] phase=storage_done up_us=2078616 free=129996 largest=65524 heap_min_boot=128800
2026-09-16 08:21:53.472 RX [PROBE] window=normal run=2 ms=60001 heap_min_boot=34944 largest_min=30708 interval_ms=10 samples=6000 gap_max_us=11319 scan_max_us=763 timer=on imu_n=2920 imu_min_hz=10.10 imu_avg_hz=48.91
2026-09-16 08:22:32.794 RX [PROBE] window=normal run=3 ms=39322 heap_min_boot=34944 largest_min=32756 interval_ms=10 samples=3932 gap_max_us=11843 scan_max_us=701 timer=on imu_n=1916 imu_min_hz=36.79 imu_avg_hz=49.00
2026-09-16 08:22:33.412 RX [PROBE] window=mqtt_connect run=2 ms=618 heap_min_boot=22656 largest_min=14324 interval_ms=10 samples=62 gap_max_us=10273 scan_max_us=364 timer=on
2026-09-16 08:22:33.425 RX [NET] WiFi=CONNECTED | MQTT=REMOTE CONNECTED
2026-09-16 08:22:36.817 TX log status [CRLF]
2026-09-16 08:22:36.821 RX [LOG] state=ready boot=14 session=boot-14 up_ms=140800 clock=synced setup=1 hooks=0 file_bytes=525503 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922528256 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 08:22:36.821 RX [LOG] measured=1 stack_min=3988 internal_min=22656 internal_largest=14324 dma_min=15160 dma_largest=14324 writes=9 slow=0 write_max_us=2767 flush_max_us=9899 sd_max_us=174391 rotations=0 pruned=0 oversized=0
2026-09-16 08:22:36.823 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 08:22:36.823 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 08:22:36.823 RX [LOG MEM] phase=before_clock up_us=1881911 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 08:22:36.823 RX [LOG MEM] phase=after_clock up_us=1882598 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:22:36.823 RX [LOG MEM] phase=before_writer up_us=1882646 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:22:36.824 RX [LOG MEM] phase=writer_entry up_us=1882852 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:22:36.824 RX [LOG MEM] phase=after_formatter up_us=1882928 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:22:36.824 RX [LOG MEM] phase=before_mount up_us=1883496 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:22:36.824 RX [LOG MEM] phase=after_mount up_us=2057772 free=129996 largest=65524 heap_min_boot=128800
2026-09-16 08:22:36.824 RX [LOG MEM] phase=before_current_open up_us=2065089 free=129996 largest=65524 heap_min_boot=128800
2026-09-16 08:22:36.824 RX [LOG MEM] phase=after_current_open up_us=2066512 free=129996 largest=65524 heap_min_boot=128800
2026-09-16 08:22:36.824 RX [LOG MEM] phase=storage_done up_us=2078616 free=129996 largest=65524 heap_min_boot=128800
2026-09-16 08:22:42.649 RX Screen touched, resetting inactivity timer.
2026-09-16 08:22:42.649 RX Live button clicked -> starting live feed
2026-09-16 08:22:42.649 RX [PROBE] window=normal run=4 ms=9227 heap_min_boot=22656 largest_min=14324 interval_ms=10 samples=923 gap_max_us=10345 scan_max_us=459 timer=on imu_n=453 imu_min_hz=32.21 imu_avg_hz=49.19
2026-09-16 08:22:42.649 RX Screen 2 Loaded.
2026-09-16 08:22:42.721 RX Video: endpoint parsed, port 9835, path /esp32/live
2026-09-16 08:22:43.427 RX [PROBE] window=live_tls run=1 ms=704 heap_min_boot=22656 largest_min=14324 interval_ms=10 samples=70 gap_max_us=10325 scan_max_us=394 timer=on
2026-09-16 08:22:43.859 RX Video: frame 432x768, panel 368x448 -> gap x=-64 y=-320, pan x=-15 y=32
2026-09-16 08:23:42.995 RX [PROBE] window=live_tls run=2 ms=668 heap_min_boot=22656 largest_min=14324 interval_ms=10 samples=67 gap_max_us=10055 scan_max_us=149 timer=on
2026-09-16 08:23:42.996 RX Video: 170 frames in 60.3s (2.8 fps) | http 346 | decode 60 | blit 61 | frame 355 ms | first_frame 1332 | max_gap 1043 ms
2026-09-16 08:23:42.996 RX Video: http = ttfb 133 + xfer 212 ms | frame 18.9 KB | 89 KB/s while transferring
2026-09-16 08:23:42.997 RX Video: free PSRAM 7610520, free heap 46588
2026-09-16 08:23:42.998 RX [PROBE] window=live run=1 ms=60341 heap_min_boot=22656 largest_min=14324 interval_ms=10 samples=6034 gap_max_us=11147 scan_max_us=883 timer=on
2026-09-16 08:23:42.998 RX Video: returning to previous screen
2026-09-16 08:23:42.998 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 08:23:42.999 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 08:23:45.965 TX log status [CRLF]
2026-09-16 08:23:45.967 RX [LOG] state=ready boot=14 session=boot-14 up_ms=209940 clock=synced setup=1 hooks=0 file_bytes=526065 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922528256 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 08:23:45.968 RX [LOG] measured=1 stack_min=3988 internal_min=22656 internal_largest=14324 dma_min=15160 dma_largest=14324 writes=10 slow=0 write_max_us=2767 flush_max_us=9899 sd_max_us=174391 rotations=0 pruned=0 oversized=0
2026-09-16 08:23:45.969 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 08:23:45.969 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 08:23:45.969 RX [LOG MEM] phase=before_clock up_us=1881911 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 08:23:45.969 RX [LOG MEM] phase=after_clock up_us=1882598 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:23:45.971 RX [LOG MEM] phase=before_writer up_us=1882646 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:23:45.971 RX [LOG MEM] phase=writer_entry up_us=1882852 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:23:45.971 RX [LOG MEM] phase=after_formatter up_us=1882928 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:23:45.971 RX [LOG MEM] phase=before_mount up_us=1883496 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:23:45.971 RX [LOG MEM] phase=after_mount up_us=2057772 free=129996 largest=65524 heap_min_boot=128800
2026-09-16 08:23:45.972 RX [LOG MEM] phase=before_current_open up_us=2065089 free=129996 largest=65524 heap_min_boot=128800
2026-09-16 08:23:45.972 RX [LOG MEM] phase=after_current_open up_us=2066512 free=129996 largest=65524 heap_min_boot=128800
2026-09-16 08:23:45.972 RX [LOG MEM] phase=storage_done up_us=2078616 free=129996 largest=65524 heap_min_boot=128800
```


## JP-requested checkpoint commit - 2026-09-16

JP requested documentation and a commit while preparing the no-media reconnect
control. The concise reproduction and checkpoint scope are in
[sd_diagnostics_reconnect_reproducer.md](sd_diagnostics_reconnect_reproducer.md).
Commit the experiment code, its review references and accumulated evidence.
The committed stack-switch default stays 0; preserve JP's local value 1 for the
pending B control. Stage 1 remains unaccepted. No push, build or flash requested.


## No-media hotspot reconnect control - 2026-09-16 08:30

JP supplied the fresh-boot control without a prior Latest, Live or other media
request. Boot 15 is B, hooks=0, valid PSRAM stack placement and internal TCB.

| Measurement | Result |
|-------------|--------|
| Initial historical internal_min / internal_largest | 84904 / 31732 bytes |
| Offline notification | 08:30:44.104 |
| Offline normal-window largest_min | 38900 bytes, 17030 ms, 1703 samples |
| MQTT reconnect | 753 ms, largest_min=31732, 75 samples |
| MQTT probe maximum gap / scan | 10340 / 152 us |
| Connected notification | 08:31:30.823 |
| Historical internal_min / internal_largest after reconnect | 84904 / 31732, unchanged |
| Historical DMA minimum / largest minimum | 77408 / 31732, unchanged |
| Writer stack used / margin | 4204 / 3988, unchanged |
| Logger state / error / drops | ready / none / zero |
| Writes / file size | 7 to 8 / 530700 to 531257 bytes |

The control PASSES the 20480-byte floor with 11252 bytes of margin.
The two normal IMU minimum/average pairs are 35.62/49.03 and 36.80/49.04 Hz.
Timer remained on at 10 ms. Network recovery succeeded, with no reported reset,
TLS/allocation failure or logger error. Queue high water remained 1/16, with
no suppressed/truncated records or slow writes. Maximum write/flush/SD operation
were 1877/5070/173062 us. No rotation/pruning occurred and no Live was requested.

Comparison: after Latest in boot 14, mqtt_connect reached 14324; without prior
media in boot 15, it reached 31732. The difference is 17408 bytes in the sampled
largest-block minima, not a measured allocation size or total RAM saving.
Recorded offline-to-green intervals differ: 46.719 seconds here versus 80.433
in boot 14. Actual hotspot switch times are unknown. Treat this as supportive
control evidence, not proof of an isolated cause or that image history is always
necessary. The requested roughly 90-second duration was not observed in either
notification interval.

Interpretation: along with pinned-core keep-alive source evidence, this makes
the still-image secure-client lifetime the leading actionable hypothesis.
No code change has been made. Recommend a narrow experiment: after complete
still-image body receipt and HTTPClient::end, explicitly stop the shared HTTPS
client before decoding, preserving JPEG data and normal Live keep-alive.
A subsequent still-to-Live handover will establish a fresh TLS connection,
so validate its first-frame delay in follow-up. Do not invoke the existing broad
cleanup helper here because it frees the downloaded image buffers.

Next work is JP's agreement to that bounded experiment, then implementation and
the known failing Latest/hotspot/reconnect test with consistent outage duration.
No additional general button tests are requested. Stage 1 remains unaccepted;
20480 floor, probes, timeouts and local B setting remain unchanged.

Documentation only in this turn. No firmware edits, builds, commits or pushes.

### Captured evidence

Calibration payload is omitted; diagnostic values are unchanged.

```text
2026-09-16 08:30:22.064 EVENT Console cleared.
2026-09-16 08:30:28.983 TX log status [CRLF]
2026-09-16 08:30:28.985 RX [LOG] state=ready boot=15 session=boot-15 up_ms=24683 clock=synced setup=1 hooks=0 file_bytes=530700 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922528256 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 08:30:28.985 RX [LOG] measured=1 stack_min=3988 internal_min=84904 internal_largest=31732 dma_min=77408 dma_largest=31732 writes=7 slow=0 write_max_us=1877 flush_max_us=5070 sd_max_us=173062 rotations=0 pruned=0 oversized=0
2026-09-16 08:30:28.986 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 08:30:28.987 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 08:30:28.988 RX [LOG MEM] phase=before_clock up_us=1781913 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 08:30:28.988 RX [LOG MEM] phase=after_clock up_us=1782601 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:30:28.988 RX [LOG MEM] phase=before_writer up_us=1782649 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:30:28.989 RX [LOG MEM] phase=writer_entry up_us=1782855 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:30:28.989 RX [LOG MEM] phase=after_formatter up_us=1782936 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:30:28.989 RX [LOG MEM] phase=before_mount up_us=1783510 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:30:28.989 RX [LOG MEM] phase=after_mount up_us=1956437 free=129996 largest=65524 heap_min_boot=128684
2026-09-16 08:30:28.989 RX [LOG MEM] phase=before_current_open up_us=1963763 free=129996 largest=65524 heap_min_boot=128684
2026-09-16 08:30:28.990 RX [LOG MEM] phase=after_current_open up_us=1965150 free=129996 largest=65524 heap_min_boot=128684
2026-09-16 08:30:28.990 RX [LOG MEM] phase=storage_done up_us=1976509 free=129996 largest=65524 heap_min_boot=128684
2026-09-16 08:30:44.104 RX [NET] WiFi=OFFLINE | MQTT=DISCONNECTED
2026-09-16 08:30:56.048 TX log status [CRLF]
2026-09-16 08:30:56.054 RX [LOG] state=ready boot=15 session=boot-15 up_ms=51753 clock=synced setup=1 hooks=0 file_bytes=530700 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922528256 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 08:30:56.055 RX [LOG] measured=1 stack_min=3988 internal_min=84904 internal_largest=31732 dma_min=77408 dma_largest=31732 writes=7 slow=0 write_max_us=1877 flush_max_us=5070 sd_max_us=173062 rotations=0 pruned=0 oversized=0
2026-09-16 08:30:56.055 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 08:30:56.056 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 08:30:56.056 RX [LOG MEM] phase=before_clock up_us=1781913 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 08:30:56.056 RX [LOG MEM] phase=after_clock up_us=1782601 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:30:56.057 RX [LOG MEM] phase=before_writer up_us=1782649 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:30:56.057 RX [LOG MEM] phase=writer_entry up_us=1782855 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:30:56.057 RX [LOG MEM] phase=after_formatter up_us=1782936 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:30:56.057 RX [LOG MEM] phase=before_mount up_us=1783510 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:30:56.057 RX [LOG MEM] phase=after_mount up_us=1956437 free=129996 largest=65524 heap_min_boot=128684
2026-09-16 08:30:56.057 RX [LOG MEM] phase=before_current_open up_us=1963763 free=129996 largest=65524 heap_min_boot=128684
2026-09-16 08:30:56.057 RX [LOG MEM] phase=after_current_open up_us=1965150 free=129996 largest=65524 heap_min_boot=128684
2026-09-16 08:30:56.057 RX [LOG MEM] phase=storage_done up_us=1976509 free=129996 largest=65524 heap_min_boot=128684
2026-09-16 08:31:13.027 RX [PROBE] window=normal run=1 ms=60000 heap_min_boot=84904 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=11192 scan_max_us=674 timer=on imu_n=2935 imu_min_hz=35.62 imu_avg_hz=49.03
2026-09-16 08:31:30.811 RX [PROBE] window=normal run=2 ms=17030 heap_min_boot=84904 largest_min=38900 interval_ms=10 samples=1703 gap_max_us=11691 scan_max_us=1349 timer=on imu_n=827 imu_min_hz=36.80 imu_avg_hz=49.04
2026-09-16 08:31:30.811 RX [PROBE] window=mqtt_connect run=2 ms=753 heap_min_boot=84904 largest_min=31732 interval_ms=10 samples=75 gap_max_us=10340 scan_max_us=152 timer=on
2026-09-16 08:31:30.823 RX [NET] WiFi=CONNECTED | MQTT=REMOTE CONNECTED
2026-09-16 08:31:36.890 TX log status [CRLF]
2026-09-16 08:31:36.893 RX [LOG] state=ready boot=15 session=boot-15 up_ms=92588 clock=synced setup=1 hooks=0 file_bytes=531257 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922528256 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 08:31:36.893 RX [LOG] measured=1 stack_min=3988 internal_min=84904 internal_largest=31732 dma_min=77408 dma_largest=31732 writes=8 slow=0 write_max_us=1877 flush_max_us=5070 sd_max_us=173062 rotations=0 pruned=0 oversized=0
2026-09-16 08:31:36.895 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 08:31:36.895 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 08:31:36.896 RX [LOG MEM] phase=before_clock up_us=1781913 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 08:31:36.896 RX [LOG MEM] phase=after_clock up_us=1782601 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:31:36.897 RX [LOG MEM] phase=before_writer up_us=1782649 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:31:36.897 RX [LOG MEM] phase=writer_entry up_us=1782855 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:31:36.897 RX [LOG MEM] phase=after_formatter up_us=1782936 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:31:36.897 RX [LOG MEM] phase=before_mount up_us=1783510 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:31:36.897 RX [LOG MEM] phase=after_mount up_us=1956437 free=129996 largest=65524 heap_min_boot=128684
2026-09-16 08:31:36.897 RX [LOG MEM] phase=before_current_open up_us=1963763 free=129996 largest=65524 heap_min_boot=128684
2026-09-16 08:31:36.897 RX [LOG MEM] phase=after_current_open up_us=1965150 free=129996 largest=65524 heap_min_boot=128684
2026-09-16 08:31:36.897 RX [LOG MEM] phase=storage_done up_us=1976509 free=129996 largest=65524 heap_min_boot=128684
```


## Approved still-image TLS-close experiment prepared - 2026-09-16

JP authorized the narrow cleanup change. The only executable firmware addition
is httpsClient.stop in image_fetcher.cpp after a full JPEG body has been received
and httpClient.end has run, before setting HTTP_DECODING. It leaves the downloaded
JPEG available to decode. No other network or image-state transitions changed.

Source review confirmed the call is in the still completion path, while
imageFetcherLoop stands down during active Live. The installed core's stop
calls stop_ssl_socket and clears connected state. The ordinary shared-client
configuration remains usable for a later connect; Live owns its separate
reuse/teardown behavior. No error-path cleanup expansion was included.

JP's local DIAG_WRITER_STACK_PSRAM=1 and DIAG_TEST_HOOKS=0 are preserved.
Stage 0 probes, writer code, timeouts and companion.ino are unchanged.
No generated-sketch deletion is needed for this src-only edit; the documented
stale-sketch deletion rule still applies to future companion.ino edits.
Source/diff checks only; JP performs compilation and flashing.

First validation is log status, Latest and return, log status, actual hotspot
off for 90 seconds, on, green MQTT, log status. Stop there without Live and
send the complete capture. Look for mqtt_connect largest_min at least 20480
instead of 14324, with normal operation and no errors or drops. A successful
result must be measured, not presumed. Follow with media and handover checks
separately, since still-to-Live now needs a fresh handshake.

No firmware build, flash, commit or push by the assistant. Stage 1 remains
unaccepted while this change awaits JP's bench result.


## First patched reconnect passes - 2026-09-16, boot 17, 08:49-08:51

JP built and flashed the still-image TLS-close experiment, then repeated Latest
and an actual hotspot outage/recovery. B remains selected with hooks off.
The capture ends after green MQTT and log status; no Live ran in this test.

| Measurement | Result |
|-------------|--------|
| Initial historical internal minimum / largest block | 84936 / 31732 bytes |
| Latest HTTPS window | 760 ms; largest_min=25588; 76 samples at 10 ms |
| Latest completion | 35085 bytes; 1302 ms; decoded and displayed |
| Offline / green notifications | 08:49:58.176 / 08:51:05.410; interval 67.234 s |
| MQTT reconnect window | 509 ms; largest_min=31732; 51 samples at 10 ms |
| MQTT maximum sample gap / scan time | 10565 / 435 us |
| Final historical internal minimum / largest block | 36084 / 25588 bytes |
| Writer used / margin | 4204 / 3988 bytes, unchanged |
| Placement / lifecycle | PSRAM 8192, internal TCB 352, valid, active |
| Logger state / errors / drops | ready / none / zero |
| Queue peak / writes / slow writes | 1 of 16 / 9 / zero |
| Maximum write / flush / SD operation | 2280 / 4704 / 100422 us |

The reconnect window improved from 14324 in boot 14 to 31732 bytes, exceeding
the unchanged 20480-byte floor by 11252. The boot's lowest observed block remains
25588 from Latest, also above the floor. Status keeps that historical minimum;
it does not contradict the separate MQTT window's 31732. Historical internal_min
and DMA minima did not fall further at reconnect.

Normal windows reported IMU averages 49.16, 48.67 and 48.64 Hz; minima were
36.98, 10.78 and 36.80 Hz. The offline normal window's largest_min was 38900.
There was no new logger error, queue drop or stack-margin loss.

This first patched sequence passes and supports releasing the retained still
HTTPS connection. It does not identify the original overnight allocation or
accept Stage 1. Actual hotspot toggle times are unknown: notification intervals
differ from boot 14's 80.433 s and the no-media control's 46.719 s.
The Latest timing is a successful completion measurement, not proof of a speed gain.

Next test: keep this firmware and boot, send log status, run one full Live cycle,
then send log status and the complete Live capture. Back still requests and
motion still-to-Live handover timing remain later checks. Hooks stress, paired
performance and the remaining Stage 1 gates are still pending.

### Serial evidence

JP supplied this capture. The endpoint and calibration payload lines are omitted.

```text
2026-09-16 08:49:14.928 EVENT Console cleared.
2026-09-16 08:49:20.426 TX log status [CRLF]
2026-09-16 08:49:20.428 RX [LOG] state=ready boot=17 session=boot-17 up_ms=28112 clock=synced setup=1 hooks=0 file_bytes=542646 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922528256 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 08:49:20.430 RX [LOG] measured=1 stack_min=3988 internal_min=84936 internal_largest=31732 dma_min=77440 dma_largest=31732 writes=7 slow=0 write_max_us=1847 flush_max_us=4704 sd_max_us=100422 rotations=0 pruned=0 oversized=0
2026-09-16 08:49:20.430 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 08:49:20.430 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 08:49:20.431 RX [LOG MEM] phase=before_clock up_us=1782940 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 08:49:20.431 RX [LOG MEM] phase=after_clock up_us=1783645 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:49:20.432 RX [LOG MEM] phase=before_writer up_us=1783692 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:49:20.432 RX [LOG MEM] phase=writer_entry up_us=1783898 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:49:20.432 RX [LOG MEM] phase=after_formatter up_us=1783967 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:49:20.432 RX [LOG MEM] phase=before_mount up_us=1784551 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:49:20.432 RX [LOG MEM] phase=after_mount up_us=1884853 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:49:20.432 RX [LOG MEM] phase=before_current_open up_us=1892161 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:49:20.432 RX [LOG MEM] phase=after_current_open up_us=1893569 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:49:20.432 RX [LOG MEM] phase=storage_done up_us=1907750 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:49:27.240 RX Screen touched, resetting inactivity timer.
2026-09-16 08:49:27.240 RX Latest button clicked
2026-09-16 08:49:27.240 RX Initiating async latest image request...
2026-09-16 08:49:27.240 RX Preparing UI for new image request...
2026-09-16 08:49:27.240 RX Cleaning up image fetcher state...
2026-09-16 08:49:27.240 RX Screen 2 Loaded.
2026-09-16 08:49:27.312 RX === requestImage('latest') START ===
2026-09-16 08:49:27.312 RX Sending HTTP GET...
2026-09-16 08:49:27.313 RX [PROBE] window=normal run=1 ms=26219 heap_min_boot=84936 largest_min=31732 interval_ms=10 samples=2622 gap_max_us=10721 scan_max_us=382 timer=on imu_n=1283 imu_min_hz=36.98 imu_avg_hz=49.16
2026-09-16 08:49:28.077 RX [PROBE] window=image_https run=1 ms=760 heap_min_boot=36084 largest_min=25588 interval_ms=10 samples=76 gap_max_us=10555 scan_max_us=310 timer=on
2026-09-16 08:49:28.077 RX Response received in 761 ms, Content-Length: 35085
2026-09-16 08:49:28.077 RX Starting to receive image data...
2026-09-16 08:49:28.405 RX Image download complete (35085 bytes, 1161 ms since button press). Starting decode...
2026-09-16 08:49:28.545 RX JPEG decoded successfully into PSRAM.
2026-09-16 08:49:28.546 RX LVGL image source updated. Total 1302 ms from button press (budget 20000 ms).
2026-09-16 08:49:32.202 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 08:49:32.202 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 08:49:33.958 TX log status [CRLF]
2026-09-16 08:49:33.961 RX [LOG] state=ready boot=17 session=boot-17 up_ms=41642 clock=synced setup=1 hooks=0 file_bytes=542646 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922528256 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 08:49:33.962 RX [LOG] measured=1 stack_min=3988 internal_min=36084 internal_largest=25588 dma_min=28588 dma_largest=25588 writes=7 slow=0 write_max_us=1847 flush_max_us=4704 sd_max_us=100422 rotations=0 pruned=0 oversized=0
2026-09-16 08:49:33.963 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 08:49:33.963 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 08:49:33.964 RX [LOG MEM] phase=before_clock up_us=1782940 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 08:49:33.964 RX [LOG MEM] phase=after_clock up_us=1783645 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:49:33.964 RX [LOG MEM] phase=before_writer up_us=1783692 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:49:33.965 RX [LOG MEM] phase=writer_entry up_us=1783898 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:49:33.965 RX [LOG MEM] phase=after_formatter up_us=1783967 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:49:33.966 RX [LOG MEM] phase=before_mount up_us=1784551 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:49:33.967 RX [LOG MEM] phase=after_mount up_us=1884853 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:49:33.967 RX [LOG MEM] phase=before_current_open up_us=1892161 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:49:33.967 RX [LOG MEM] phase=after_current_open up_us=1893569 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:49:33.967 RX [LOG MEM] phase=storage_done up_us=1907750 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:49:58.175 TX log status [CRLF]
2026-09-16 08:49:58.176 RX [NET] WiFi=OFFLINE | MQTT=DISCONNECTED
2026-09-16 08:49:58.177 RX [LOG] state=ready boot=17 session=boot-17 up_ms=65858 clock=synced setup=1 hooks=0 file_bytes=543202 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922528256 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 08:49:58.177 RX [LOG] measured=1 stack_min=3988 internal_min=36084 internal_largest=25588 dma_min=28588 dma_largest=25588 writes=8 slow=0 write_max_us=1847 flush_max_us=4704 sd_max_us=100422 rotations=0 pruned=0 oversized=0
2026-09-16 08:49:58.178 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 08:49:58.179 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 08:49:58.179 RX [LOG MEM] phase=before_clock up_us=1782940 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 08:49:58.179 RX [LOG MEM] phase=after_clock up_us=1783645 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:49:58.179 RX [LOG MEM] phase=before_writer up_us=1783692 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:49:58.179 RX [LOG MEM] phase=writer_entry up_us=1783898 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:49:58.179 RX [LOG MEM] phase=after_formatter up_us=1783967 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:49:58.179 RX [LOG MEM] phase=before_mount up_us=1784551 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:49:58.179 RX [LOG MEM] phase=after_mount up_us=1884853 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:49:58.179 RX [LOG MEM] phase=before_current_open up_us=1892161 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:49:58.179 RX [LOG MEM] phase=after_current_open up_us=1893569 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:49:58.179 RX [LOG MEM] phase=storage_done up_us=1907750 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:50:28.553 RX [PROBE] window=normal run=2 ms=60004 heap_min_boot=36084 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10982 scan_max_us=638 timer=on imu_n=2904 imu_min_hz=10.78 imu_avg_hz=48.67
2026-09-16 08:51:04.597 RX [PROBE] window=normal run=3 ms=36044 heap_min_boot=36084 largest_min=38900 interval_ms=10 samples=3605 gap_max_us=13046 scan_max_us=1587 timer=on imu_n=1742 imu_min_hz=36.80 imu_avg_hz=48.64
2026-09-16 08:51:05.107 RX [PROBE] window=mqtt_connect run=2 ms=509 heap_min_boot=36084 largest_min=31732 interval_ms=10 samples=51 gap_max_us=10565 scan_max_us=435 timer=on
2026-09-16 08:51:05.410 RX [NET] WiFi=CONNECTED | MQTT=REMOTE CONNECTED
2026-09-16 08:51:08.531 TX log status [CRLF]
2026-09-16 08:51:08.534 RX [LOG] state=ready boot=17 session=boot-17 up_ms=136213 clock=synced setup=1 hooks=0 file_bytes=543761 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922528256 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 08:51:08.535 RX [LOG] measured=1 stack_min=3988 internal_min=36084 internal_largest=25588 dma_min=28588 dma_largest=25588 writes=9 slow=0 write_max_us=2280 flush_max_us=4704 sd_max_us=100422 rotations=0 pruned=0 oversized=0
2026-09-16 08:51:08.536 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 08:51:08.536 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 08:51:08.536 RX [LOG MEM] phase=before_clock up_us=1782940 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 08:51:08.537 RX [LOG MEM] phase=after_clock up_us=1783645 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:51:08.537 RX [LOG MEM] phase=before_writer up_us=1783692 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:51:08.537 RX [LOG MEM] phase=writer_entry up_us=1783898 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:51:08.537 RX [LOG MEM] phase=after_formatter up_us=1783967 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:51:08.537 RX [LOG MEM] phase=before_mount up_us=1784551 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:51:08.537 RX [LOG MEM] phase=after_mount up_us=1884853 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:51:08.537 RX [LOG MEM] phase=before_current_open up_us=1892161 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:51:08.537 RX [LOG MEM] phase=after_current_open up_us=1893569 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:51:08.537 RX [LOG MEM] phase=storage_done up_us=1907750 free=129996 largest=65524 heap_min_boot=128864
```


## Patched full Live after reconnect passes - 2026-09-16, boot 17, 08:57-08:58

JP continued with the installed patched B firmware, hooks off, in the same boot
as the passing Latest/hotspot/reconnect sequence. JP reported normal operation
visually throughout this full Live cycle. The unit returned to screen 1 without
saving a screen preference.

| Measurement | Result |
|-------------|--------|
| First Live TLS | 782 ms; largest_min=26612; 78 samples |
| Second Live TLS | 630 ms; largest_min=25588; 63 samples |
| Full Live probe | 60260 ms; largest_min=25588; 6026 samples |
| Sampling interval / maximum gap / longest scan, full Live | 10 ms / 11339 us / 1029 us |
| Video | 196 frames in 60.3 s; reported 3.3 fps |
| First frame / maximum frame gap | 1241 / 905 ms |
| Average HTTP / decode / blit / frame | 299 / 60 / 62 / 307 ms |
| HTTP TTFB / transfer / average frame / transfer rate | 130 ms / 169 ms / 18.1 KB / 107 KB/s |
| End Video free PSRAM / free heap | 7610560 / 46600 bytes |
| Historical internal_min, before to after | 36084 to 34688 bytes |
| Historical largest internal block, before and after | 25588 bytes |
| DMA minimum / largest, after | 27192 / 25588 bytes |
| Writer used / margin, before and after | 4204 / 3988 bytes |
| Logger state / errors / drops / queue peak | ready / none / zero / 1 of 16 |
| Writes, before to after / slow writes | 15 to 16 / zero |
| File bytes, before to after | 547132 to 547694 |

All sampled Live connection and full-cycle block minima exceed the unchanged
20480-byte gate; the lowest has 5108 bytes of margin. The failed pre-patch
Latest/outage sequence measured 14324 during reconnect and subsequent Live.
Patched boot 17 passed reconnect at 31732 and the following Live at 25588.
This supports the cleanup for the reproduced sequence.

The 1396-byte decrease in historical internal_min occurred during Live; it is
not a decrease of the largest-block minimum and alone does not demonstrate a leak.
Writer placement remained valid (8192-byte PSRAM stack, internal 352-byte TCB).
Stack used and margin were unchanged, as were the logger latency maxima:
write 2280 us, flush 4704 us, SD operation 100422 us.
The preceding normal window reported IMU average 49.12 Hz and minimum 38.26 Hz.

The 3.3 fps is a successful functional result, not a paired logging-on/off
performance comparison. The unobserved overnight event remains untraced.
Stage 1 acceptance, hooks stress and remaining storage tests are still pending.
Next single test: log status, Back older-image request, wait for display,
return to dashboard, log status. No restart, reflash or hotspot outage.
Motion still-to-Live handover latency remains a later separate check.

### Serial evidence

```text
2026-09-16 08:57:09.587 EVENT Console cleared.
2026-09-16 08:57:12.481 TX log status [CRLF]
2026-09-16 08:57:12.484 RX [LOG] state=ready boot=17 session=boot-17 up_ms=500158 clock=synced setup=1 hooks=0 file_bytes=547132 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922528256 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 08:57:12.485 RX [LOG] measured=1 stack_min=3988 internal_min=36084 internal_largest=25588 dma_min=28588 dma_largest=25588 writes=15 slow=0 write_max_us=2280 flush_max_us=4704 sd_max_us=100422 rotations=0 pruned=0 oversized=0
2026-09-16 08:57:12.486 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 08:57:12.486 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 08:57:12.486 RX [LOG MEM] phase=before_clock up_us=1782940 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 08:57:12.486 RX [LOG MEM] phase=after_clock up_us=1783645 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:57:12.487 RX [LOG MEM] phase=before_writer up_us=1783692 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:57:12.488 RX [LOG MEM] phase=writer_entry up_us=1783898 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:57:12.488 RX [LOG MEM] phase=after_formatter up_us=1783967 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:57:12.488 RX [LOG MEM] phase=before_mount up_us=1784551 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:57:12.488 RX [LOG MEM] phase=after_mount up_us=1884853 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:57:12.488 RX [LOG MEM] phase=before_current_open up_us=1892161 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:57:12.488 RX [LOG MEM] phase=after_current_open up_us=1893569 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:57:12.488 RX [LOG MEM] phase=storage_done up_us=1907750 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:57:14.451 RX Screen touched, resetting inactivity timer.
2026-09-16 08:57:14.451 RX Live button clicked -> starting live feed
2026-09-16 08:57:14.452 RX [PROBE] window=normal run=10 ms=9315 heap_min_boot=36084 largest_min=31732 interval_ms=10 samples=932 gap_max_us=11068 scan_max_us=1170 timer=on imu_n=455 imu_min_hz=38.26 imu_avg_hz=49.12
2026-09-16 08:57:14.452 RX Screen 2 Loaded.
2026-09-16 08:57:14.523 RX Video: endpoint parsed, port 9835, path /esp32/live
2026-09-16 08:57:15.307 RX [PROBE] window=live_tls run=1 ms=782 heap_min_boot=34932 largest_min=26612 interval_ms=10 samples=78 gap_max_us=10411 scan_max_us=466 timer=on
2026-09-16 08:57:15.572 RX Video: frame 432x768, panel 368x448 -> gap x=-64 y=-320, pan x=-15 y=32
2026-09-16 08:57:45.532 RX [PROBE] window=live_tls run=2 ms=630 heap_min_boot=34688 largest_min=25588 interval_ms=10 samples=63 gap_max_us=10082 scan_max_us=138 timer=on
2026-09-16 08:58:14.711 RX Video: 196 frames in 60.3s (3.3 fps) | http 299 | decode 60 | blit 62 | frame 307 ms | first_frame 1241 | max_gap 905 ms
2026-09-16 08:58:14.712 RX Video: http = ttfb 130 + xfer 169 ms | frame 18.1 KB | 107 KB/s while transferring
2026-09-16 08:58:14.712 RX Video: free PSRAM 7610560, free heap 46600
2026-09-16 08:58:14.713 RX [PROBE] window=live run=1 ms=60260 heap_min_boot=34688 largest_min=25588 interval_ms=10 samples=6026 gap_max_us=11339 scan_max_us=1029 timer=on
2026-09-16 08:58:14.714 RX Video: returning to previous screen
2026-09-16 08:58:14.714 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 08:58:14.714 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 08:58:18.061 TX log status [CRLF]
2026-09-16 08:58:18.064 RX [LOG] state=ready boot=17 session=boot-17 up_ms=565736 clock=synced setup=1 hooks=0 file_bytes=547694 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922528256 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 08:58:18.064 RX [LOG] measured=1 stack_min=3988 internal_min=34688 internal_largest=25588 dma_min=27192 dma_largest=25588 writes=16 slow=0 write_max_us=2280 flush_max_us=4704 sd_max_us=100422 rotations=0 pruned=0 oversized=0
2026-09-16 08:58:18.065 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 08:58:18.066 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 08:58:18.066 RX [LOG MEM] phase=before_clock up_us=1782940 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 08:58:18.068 RX [LOG MEM] phase=after_clock up_us=1783645 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:58:18.068 RX [LOG MEM] phase=before_writer up_us=1783692 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:58:18.068 RX [LOG MEM] phase=writer_entry up_us=1783898 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:58:18.068 RX [LOG MEM] phase=after_formatter up_us=1783967 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:58:18.068 RX [LOG MEM] phase=before_mount up_us=1784551 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 08:58:18.068 RX [LOG MEM] phase=after_mount up_us=1884853 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:58:18.068 RX [LOG MEM] phase=before_current_open up_us=1892161 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:58:18.068 RX [LOG MEM] phase=after_current_open up_us=1893569 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 08:58:18.068 RX [LOG MEM] phase=storage_done up_us=1907750 free=129996 largest=65524 heap_min_boot=128864
```


## Early Live exit followed by Latest passes - 2026-09-16, boot 17, 09:02-09:03

JP reported normal operation on the device. The hotspot remained on; this was
a manual early exit from Live, not a Wi-Fi interruption. The patched B firmware
and boot were unchanged, with hooks off.

| Measurement | Result |
|-------------|--------|
| Live TLS window | 782 ms; largest_min=26612; 78 samples |
| Early Live cycle | 35 frames in 11.7 s; reported 3.0 fps |
| First frame / maximum frame gap | 1268 / 349 ms |
| Full active-Live probe | 11727 ms; largest_min=26612; 1172 samples |
| Live sampling interval / maximum gap / maximum scan | 10 ms / 10648 us / 725 us |
| Exit evidence | 09:02:48.858: screen left, stopping feed; returned to screen 1 |
| Normal window after exit | largest_min=31732; 586 samples over 5865 ms |
| Following Latest HTTPS | 708 ms; largest_min=26612; 71 samples at 10 ms |
| Latest completion | 35085 bytes; 1298 ms; decoded and displayed |
| Historical internal minimum, before to after | 34688 to 34548 bytes |
| Historical largest internal block, before and after | 25588 bytes |
| Writer used / margin, before and after | 4204 / 3988 bytes |
| Logger state / errors / drops / queue peak | ready / none / zero / 1 of 16 |
| Writes / file bytes, before to after | 20 to 21 / 549942 to 550504 |
| Final DMA minimum / largest | 27052 / 25588 bytes |

Both Live and the subsequent Latest request passed the unchanged 20480-byte
floor, with 6132 bytes of margin in these windows. The historical largest-block
minimum stayed 25588 from earlier operations. The 140-byte change in historical
internal_min occurred during Live and did not fall further during Latest.
This single change does not establish a leak.

The screen-left path stopped Live, and the subsequent normal-window minimum
recovered to 31732. Latest then connected and displayed successfully. Together
with JP's visual confirmation, this passes the early-exit functional and memory
check; it does not account individually for every freed allocation.

Writer placement remained valid and active, with the 8192-byte PSRAM stack and
352-byte internal TCB. Stack margin and logger latency maxima were unchanged:
write 2280 us, flush 4704 us, SD operation 100422 us, slow writes zero.
Normal-operation IMU averages were 49.27 and 49.17 Hz; minima 37.01 and 30.26 Hz.
The short Live cycle's fps includes startup and is not a paired performance test.

Next single test remains Back older-image retrieval: log status, Back, wait for
the image, return to dashboard, log status. Keep the hotspot on and do not
restart or reflash. Motion still-to-Live handover, hooks stress, paired performance
and the remaining Stage 1 gates are still pending. No firmware edits, build,
flash, commit or push were performed for this result.

### Serial evidence

The endpoint line is omitted.

```text
2026-09-16 09:02:33.478 EVENT Console cleared.
2026-09-16 09:02:34.904 TX log status [CRLF]
2026-09-16 09:02:34.912 RX [LOG] state=ready boot=17 session=boot-17 up_ms=822583 clock=synced setup=1 hooks=0 file_bytes=549942 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922528256 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 09:02:34.913 RX [LOG] measured=1 stack_min=3988 internal_min=34688 internal_largest=25588 dma_min=27192 dma_largest=25588 writes=20 slow=0 write_max_us=2280 flush_max_us=4704 sd_max_us=100422 rotations=0 pruned=0 oversized=0
2026-09-16 09:02:34.913 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 09:02:34.915 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 09:02:34.915 RX [LOG MEM] phase=before_clock up_us=1782940 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 09:02:34.915 RX [LOG MEM] phase=after_clock up_us=1783645 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:02:34.915 RX [LOG MEM] phase=before_writer up_us=1783692 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:02:34.915 RX [LOG MEM] phase=writer_entry up_us=1783898 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:02:34.915 RX [LOG MEM] phase=after_formatter up_us=1783967 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:02:34.915 RX [LOG MEM] phase=before_mount up_us=1784551 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:02:34.915 RX [LOG MEM] phase=after_mount up_us=1884853 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:02:34.916 RX [LOG MEM] phase=before_current_open up_us=1892161 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:02:34.916 RX [LOG MEM] phase=after_current_open up_us=1893569 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:02:34.916 RX [LOG MEM] phase=storage_done up_us=1907750 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:02:37.132 RX Screen touched, resetting inactivity timer.
2026-09-16 09:02:37.132 RX Live button clicked -> starting live feed
2026-09-16 09:02:37.133 RX [PROBE] window=normal run=15 ms=22403 heap_min_boot=34688 largest_min=31732 interval_ms=10 samples=2241 gap_max_us=10380 scan_max_us=357 timer=on imu_n=1102 imu_min_hz=37.01 imu_avg_hz=49.27
2026-09-16 09:02:37.133 RX Screen 2 Loaded.
2026-09-16 09:02:37.204 RX Video: endpoint parsed, port 9835, path /esp32/live
2026-09-16 09:02:37.987 RX [PROBE] window=live_tls run=3 ms=782 heap_min_boot=34688 largest_min=26612 interval_ms=10 samples=78 gap_max_us=10178 scan_max_us=151 timer=on
2026-09-16 09:02:48.858 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 09:02:48.858 RX Video: screen left, stopping feed
2026-09-16 09:02:48.858 RX Video: 35 frames in 11.7s (3.0 fps) | http 298 | decode 60 | blit 62 | frame 326 ms | first_frame 1268 | max_gap 349 ms
2026-09-16 09:02:48.859 RX Video: http = ttfb 129 + xfer 169 ms | frame 18.1 KB | 107 KB/s while transferring
2026-09-16 09:02:48.859 RX Video: free PSRAM 7601064, free heap 46616
2026-09-16 09:02:48.861 RX [PROBE] window=live run=2 ms=11727 heap_min_boot=34548 largest_min=26612 interval_ms=10 samples=1172 gap_max_us=10648 scan_max_us=725 timer=on
2026-09-16 09:02:48.861 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 09:02:51.355 TX log status [CRLF]
2026-09-16 09:02:51.358 RX [LOG] state=ready boot=17 session=boot-17 up_ms=839030 clock=synced setup=1 hooks=0 file_bytes=549942 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922528256 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 09:02:51.359 RX [LOG] measured=1 stack_min=3988 internal_min=34548 internal_largest=25588 dma_min=27052 dma_largest=25588 writes=20 slow=0 write_max_us=2280 flush_max_us=4704 sd_max_us=100422 rotations=0 pruned=0 oversized=0
2026-09-16 09:02:51.360 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 09:02:51.360 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 09:02:51.361 RX [LOG MEM] phase=before_clock up_us=1782940 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 09:02:51.361 RX [LOG MEM] phase=after_clock up_us=1783645 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:02:51.361 RX [LOG MEM] phase=before_writer up_us=1783692 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:02:51.362 RX [LOG MEM] phase=writer_entry up_us=1783898 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:02:51.362 RX [LOG MEM] phase=after_formatter up_us=1783967 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:02:51.362 RX [LOG MEM] phase=before_mount up_us=1784551 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:02:51.362 RX [LOG MEM] phase=after_mount up_us=1884853 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:02:51.362 RX [LOG MEM] phase=before_current_open up_us=1892161 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:02:51.362 RX [LOG MEM] phase=after_current_open up_us=1893569 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:02:51.362 RX [LOG MEM] phase=storage_done up_us=1907750 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:02:54.746 RX Screen touched, resetting inactivity timer.
2026-09-16 09:02:54.746 RX Latest button clicked
2026-09-16 09:02:54.746 RX Initiating async latest image request...
2026-09-16 09:02:54.746 RX Preparing UI for new image request...
2026-09-16 09:02:54.746 RX Cleaning up image fetcher state...
2026-09-16 09:02:54.746 RX Screen 2 Loaded.
2026-09-16 09:02:54.818 RX === requestImage('latest') START ===
2026-09-16 09:02:54.818 RX Sending HTTP GET...
2026-09-16 09:02:54.819 RX [PROBE] window=normal run=16 ms=5865 heap_min_boot=34548 largest_min=31732 interval_ms=10 samples=586 gap_max_us=10687 scan_max_us=212 timer=on imu_n=284 imu_min_hz=30.26 imu_avg_hz=49.17
2026-09-16 09:02:55.528 RX [PROBE] window=image_https run=2 ms=708 heap_min_boot=34548 largest_min=26612 interval_ms=10 samples=71 gap_max_us=10214 scan_max_us=155 timer=on
2026-09-16 09:02:55.528 RX Response received in 709 ms, Content-Length: 35085
2026-09-16 09:02:55.528 RX Starting to receive image data...
2026-09-16 09:02:55.905 RX Image download complete (35085 bytes, 1158 ms since button press). Starting decode...
2026-09-16 09:02:56.044 RX JPEG decoded successfully into PSRAM.
2026-09-16 09:02:56.045 RX LVGL image source updated. Total 1298 ms from button press (budget 20000 ms).
2026-09-16 09:02:59.872 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 09:02:59.873 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 09:03:03.274 TX log status [CRLF]
2026-09-16 09:03:03.275 RX [LOG] state=ready boot=17 session=boot-17 up_ms=850947 clock=synced setup=1 hooks=0 file_bytes=550504 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922528256 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 09:03:03.275 RX [LOG] measured=1 stack_min=3988 internal_min=34548 internal_largest=25588 dma_min=27052 dma_largest=25588 writes=21 slow=0 write_max_us=2280 flush_max_us=4704 sd_max_us=100422 rotations=0 pruned=0 oversized=0
2026-09-16 09:03:03.276 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 09:03:03.277 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 09:03:03.277 RX [LOG MEM] phase=before_clock up_us=1782940 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 09:03:03.277 RX [LOG MEM] phase=after_clock up_us=1783645 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:03:03.278 RX [LOG MEM] phase=before_writer up_us=1783692 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:03:03.278 RX [LOG MEM] phase=writer_entry up_us=1783898 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:03:03.278 RX [LOG MEM] phase=after_formatter up_us=1783967 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:03:03.278 RX [LOG MEM] phase=before_mount up_us=1784551 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:03:03.278 RX [LOG MEM] phase=after_mount up_us=1884853 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:03:03.278 RX [LOG MEM] phase=before_current_open up_us=1892161 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:03:03.278 RX [LOG MEM] phase=after_current_open up_us=1893569 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:03:03.278 RX [LOG MEM] phase=storage_done up_us=1907750 free=129996 largest=65524 heap_min_boot=128864
```


## Hotspot loss during Live and recovery passes - 2026-09-16, boot 17, 09:06-09:07

JP deliberately turned off the iPhone hotspot during Live, restored it, then
requested Latest. JP reported normal behavior on the device. This differs from
the preceding manual screen-exit test: this run exercised a network failure.
Patched B firmware and boot 17 were retained, with hooks off.

| Measurement or event | Result |
|----------------------|--------|
| Live start | 09:06:39.202 |
| Live TLS | 724 ms; largest_min=26612; 72 samples |
| Failure and return | 09:06:50.588 connection closed mid-response; fetch failed; screen 1 at 09:06:50.590 |
| Interrupted video | 31 frames in 11.4 s; 2.7 fps; first frame 1230 ms; max gap 660 ms |
| Full active-Live probe | 11386 ms; largest_min=26612; 1139 samples at 10 ms |
| Live maximum sampling gap / scan | 10881 / 980 us |
| Wi-Fi offline notification | 09:06:53.302 |
| Real MQTT reconnect | 1516 ms; largest_min=31732; 152 samples at 10 ms |
| Reconnect maximum sampling gap / scan | 10090 / 197 us |
| Remote connected notification | 09:07:28.682 |
| Following Latest HTTPS | 764 ms; largest_min=26612; 76 samples at 10 ms |
| Latest completion | 35085 bytes; decoded and displayed in 1367 ms |
| Historical internal minimum, before / after reconnect / after Latest | 34548 / 34548 / 34520 bytes |
| Historical largest block, all status readings | 25588 bytes |
| Writer used / margin, all status readings | 4204 / 3988 bytes |
| Logger state / errors / drops / queue peak | ready / none / zero / 1 of 16 |
| Writes / file bytes, before to after | 24 to 25 / 552192 to 552755 |

Live exited automatically on the expected fetch failure and returned to the
dashboard. MQTT reconnected, and Latest subsequently worked. All adequately
sampled operation windows remained above the unchanged 20480-byte floor.
The logger stayed healthy; the expected video fetch error is separate from
logger errors. Writer placement remained valid and active, with the 8192-byte
PSRAM stack and internal 352-byte TCB.

There was also an immediate mqtt_connect run 3 at 09:06:50.786, with ms=0,
samples=0 and largest_min=38900. Do not use it as evidence of memory headroom
through a TLS handshake. The actual successful recovery is run 4, followed by
calibration and green MQTT. A brief WiFi=CONNECTED/MQTT=DISCONNECTED notification
preceded the offline notification during loss detection.

The offline-to-green notification interval was 35.380 seconds. Actual hotspot
toggle times were not captured, so this is not a measured reconnect delay after
turning the hotspot on, nor an exact outage duration. No reset occurred in the
capture; boot stayed 17 and uptime advanced.

The normal window during outage measured largest_min=38900; the post-reconnect
normal window measured 31732. Normal IMU averages were 49.17, 48.73 and 49.16 Hz.
The 194 ms transitional normal window had no IMU samples; its zero rate fields
are not an IMU-rate failure. Historical internal_min fell only 28 bytes during
Latest; largest-block and writer stack minima did not worsen. This does not
establish a leak. Logger latency maxima stayed write 2280 us, flush 4704 us and
SD operation 100422 us, with zero slow writes.

This interrupted-Live recovery check passes. Back older-image retrieval remains
the next single test, with the hotspot kept on: log status, Back, confirm image,
return to dashboard, log status. Motion still-to-Live handover and the remaining
Stage 1 gates are pending. No firmware changes, builds, flashes or commits were
made for this result.

### Serial evidence

Endpoint and calibration payload lines are omitted.

```text
2026-09-16 09:06:22.875 EVENT Console cleared.
2026-09-16 09:06:35.093 TX log status [CRLF]
2026-09-16 09:06:35.097 RX [LOG] state=ready boot=17 session=boot-17 up_ms=1062766 clock=synced setup=1 hooks=0 file_bytes=552192 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922528256 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 09:06:35.097 RX [LOG] measured=1 stack_min=3988 internal_min=34548 internal_largest=25588 dma_min=27052 dma_largest=25588 writes=24 slow=0 write_max_us=2280 flush_max_us=4704 sd_max_us=100422 rotations=0 pruned=0 oversized=0
2026-09-16 09:06:35.098 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 09:06:35.098 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 09:06:35.099 RX [LOG MEM] phase=before_clock up_us=1782940 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 09:06:35.099 RX [LOG MEM] phase=after_clock up_us=1783645 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:06:35.099 RX [LOG MEM] phase=before_writer up_us=1783692 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:06:35.100 RX [LOG MEM] phase=writer_entry up_us=1783898 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:06:35.100 RX [LOG MEM] phase=after_formatter up_us=1783967 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:06:35.100 RX [LOG MEM] phase=before_mount up_us=1784551 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:06:35.100 RX [LOG MEM] phase=after_mount up_us=1884853 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:06:35.100 RX [LOG MEM] phase=before_current_open up_us=1892161 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:06:35.101 RX [LOG MEM] phase=after_current_open up_us=1893569 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:06:35.101 RX [LOG MEM] phase=storage_done up_us=1907750 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:06:39.202 RX Screen touched, resetting inactivity timer.
2026-09-16 09:06:39.202 RX Live button clicked -> starting live feed
2026-09-16 09:06:39.203 RX [PROBE] window=normal run=20 ms=43139 heap_min_boot=34548 largest_min=31732 interval_ms=10 samples=4314 gap_max_us=10829 scan_max_us=424 timer=on imu_n=2117 imu_min_hz=35.58 imu_avg_hz=49.17
2026-09-16 09:06:39.203 RX Screen 2 Loaded.
2026-09-16 09:06:39.274 RX Video: endpoint parsed, port 9835, path /esp32/live
2026-09-16 09:06:39.999 RX [PROBE] window=live_tls run=4 ms=724 heap_min_boot=34548 largest_min=26612 interval_ms=10 samples=72 gap_max_us=10070 scan_max_us=155 timer=on
2026-09-16 09:06:50.588 RX Video: connection closed mid-response
2026-09-16 09:06:50.588 RX Video: fetch failed, stopping
2026-09-16 09:06:50.589 RX Video: 31 frames in 11.4s (2.7 fps) | http 328 | decode 60 | blit 61 | frame 358 ms | first_frame 1230 | max_gap 660 ms
2026-09-16 09:06:50.589 RX Video: http = ttfb 136 + xfer 192 ms | frame 18.3 KB | 95 KB/s while transferring
2026-09-16 09:06:50.589 RX Video: free PSRAM 7611896, free heap 128836
2026-09-16 09:06:50.589 RX [PROBE] window=live run=3 ms=11386 heap_min_boot=34548 largest_min=26612 interval_ms=10 samples=1139 gap_max_us=10881 scan_max_us=980 timer=on
2026-09-16 09:06:50.590 RX Video: returning to previous screen
2026-09-16 09:06:50.590 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 09:06:50.590 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 09:06:50.785 RX [PROBE] window=normal run=21 ms=194 heap_min_boot=34548 largest_min=38900 interval_ms=10 samples=20 gap_max_us=10029 scan_max_us=126 timer=on imu_n=0 imu_min_hz=0.00 imu_avg_hz=0.00
2026-09-16 09:06:50.786 RX [PROBE] window=mqtt_connect run=3 ms=0 heap_min_boot=34548 largest_min=38900 interval_ms=10 samples=0 gap_max_us=875 scan_max_us=94 timer=on
2026-09-16 09:06:51.301 RX [NET] WiFi=CONNECTED | MQTT=DISCONNECTED
2026-09-16 09:06:53.302 RX [NET] WiFi=OFFLINE | MQTT=DISCONNECTED
2026-09-16 09:07:19.641 TX log status [CRLF]
2026-09-16 09:07:19.648 RX [LOG] state=ready boot=17 session=boot-17 up_ms=1107315 clock=synced setup=1 hooks=0 file_bytes=552755 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922528256 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 09:07:19.649 RX [LOG] measured=1 stack_min=3988 internal_min=34548 internal_largest=25588 dma_min=27052 dma_largest=25588 writes=25 slow=0 write_max_us=2280 flush_max_us=4704 sd_max_us=100422 rotations=0 pruned=0 oversized=0
2026-09-16 09:07:19.650 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 09:07:19.650 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 09:07:19.650 RX [LOG MEM] phase=before_clock up_us=1782940 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 09:07:19.651 RX [LOG MEM] phase=after_clock up_us=1783645 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:07:19.651 RX [LOG MEM] phase=before_writer up_us=1783692 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:07:19.651 RX [LOG MEM] phase=writer_entry up_us=1783898 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:07:19.651 RX [LOG MEM] phase=after_formatter up_us=1783967 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:07:19.651 RX [LOG MEM] phase=before_mount up_us=1784551 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:07:19.651 RX [LOG MEM] phase=after_mount up_us=1884853 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:07:19.651 RX [LOG MEM] phase=before_current_open up_us=1892161 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:07:19.651 RX [LOG MEM] phase=after_current_open up_us=1893569 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:07:19.651 RX [LOG MEM] phase=storage_done up_us=1907750 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:07:27.154 RX [PROBE] window=normal run=22 ms=36362 heap_min_boot=34548 largest_min=38900 interval_ms=10 samples=3636 gap_max_us=11418 scan_max_us=729 timer=on imu_n=1761 imu_min_hz=36.77 imu_avg_hz=48.73
2026-09-16 09:07:28.671 RX [PROBE] window=mqtt_connect run=4 ms=1516 heap_min_boot=34548 largest_min=31732 interval_ms=10 samples=152 gap_max_us=10090 scan_max_us=197 timer=on
2026-09-16 09:07:28.682 RX [NET] WiFi=CONNECTED | MQTT=REMOTE CONNECTED
2026-09-16 09:07:33.180 TX log status [CRLF]
2026-09-16 09:07:33.184 RX [LOG] state=ready boot=17 session=boot-17 up_ms=1120852 clock=synced setup=1 hooks=0 file_bytes=552755 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922528256 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 09:07:33.186 RX [LOG] measured=1 stack_min=3988 internal_min=34548 internal_largest=25588 dma_min=27052 dma_largest=25588 writes=25 slow=0 write_max_us=2280 flush_max_us=4704 sd_max_us=100422 rotations=0 pruned=0 oversized=0
2026-09-16 09:07:33.186 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 09:07:33.186 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 09:07:33.186 RX [LOG MEM] phase=before_clock up_us=1782940 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 09:07:33.186 RX [LOG MEM] phase=after_clock up_us=1783645 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:07:33.187 RX [LOG MEM] phase=before_writer up_us=1783692 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:07:33.187 RX [LOG MEM] phase=writer_entry up_us=1783898 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:07:33.188 RX [LOG MEM] phase=after_formatter up_us=1783967 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:07:33.188 RX [LOG MEM] phase=before_mount up_us=1784551 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:07:33.188 RX [LOG MEM] phase=after_mount up_us=1884853 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:07:33.188 RX [LOG MEM] phase=before_current_open up_us=1892161 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:07:33.188 RX [LOG MEM] phase=after_current_open up_us=1893569 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:07:33.188 RX [LOG MEM] phase=storage_done up_us=1907750 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:07:38.706 RX Screen touched, resetting inactivity timer.
2026-09-16 09:07:38.706 RX Latest button clicked
2026-09-16 09:07:38.706 RX Initiating async latest image request...
2026-09-16 09:07:38.706 RX Preparing UI for new image request...
2026-09-16 09:07:38.706 RX Cleaning up image fetcher state...
2026-09-16 09:07:38.706 RX Screen 2 Loaded.
2026-09-16 09:07:38.778 RX === requestImage('latest') START ===
2026-09-16 09:07:38.778 RX Sending HTTP GET...
2026-09-16 09:07:38.779 RX [PROBE] window=normal run=23 ms=10100 heap_min_boot=34548 largest_min=31732 interval_ms=10 samples=1010 gap_max_us=10480 scan_max_us=396 timer=on imu_n=492 imu_min_hz=38.31 imu_avg_hz=49.16
2026-09-16 09:07:39.544 RX [PROBE] window=image_https run=3 ms=764 heap_min_boot=34520 largest_min=26612 interval_ms=10 samples=76 gap_max_us=10286 scan_max_us=140 timer=on
2026-09-16 09:07:39.544 RX Response received in 765 ms, Content-Length: 35085
2026-09-16 09:07:39.544 RX Starting to receive image data...
2026-09-16 09:07:39.934 RX Image download complete (35085 bytes, 1227 ms since button press). Starting decode...
2026-09-16 09:07:40.073 RX JPEG decoded successfully into PSRAM.
2026-09-16 09:07:40.073 RX LVGL image source updated. Total 1367 ms from button press (budget 20000 ms).
2026-09-16 09:07:45.156 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 09:07:45.157 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 09:07:48.457 TX log status [CRLF]
2026-09-16 09:07:48.458 RX [LOG] state=ready boot=17 session=boot-17 up_ms=1136126 clock=synced setup=1 hooks=0 file_bytes=552755 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922528256 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 09:07:48.458 RX [LOG] measured=1 stack_min=3988 internal_min=34520 internal_largest=25588 dma_min=27024 dma_largest=25588 writes=25 slow=0 write_max_us=2280 flush_max_us=4704 sd_max_us=100422 rotations=0 pruned=0 oversized=0
2026-09-16 09:07:48.460 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 09:07:48.460 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 09:07:48.461 RX [LOG MEM] phase=before_clock up_us=1782940 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 09:07:48.461 RX [LOG MEM] phase=after_clock up_us=1783645 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:07:48.462 RX [LOG MEM] phase=before_writer up_us=1783692 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:07:48.462 RX [LOG MEM] phase=writer_entry up_us=1783898 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:07:48.462 RX [LOG MEM] phase=after_formatter up_us=1783967 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:07:48.462 RX [LOG MEM] phase=before_mount up_us=1784551 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:07:48.463 RX [LOG MEM] phase=after_mount up_us=1884853 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:07:48.463 RX [LOG MEM] phase=before_current_open up_us=1892161 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:07:48.463 RX [LOG MEM] phase=after_current_open up_us=1893569 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:07:48.463 RX [LOG MEM] phase=storage_done up_us=1907750 free=129996 largest=65524 heap_min_boot=128864
```


## Back older-image request passes - 2026-09-16, boot 17, 09:11

JP confirmed that Back worked and the image was visible. Patched B firmware,
boot 17 and hooks-off settings were unchanged. No hotspot interruption was
requested in this test.

| Measurement | Result |
|-------------|--------|
| Request | requestImage('back') at 09:11:40.859 |
| HTTPS window | 868 ms; largest_min=26612; 87 samples at 10 ms |
| Maximum sampling gap / scan | 10373 / 288 us |
| Response / image size | 870 ms / 33949 bytes |
| Download / total display time from button | 1288 / 1427 ms |
| Return | screen 1, no preference save needed |
| Historical internal minimum / largest block | 34520 / 25588 bytes, unchanged |
| DMA minimum / largest block | 27024 / 25588 bytes, unchanged |
| Writer used / margin | 4204 / 3988 bytes, unchanged |
| Logger state / errors / drops / queue peak | ready / none / zero / 1 of 16 |
| Writes / file bytes, before and after | 29 / 555011 |
| Maximum write / flush / SD operation | 2421 / 4704 / 100422 us |

Back passed the unchanged 20480-byte floor with 6132 bytes of margin.
It decoded and displayed successfully and returned normally. No new historical
memory low, stack-margin loss, reset or logger error was recorded. Placement
remained valid and active: 8192-byte PSRAM stack, internal TCB 352 bytes.
The normal windows measured largest_min=31732, with IMU averages 49.16 and
48.96 Hz and minima 28.56 and 27.03 Hz. This is a functional and memory pass;
it is not a logging-on/off performance comparison.

JP confirmed that he can trigger the driveway camera detection for the next
automatic still-to-Live test. The firmware receives payload latest on its image
MQTT topic, calls requestLatestImage(true), displays the still for about one
second and starts Live. Moving the companion's IMU is a different trigger.
Wait at least 15 seconds after the last displayed image to avoid the existing
10-second image-notification echo suppression window.

Next single test: keep Wi-Fi on and MQTT green, send log status, trigger one
camera detection, let the still and automatic 60-second Live sequence complete,
then send log status. Do not use Latest or Live buttons to substitute for the
automatic transition. Capture the line "Motion still shown, starting live feed",
image_https and live_tls probes, the full Video and Live summaries, and any
visible delay or failure. If no notification reaches the unit, send the output;
do not count a manual Live start as an automatic-handover pass.

Require operation-window block minima at least 20480, successful still display
and automatic video, valid placement, adequate stack margin and no logger errors
or drops. Record first-frame timing without inventing a latency acceptance limit.
The remaining Stage 1 gates are still pending. Documentation only for this
result; no build, flash, firmware edit, commit or push.

### Serial evidence

The endpoint line is omitted.

```text
2026-09-16 09:11:23.362 EVENT Console cleared.
2026-09-16 09:11:38.623 TX log status [CRLF]
2026-09-16 09:11:38.626 RX [LOG] state=ready boot=17 session=boot-17 up_ms=1366294 clock=synced setup=1 hooks=0 file_bytes=555011 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922528256 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 09:11:38.627 RX [LOG] measured=1 stack_min=3988 internal_min=34520 internal_largest=25588 dma_min=27024 dma_largest=25588 writes=29 slow=0 write_max_us=2421 flush_max_us=4704 sd_max_us=100422 rotations=0 pruned=0 oversized=0
2026-09-16 09:11:38.628 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 09:11:38.628 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 09:11:38.628 RX [LOG MEM] phase=before_clock up_us=1782940 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 09:11:38.629 RX [LOG MEM] phase=after_clock up_us=1783645 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:11:38.629 RX [LOG MEM] phase=before_writer up_us=1783692 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:11:38.629 RX [LOG MEM] phase=writer_entry up_us=1783898 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:11:38.629 RX [LOG MEM] phase=after_formatter up_us=1783967 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:11:38.629 RX [LOG MEM] phase=before_mount up_us=1784551 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:11:38.629 RX [LOG MEM] phase=after_mount up_us=1884853 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:11:38.629 RX [LOG MEM] phase=before_current_open up_us=1892161 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:11:38.629 RX [LOG MEM] phase=after_current_open up_us=1893569 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:11:38.630 RX [LOG MEM] phase=storage_done up_us=1907750 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:11:40.084 RX [PROBE] window=normal run=27 ms=60000 heap_min_boot=34520 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10939 scan_max_us=404 timer=on imu_n=2943 imu_min_hz=28.56 imu_avg_hz=49.16
2026-09-16 09:11:40.787 RX Screen touched, resetting inactivity timer.
2026-09-16 09:11:40.787 RX Back button clicked, initiating async request...
2026-09-16 09:11:40.787 RX Preparing UI for new image request...
2026-09-16 09:11:40.787 RX Cleaning up image fetcher state...
2026-09-16 09:11:40.787 RX Screen 2 Loaded.
2026-09-16 09:11:40.859 RX === requestImage('back') START ===
2026-09-16 09:11:40.860 RX Sending HTTP GET...
2026-09-16 09:11:40.861 RX [PROBE] window=normal run=28 ms=774 heap_min_boot=34520 largest_min=31732 interval_ms=10 samples=78 gap_max_us=10061 scan_max_us=157 timer=on imu_n=33 imu_min_hz=27.03 imu_avg_hz=48.96
2026-09-16 09:11:41.730 RX [PROBE] window=image_https run=4 ms=868 heap_min_boot=34520 largest_min=26612 interval_ms=10 samples=87 gap_max_us=10373 scan_max_us=288 timer=on
2026-09-16 09:11:41.730 RX Response received in 870 ms, Content-Length: 33949
2026-09-16 09:11:41.730 RX Starting to receive image data...
2026-09-16 09:11:42.075 RX Image download complete (33949 bytes, 1288 ms since button press). Starting decode...
2026-09-16 09:11:42.214 RX JPEG decoded successfully into PSRAM.
2026-09-16 09:11:42.214 RX LVGL image source updated. Total 1427 ms from button press (budget 20000 ms).
2026-09-16 09:11:44.853 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 09:11:44.853 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 09:11:47.567 TX log status [CRLF]
2026-09-16 09:11:47.570 RX [LOG] state=ready boot=17 session=boot-17 up_ms=1375237 clock=synced setup=1 hooks=0 file_bytes=555011 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922528256 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 09:11:47.571 RX [LOG] measured=1 stack_min=3988 internal_min=34520 internal_largest=25588 dma_min=27024 dma_largest=25588 writes=29 slow=0 write_max_us=2421 flush_max_us=4704 sd_max_us=100422 rotations=0 pruned=0 oversized=0
2026-09-16 09:11:47.572 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 09:11:47.572 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 09:11:47.573 RX [LOG MEM] phase=before_clock up_us=1782940 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 09:11:47.573 RX [LOG MEM] phase=after_clock up_us=1783645 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:11:47.573 RX [LOG MEM] phase=before_writer up_us=1783692 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:11:47.574 RX [LOG MEM] phase=writer_entry up_us=1783898 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:11:47.574 RX [LOG MEM] phase=after_formatter up_us=1783967 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:11:47.574 RX [LOG MEM] phase=before_mount up_us=1784551 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:11:47.574 RX [LOG MEM] phase=after_mount up_us=1884853 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:11:47.574 RX [LOG MEM] phase=before_current_open up_us=1892161 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:11:47.574 RX [LOG MEM] phase=after_current_open up_us=1893569 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:11:47.574 RX [LOG MEM] phase=storage_done up_us=1907750 free=129996 largest=65524 heap_min_boot=128864
```


## Automatic camera still-to-Live handover passes - 2026-09-16, boot 17, 09:23-09:24

JP triggered the driveway camera and reported a normal full cycle on screen.
The capture includes "Motion still shown, starting live feed", confirming the
automatic notification path rather than a manual Live-button substitute.
Patched B, boot 17 and hooks-off settings remained unchanged.

| Measurement or event | Result |
|----------------------|--------|
| Automatic still request | 09:23:17.592 |
| Still HTTPS | 760 ms; largest_min=25588; 76 samples at 10 ms |
| Still completion | 33629 bytes; displayed at 09:23:18.977, 1384 ms from request |
| Automatic Live start | 09:23:19.978, 1001 ms after still display |
| First Live TLS | 651 ms; largest_min=26612; 65 samples |
| Second Live TLS | 629 ms; largest_min=26612; 63 samples |
| Video | 195 frames in 60.3 s; reported 3.2 fps |
| First video frame / maximum frame gap | 1036 / 921 ms |
| Average HTTP / decode / blit / frame | 302 / 59 / 62 / 309 ms |
| HTTP TTFB / transfer / average frame / transfer rate | 131 ms / 171 ms / 17.9 KB / 105 KB/s |
| Full Live probe | 60316 ms; largest_min=25588; 6031 samples at 10 ms |
| Live maximum sampling gap / scan | 11471 / 1372 us |
| Historical internal minimum, before to after | 34520 to 34388 bytes |
| Historical largest block, before and after | 25588 bytes |
| Writer used / margin, before and after | 4204 / 3988 bytes |
| Logger state / errors / drops / queue peak | ready / none / zero / 1 of 16 |
| Writes / file bytes, before to after | 41 to 42 / 561778 to 562342 |
| Maximum write / flush / SD operation | 2642 / 5894 / 100422 us |

The still and full Live windows passed the unchanged 20480-byte floor with
5108 bytes of margin; both TLS windows had 6132 bytes of margin.
The 1036 ms first-frame time is measured from Live start, after the intentional
one-second still display. It is not camera-detection-to-display latency.
The older "since button press" output label also appears on the automatic
request; no button press is implied by that wording.

JP saw a normal transition despite the fresh TLS handshake. The unit returned
to screen 1 without saving a preference. Placement stayed valid and active
(8192-byte PSRAM stack, internal 352-byte TCB). No reset, logger error, queue
drop, slow write or stack-margin loss was recorded.
Historical internal_min decreased 132 bytes during Live; the largest-block
minimum remained 25588. This alone does not establish a leak.
The end-of-video free PSRAM reading was 7280592 and free heap 46544; it precedes
screen unload, so it is not a post-cleanup memory inventory.
Normal IMU averages were 49.23 and 48.72 Hz; minima 37.20 and 15.90 Hz.
The short second normal window covered the still display before Live.

The targeted cleanup checks now pass for the tested patched boot: Latest then
hotspot recovery, full Live, manual early Live exit then Latest, hotspot loss
during Live then recovery and Latest, Back, and automatic still-to-Live.
These results support retaining the explicit still-body TLS close. They do not
trace the original overnight allocation or establish the paired performance gate.

Recommended next checkpoint: review the cleanup and recorded results with JP,
then proceed to the remaining Stage 1 gates. First obtain the required same-session
baseline and logging-on performance comparison; then run the separate hooks-phase
NVS/cache-safety and storage, clock, breadcrumb and shutdown checks.
Keep the 20480-byte floor and identical Stage 0 probes. Stage 1 is not accepted
and Stage 1B retrieval has not started. No new firmware edits, build, flash,
commit or push were made for this result.

### Serial evidence

The endpoint line is omitted.

```text
2026-09-16 09:23:01.946 EVENT Console cleared.
2026-09-16 09:23:07.485 TX log status [CRLF]
2026-09-16 09:23:07.492 RX [LOG] state=ready boot=17 session=boot-17 up_ms=2055159 clock=synced setup=1 hooks=0 file_bytes=561778 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922495488 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 09:23:07.492 RX [LOG] measured=1 stack_min=3988 internal_min=34520 internal_largest=25588 dma_min=27024 dma_largest=25588 writes=41 slow=0 write_max_us=2642 flush_max_us=5894 sd_max_us=100422 rotations=0 pruned=0 oversized=0
2026-09-16 09:23:07.493 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 09:23:07.494 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 09:23:07.494 RX [LOG MEM] phase=before_clock up_us=1782940 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 09:23:07.494 RX [LOG MEM] phase=after_clock up_us=1783645 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:23:07.494 RX [LOG MEM] phase=before_writer up_us=1783692 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:23:07.496 RX [LOG MEM] phase=writer_entry up_us=1783898 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:23:07.496 RX [LOG MEM] phase=after_formatter up_us=1783967 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:23:07.496 RX [LOG MEM] phase=before_mount up_us=1784551 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:23:07.496 RX [LOG MEM] phase=after_mount up_us=1884853 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:23:07.496 RX [LOG MEM] phase=before_current_open up_us=1892161 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:23:07.496 RX [LOG MEM] phase=after_current_open up_us=1893569 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:23:07.496 RX [LOG MEM] phase=storage_done up_us=1907750 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:23:17.592 RX Initiating async latest image request...
2026-09-16 09:23:17.592 RX Preparing UI for new image request...
2026-09-16 09:23:17.592 RX Cleaning up image fetcher state...
2026-09-16 09:23:17.592 RX Screen 2 Loaded.
2026-09-16 09:23:17.665 RX === requestImage('latest') START ===
2026-09-16 09:23:17.665 RX Sending HTTP GET...
2026-09-16 09:23:17.666 RX [PROBE] window=normal run=40 ms=35400 heap_min_boot=34520 largest_min=31732 interval_ms=10 samples=3540 gap_max_us=10629 scan_max_us=392 timer=on imu_n=1735 imu_min_hz=37.20 imu_avg_hz=49.23
2026-09-16 09:23:18.427 RX [PROBE] window=image_https run=5 ms=760 heap_min_boot=34520 largest_min=25588 interval_ms=10 samples=76 gap_max_us=10057 scan_max_us=132 timer=on
2026-09-16 09:23:18.427 RX Response received in 762 ms, Content-Length: 33629
2026-09-16 09:23:18.427 RX Starting to receive image data...
2026-09-16 09:23:18.839 RX Image download complete (33629 bytes, 1246 ms since button press). Starting decode...
2026-09-16 09:23:18.977 RX JPEG decoded successfully into PSRAM.
2026-09-16 09:23:18.977 RX LVGL image source updated. Total 1384 ms from button press (budget 20000 ms).
2026-09-16 09:23:19.978 RX Motion still shown, starting live feed
2026-09-16 09:23:19.978 RX [PROBE] window=normal run=41 ms=999 heap_min_boot=34520 largest_min=31732 interval_ms=10 samples=100 gap_max_us=10233 scan_max_us=155 timer=on imu_n=47 imu_min_hz=15.90 imu_avg_hz=48.72
2026-09-16 09:23:19.979 RX Video: endpoint parsed, port 9835, path /esp32/live
2026-09-16 09:23:20.631 RX [PROBE] window=live_tls run=5 ms=651 heap_min_boot=34520 largest_min=26612 interval_ms=10 samples=65 gap_max_us=10079 scan_max_us=139 timer=on
2026-09-16 09:23:52.093 RX [PROBE] window=live_tls run=6 ms=629 heap_min_boot=34388 largest_min=26612 interval_ms=10 samples=63 gap_max_us=10520 scan_max_us=131 timer=on
2026-09-16 09:24:20.293 RX Video: 195 frames in 60.3s (3.2 fps) | http 302 | decode 59 | blit 62 | frame 309 ms | first_frame 1036 | max_gap 921 ms
2026-09-16 09:24:20.294 RX Video: http = ttfb 131 + xfer 171 ms | frame 17.9 KB | 105 KB/s while transferring
2026-09-16 09:24:20.294 RX Video: free PSRAM 7280592, free heap 46544
2026-09-16 09:24:20.295 RX [PROBE] window=live run=4 ms=60316 heap_min_boot=34388 largest_min=25588 interval_ms=10 samples=6031 gap_max_us=11471 scan_max_us=1372 timer=on
2026-09-16 09:24:20.295 RX Video: returning to previous screen
2026-09-16 09:24:20.296 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 09:24:20.296 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 09:24:23.214 TX log status [CRLF]
2026-09-16 09:24:23.217 RX [LOG] state=ready boot=17 session=boot-17 up_ms=2130885 clock=synced setup=1 hooks=0 file_bytes=562342 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922495488 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 09:24:23.219 RX [LOG] measured=1 stack_min=3988 internal_min=34388 internal_largest=25588 dma_min=26892 dma_largest=25588 writes=42 slow=0 write_max_us=2642 flush_max_us=5894 sd_max_us=100422 rotations=0 pruned=0 oversized=0
2026-09-16 09:24:23.219 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 09:24:23.219 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 09:24:23.220 RX [LOG MEM] phase=before_clock up_us=1782940 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 09:24:23.220 RX [LOG MEM] phase=after_clock up_us=1783645 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:24:23.220 RX [LOG MEM] phase=before_writer up_us=1783692 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:24:23.220 RX [LOG MEM] phase=writer_entry up_us=1783898 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:24:23.220 RX [LOG MEM] phase=after_formatter up_us=1783967 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:24:23.221 RX [LOG MEM] phase=before_mount up_us=1784551 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:24:23.221 RX [LOG MEM] phase=after_mount up_us=1884853 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:24:23.221 RX [LOG MEM] phase=before_current_open up_us=1892161 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:24:23.221 RX [LOG MEM] phase=after_current_open up_us=1893569 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:24:23.221 RX [LOG MEM] phase=storage_done up_us=1907750 free=129996 largest=65524 heap_min_boot=128864
```


## Same-session performance pair: logging OFF baseline - 2026-09-16, 09:37-09:39

JP compiled the patched source with DIAG_ENABLED=0, DIAG_WRITER_STACK_PSRAM=1
and DIAG_TEST_HOOKS=0. JP reported normal operation throughout. The working
configuration and serial state=off both confirm the disabled logger. The probes
remain active. This is the OFF half only; the enabled comparison is pending.

| Measurement | Logging OFF baseline |
|-------------|----------------------|
| Full normal window | 60001 ms; 6001 memory samples at 10 ms |
| Normal internal minimum / largest block | 87448 / 31732 bytes |
| Normal IMU samples / minimum / average | 2950 / 28.55 Hz / 49.26 Hz |
| Normal maximum sampling gap / scan | 10545 / 421 us |
| Latest HTTPS | 744 ms; largest_min=29684; 74 samples |
| Latest image / completion | 33629 bytes / 1287 ms |
| First Live TLS | 630 ms; largest_min=28660; 63 samples |
| Second Live TLS | 614 ms; largest_min=29684; 61 samples |
| Full Live | 200 frames; 60132 ms probe duration; printed 60.1 s and 3.3 fps |
| Derived Live rate | approximately 3.326 fps using 200 / 60.132 |
| First frame / maximum frame gap | 1067 / 987 ms |
| Average HTTP / decode / blit / frame | 293 / 59 / 62 / 301 ms |
| HTTP TTFB / transfer / average JPEG / transfer rate | 129 ms / 164 ms / 18.3 KB / 111 KB/s |
| Full Live historical internal minimum / largest block | 36732 / 27636 bytes |
| Full Live sampling | 6013 samples; 10 ms interval; max gap 11160 us; max scan 1255 us |
| End Video free PSRAM / free heap | 7647136 / 92796 bytes |

All measured media block minima exceed 20480. No image or Live failure appears
in the capture. Latest and Live returned to screen 1 without a preference save.
The additional short normal windows include UI interaction and are not substitutes
for the full idle minute: IMU averages 49.03 and 48.77 Hz, minima 29.40 and 10.09 Hz.

The logger's boot=0, clock=unknown, measured=0, zero memory fields, absent startup
snapshots and inactive writer placement are expected with DIAG_ENABLED=0.
Those zeros are unavailable logger measurements, not exhausted memory or placement
failures. Use the PROBE records for this baseline. Configured stack_mode=psram
does not mean a writer task was created; writer_lifecycle=off confirms it was not.

Next: JP restores only DIAG_ENABLED=1 and repeats status, a full untouched normal
minute, Latest, full Live, status in the same sitting. Keep the card, hotspot,
power, scene, PSRAM selection and hooks-off settings unchanged. Require ready and
valid writer placement in the ON capture. At this baseline, a roughly 5% lower
fps bound is about 3.160 fps; compare frame counts and duration, not just rounded
3.3-fps output. Also compare image size, transfer rate, IMU samples/minimum/average,
block minima, first-frame delay, maximum frame gap, drops and stack margin.

Do not use the earlier morning logging-on captures as this pair's ON half.
Logging overhead and Stage 1 acceptance remain pending the matching capture.
No firmware edit, build, flash or commit by the assistant for this result.

### Serial evidence

The endpoint line is omitted.

```text
2026-09-16 09:37:18.717 EVENT Console cleared.
2026-09-16 09:37:25.437 TX log status [CRLF]
2026-09-16 09:37:25.441 RX [LOG] state=off boot=0 session= up_ms=32663 clock=unknown setup=0 hooks=0 file_bytes=0 generation=0 newest=0 archives=0 card_bytes=0 free_bytes=0 queue=0/16 high=0 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 09:37:25.442 RX [LOG] measured=0 stack_min=0 internal_min=0 internal_largest=0 dma_min=0 dma_largest=0 writes=0 slow=0 write_max_us=0 flush_max_us=0 sd_max_us=0 rotations=0 pruned=0 oversized=0
2026-09-16 09:37:25.442 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=0 stack_start=0x0 stack_external=-1 stack_local_external=-1 tcb_internal=-1 tcb_bytes=352 writer_lifecycle=off stack_used_max=-1 stack_final_margin=-1
2026-09-16 09:37:25.443 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 09:37:25.443 RX [LOG MEM] phase=before_clock captured=0
2026-09-16 09:37:25.443 RX [LOG MEM] phase=after_clock captured=0
2026-09-16 09:37:25.443 RX [LOG MEM] phase=before_writer captured=0
2026-09-16 09:37:25.443 RX [LOG MEM] phase=writer_entry captured=0
2026-09-16 09:37:25.443 RX [LOG MEM] phase=after_formatter captured=0
2026-09-16 09:37:25.443 RX [LOG MEM] phase=before_mount captured=0
2026-09-16 09:37:25.443 RX [LOG MEM] phase=after_mount captured=0
2026-09-16 09:37:25.443 RX [LOG MEM] phase=before_current_open captured=0
2026-09-16 09:37:25.443 RX [LOG MEM] phase=after_current_open captured=0
2026-09-16 09:37:25.444 RX [LOG MEM] phase=storage_done captured=0
2026-09-16 09:38:01.233 RX [PROBE] window=normal run=1 ms=60001 heap_min_boot=87448 largest_min=31732 interval_ms=10 samples=6001 gap_max_us=10545 scan_max_us=421 timer=on imu_n=2950 imu_min_hz=28.55 imu_avg_hz=49.26
2026-09-16 09:38:07.321 RX Screen touched, resetting inactivity timer.
2026-09-16 09:38:07.322 RX Latest button clicked
2026-09-16 09:38:07.322 RX Initiating async latest image request...
2026-09-16 09:38:07.322 RX Preparing UI for new image request...
2026-09-16 09:38:07.322 RX Cleaning up image fetcher state...
2026-09-16 09:38:07.322 RX Screen 2 Loaded.
2026-09-16 09:38:07.393 RX === requestImage('latest') START ===
2026-09-16 09:38:07.394 RX Sending HTTP GET...
2026-09-16 09:38:07.395 RX [PROBE] window=normal run=2 ms=6161 heap_min_boot=87448 largest_min=31732 interval_ms=10 samples=616 gap_max_us=10178 scan_max_us=294 timer=on imu_n=297 imu_min_hz=29.40 imu_avg_hz=49.03
2026-09-16 09:38:08.140 RX [PROBE] window=image_https run=1 ms=744 heap_min_boot=37652 largest_min=29684 interval_ms=10 samples=74 gap_max_us=10203 scan_max_us=305 timer=on
2026-09-16 09:38:08.140 RX Response received in 746 ms, Content-Length: 33629
2026-09-16 09:38:08.140 RX Starting to receive image data...
2026-09-16 09:38:08.472 RX Image download complete (33629 bytes, 1149 ms since button press). Starting decode...
2026-09-16 09:38:08.609 RX JPEG decoded successfully into PSRAM.
2026-09-16 09:38:08.609 RX LVGL image source updated. Total 1287 ms from button press (budget 20000 ms).
2026-09-16 09:38:11.234 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 09:38:11.234 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 09:38:14.330 RX Screen touched, resetting inactivity timer.
2026-09-16 09:38:14.330 RX Live button clicked -> starting live feed
2026-09-16 09:38:14.331 RX [PROBE] window=normal run=3 ms=5720 heap_min_boot=37652 largest_min=31732 interval_ms=10 samples=572 gap_max_us=10472 scan_max_us=248 timer=on imu_n=271 imu_min_hz=10.09 imu_avg_hz=48.77
2026-09-16 09:38:14.331 RX Screen 2 Loaded.
2026-09-16 09:38:14.402 RX Video: endpoint parsed, port 9835, path /esp32/live
2026-09-16 09:38:15.034 RX [PROBE] window=live_tls run=1 ms=630 heap_min_boot=37600 largest_min=28660 interval_ms=10 samples=63 gap_max_us=10391 scan_max_us=342 timer=on
2026-09-16 09:38:15.276 RX Video: frame 432x768, panel 368x448 -> gap x=-64 y=-320, pan x=-15 y=32
2026-09-16 09:38:44.747 RX [PROBE] window=live_tls run=2 ms=614 heap_min_boot=36732 largest_min=29684 interval_ms=10 samples=61 gap_max_us=10631 scan_max_us=146 timer=on
2026-09-16 09:39:14.463 RX Video: 200 frames in 60.1s (3.3 fps) | http 293 | decode 59 | blit 62 | frame 301 ms | first_frame 1067 | max_gap 987 ms
2026-09-16 09:39:14.463 RX Video: http = ttfb 129 + xfer 164 ms | frame 18.3 KB | 111 KB/s while transferring
2026-09-16 09:39:14.463 RX Video: free PSRAM 7647136, free heap 92796
2026-09-16 09:39:14.464 RX [PROBE] window=live run=1 ms=60132 heap_min_boot=36732 largest_min=27636 interval_ms=10 samples=6013 gap_max_us=11160 scan_max_us=1255 timer=on
2026-09-16 09:39:14.464 RX Video: returning to previous screen
2026-09-16 09:39:14.464 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 09:39:14.464 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 09:39:18.570 TX log status [CRLF]
2026-09-16 09:39:18.573 RX [LOG] state=off boot=0 session= up_ms=145795 clock=unknown setup=0 hooks=0 file_bytes=0 generation=0 newest=0 archives=0 card_bytes=0 free_bytes=0 queue=0/16 high=0 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 09:39:18.573 RX [LOG] measured=0 stack_min=0 internal_min=0 internal_largest=0 dma_min=0 dma_largest=0 writes=0 slow=0 write_max_us=0 flush_max_us=0 sd_max_us=0 rotations=0 pruned=0 oversized=0
2026-09-16 09:39:18.574 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=0 stack_start=0x0 stack_external=-1 stack_local_external=-1 tcb_internal=-1 tcb_bytes=352 writer_lifecycle=off stack_used_max=-1 stack_final_margin=-1
2026-09-16 09:39:18.575 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 09:39:18.575 RX [LOG MEM] phase=before_clock captured=0
2026-09-16 09:39:18.575 RX [LOG MEM] phase=after_clock captured=0
2026-09-16 09:39:18.575 RX [LOG MEM] phase=before_writer captured=0
2026-09-16 09:39:18.575 RX [LOG MEM] phase=writer_entry captured=0
2026-09-16 09:39:18.575 RX [LOG MEM] phase=after_formatter captured=0
2026-09-16 09:39:18.575 RX [LOG MEM] phase=before_mount captured=0
2026-09-16 09:39:18.575 RX [LOG MEM] phase=after_mount captured=0
2026-09-16 09:39:18.575 RX [LOG MEM] phase=before_current_open captured=0
2026-09-16 09:39:18.576 RX [LOG MEM] phase=after_current_open captured=0
2026-09-16 09:39:18.576 RX [LOG MEM] phase=storage_done captured=0
```


## Same-session performance pair passes - logging ON, 2026-09-16, 09:43-09:45

JP restored DIAG_ENABLED=1, keeping DIAG_WRITER_STACK_PSRAM=1 and
DIAG_TEST_HOOKS=0, and reported normal operation throughout. Logger boot 19
was ready and synced, with valid active PSRAM placement. This completes the
same-sitting pair with the 09:37-09:39 OFF capture on the same patched source.

| Measurement | Logging OFF | Logging ON |
|-------------|-------------|------------|
| Normal full window | 60001 ms | 60001 ms |
| IMU samples / average / minimum | 2950 / 49.26 Hz / 28.55 Hz | 2945 / 49.18 Hz / 35.58 Hz |
| Normal historical internal minimum / largest block | 87448 / 31732 bytes | 84896 / 31732 bytes |
| Latest image size | 33629 bytes | 33629 bytes |
| Latest HTTPS time / samples / largest_min | 744 ms / 74 / 29684 | 706 ms / 71 / 26612 |
| Latest total time | 1287 ms | 1260 ms |
| Live frames / probe duration | 200 / 60132 ms | 194 / 60417 ms |
| Derived Live fps | 3.326 | 3.211 |
| First frame / maximum frame gap | 1067 / 987 ms | 1085 / 931 ms |
| First Live TLS time / samples / largest_min | 630 ms / 63 / 28660 | 634 ms / 63 / 26612 |
| Second Live TLS time / samples / largest_min | 614 ms / 61 / 29684 | 619 ms / 62 / 26612 |
| Full Live largest_min / samples | 27636 / 6013 | 26612 / 6042 |
| Full Live historical internal_min | 36732 bytes | 34728 bytes |
| Live maximum sampling gap / scan | 11160 / 1255 us | 11506 / 1257 us |
| Average HTTP / decode / blit / frame | 293 / 59 / 62 / 301 ms | 304 / 74 / 62 / 311 ms |
| TTFB / transfer | 129 / 164 ms | 145 / 159 ms |
| Average frame size / transfer rate | 18.3 KB / 111 KB/s | 18.3 KB / 115 KB/s |

Using frame counts and probe durations, the Live fps difference is -3.4576%,
within the approximately 5% gate. Average normal IMU rate differs by -0.08 Hz
(-0.1624%); the observed minimum is higher in ON. Latest is 27 ms faster in ON.
These are measurements of this pair, not evidence that logging improves image
speed or that every timing difference is caused by logging. Decode increased
from 59 to 74 ms and TTFB from 129 to 145 ms; JPEG content and scheduling can
vary even when rounded average image sizes match. No repeat is required solely
because those individual components differ while the agreed fps gate passes.

All measured ON image/TLS/Live block minima were 26612, leaving 6132 bytes
above the unchanged 20480 floor. Stage 0 probes retained 10 ms sampling.
Logger errors, queue drops, suppressed/truncated records and slow writes stayed
zero; queue peak was 1 of 16. Writer used/margin stayed 4204/3988 bytes.
Placement remained valid: 8192-byte PSRAM stack, internal 352-byte TCB.
Writes grew 7 to 9 and file bytes 571507 to 572622. Maximum write/flush/SD times
were 2851/4690/97190 us. Final DMA minimum/largest were 27232/26612 bytes.
There was no reset or media failure in the capture. JP saw normal behavior.

Some browser RX timestamps group multiple buffered lines. Use the firmware's
elapsed times for request timing, rather than interpreting receive timestamps
as precise stage boundaries. The same-session performance and tested media
memory checks pass. This does not complete all of Stage 1 or PSRAM-stack safety.

Next single test is the accepted hooks-only NVS/SD stress: on a backed-up test
card, JP enables DIAG_TEST_HOOKS=1, retaining DIAG_ENABLED=1 and
DIAG_WRITER_STACK_PSRAM=1. Compile/flash, wait for green real MQTT and ready logger,
send log status, then log test nvs. Stay on the dashboard with hotspot on and
let the 30-second test finish. After its ended message, allow two seconds for
the final record, then send log status and return the complete capture.

The main task writes a dedicated dummy NVS key; the writer flushes SD test
records. Require no reset, errors or drops, adequate stack margin, positive NVS
and SD counts with overlapping intervals, nvs_active=0, summary_pending=0,
key_removed=1 and nvs_errors=0. If summary_pending remains 1, report it and stop
before another test. SD record inspection is still part of the later gate;
serial success alone does not replace it. Fault hooks remain a separate build,
not an ordinary-use setting. No other test command or limit change is part of
this run. No assistant firmware edits, builds, flashes or commits for this result.

### Serial evidence

The endpoint line is omitted.

```text
2026-09-16 09:43:40.181 EVENT Console cleared.
2026-09-16 09:43:47.947 TX log status [CRLF]
2026-09-16 09:43:47.950 RX [LOG] state=ready boot=19 session=boot-19 up_ms=23979 clock=synced setup=1 hooks=0 file_bytes=571507 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922495488 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 09:43:47.952 RX [LOG] measured=1 stack_min=3988 internal_min=84896 internal_largest=31732 dma_min=77400 dma_largest=31732 writes=7 slow=0 write_max_us=2851 flush_max_us=4690 sd_max_us=97190 rotations=0 pruned=0 oversized=0
2026-09-16 09:43:47.953 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 09:43:47.953 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 09:43:47.953 RX [LOG MEM] phase=before_clock up_us=1782942 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 09:43:47.953 RX [LOG MEM] phase=after_clock up_us=1783645 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:43:47.953 RX [LOG MEM] phase=before_writer up_us=1783693 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:43:47.953 RX [LOG MEM] phase=writer_entry up_us=1783899 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:43:47.953 RX [LOG MEM] phase=after_formatter up_us=1783969 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:43:47.954 RX [LOG MEM] phase=before_mount up_us=1784553 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:43:47.954 RX [LOG MEM] phase=after_mount up_us=1881623 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:43:47.954 RX [LOG MEM] phase=before_current_open up_us=1889060 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:43:47.954 RX [LOG MEM] phase=after_current_open up_us=1890514 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:43:47.954 RX [LOG MEM] phase=storage_done up_us=1904895 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:44:32.343 RX [PROBE] window=normal run=1 ms=60001 heap_min_boot=84896 largest_min=31732 interval_ms=10 samples=6001 gap_max_us=10707 scan_max_us=862 timer=on imu_n=2945 imu_min_hz=35.58 imu_avg_hz=49.18
2026-09-16 09:44:38.666 RX Screen touched, resetting inactivity timer.
2026-09-16 09:44:38.666 RX Latest button clicked
2026-09-16 09:44:38.666 RX Initiating async latest image request...
2026-09-16 09:44:38.666 RX Preparing UI for new image request...
2026-09-16 09:44:38.666 RX Cleaning up image fetcher state...
2026-09-16 09:44:38.666 RX Screen 2 Loaded.
2026-09-16 09:44:38.738 RX === requestImage('latest') START ===
2026-09-16 09:44:38.739 RX Sending HTTP GET...
2026-09-16 09:44:38.739 RX [PROBE] window=normal run=2 ms=6396 heap_min_boot=84896 largest_min=31732 interval_ms=10 samples=639 gap_max_us=10544 scan_max_us=395 timer=on imu_n=309 imu_min_hz=37.99 imu_avg_hz=48.99
2026-09-16 09:44:39.446 RX [PROBE] window=image_https run=1 ms=706 heap_min_boot=34980 largest_min=26612 interval_ms=10 samples=71 gap_max_us=10727 scan_max_us=229 timer=on
2026-09-16 09:44:39.788 RX Response received in 708 ms, Content-Length: 33629
2026-09-16 09:44:39.788 RX Starting to receive image data...
2026-09-16 09:44:39.788 RX Image download complete (33629 bytes, 1122 ms since button press). Starting decode...
2026-09-16 09:44:39.926 RX JPEG decoded successfully into PSRAM.
2026-09-16 09:44:39.926 RX LVGL image source updated. Total 1260 ms from button press (budget 20000 ms).
2026-09-16 09:44:41.643 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 09:44:41.643 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 09:44:42.890 RX Screen touched, resetting inactivity timer.
2026-09-16 09:44:42.890 RX Live button clicked -> starting live feed
2026-09-16 09:44:42.890 RX [PROBE] window=normal run=3 ms=2962 heap_min_boot=34980 largest_min=31732 interval_ms=10 samples=296 gap_max_us=10140 scan_max_us=154 timer=on imu_n=134 imu_min_hz=10.42 imu_avg_hz=47.78
2026-09-16 09:44:42.890 RX Screen 2 Loaded.
2026-09-16 09:44:43.597 RX Video: endpoint parsed, port 9835, path /esp32/live
2026-09-16 09:44:43.597 RX [PROBE] window=live_tls run=1 ms=634 heap_min_boot=34972 largest_min=26612 interval_ms=10 samples=63 gap_max_us=10058 scan_max_us=130 timer=on
2026-09-16 09:44:43.841 RX Video: frame 432x768, panel 368x448 -> gap x=-64 y=-320, pan x=-15 y=32
2026-09-16 09:45:14.319 RX [PROBE] window=live_tls run=2 ms=619 heap_min_boot=34728 largest_min=26612 interval_ms=10 samples=62 gap_max_us=10647 scan_max_us=380 timer=on
2026-09-16 09:45:43.305 RX Video: 194 frames in 60.4s (3.2 fps) | http 304 | decode 74 | blit 62 | frame 311 ms | first_frame 1085 | max_gap 931 ms
2026-09-16 09:45:43.305 RX Video: http = ttfb 145 + xfer 159 ms | frame 18.3 KB | 115 KB/s while transferring
2026-09-16 09:45:43.306 RX Video: free PSRAM 7610340, free heap 46528
2026-09-16 09:45:43.307 RX [PROBE] window=live run=1 ms=60417 heap_min_boot=34728 largest_min=26612 interval_ms=10 samples=6042 gap_max_us=11506 scan_max_us=1257 timer=on
2026-09-16 09:45:43.307 RX Video: returning to previous screen
2026-09-16 09:45:43.308 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 09:45:43.308 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 09:45:48.790 TX log status [CRLF]
2026-09-16 09:45:48.796 RX [LOG] state=ready boot=19 session=boot-19 up_ms=144826 clock=synced setup=1 hooks=0 file_bytes=572622 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922495488 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 09:45:48.796 RX [LOG] measured=1 stack_min=3988 internal_min=34728 internal_largest=26612 dma_min=27232 dma_largest=26612 writes=9 slow=0 write_max_us=2851 flush_max_us=4690 sd_max_us=97190 rotations=0 pruned=0 oversized=0
2026-09-16 09:45:48.798 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 09:45:48.798 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 09:45:48.798 RX [LOG MEM] phase=before_clock up_us=1782942 free=172844 largest=110580 heap_min_boot=172844
2026-09-16 09:45:48.799 RX [LOG MEM] phase=after_clock up_us=1783645 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:45:48.799 RX [LOG MEM] phase=before_writer up_us=1783693 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:45:48.800 RX [LOG MEM] phase=writer_entry up_us=1783899 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:45:48.800 RX [LOG MEM] phase=after_formatter up_us=1783969 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:45:48.800 RX [LOG MEM] phase=before_mount up_us=1784553 free=167264 largest=102388 heap_min_boot=167156
2026-09-16 09:45:48.800 RX [LOG MEM] phase=after_mount up_us=1881623 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:45:48.800 RX [LOG MEM] phase=before_current_open up_us=1889060 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:45:48.800 RX [LOG MEM] phase=after_current_open up_us=1890514 free=129996 largest=65524 heap_min_boot=128864
2026-09-16 09:45:48.800 RX [LOG MEM] phase=storage_done up_us=1904895 free=129996 largest=65524 heap_min_boot=128864
```


## NVS/SD stress runtime check passes - 2026-09-16, boot 21, 09:55-09:56

JP ran log test nvs with DIAG_ENABLED=1, DIAG_WRITER_STACK_PSRAM=1 and
DIAG_TEST_HOOKS=1. Logger status was ready before and after, with valid active
8192-byte PSRAM stack placement and an internal 352-byte TCB.

| Measurement | Result |
|-------------|--------|
| Start / completion message | 09:55:28.941 / 09:55:58.944 |
| Completion | reason=complete; key_removed=1 |
| NVS writes / errors | 283 / 0 |
| SD test records | 993 |
| NVS first / last uptime | 46660 / 76650 ms |
| SD first / last uptime | 46671 / 76645 ms |
| Overlap of first-to-last intervals | 29974 ms |
| Final active / summary pending / key removed | 0 / 0 / 1 |
| Writer maximum used, before to after | 4204 to 4716 bytes |
| Writer minimum margin, before to after | 3988 to 3476 bytes |
| Historical internal minimum / largest, before and after | 84940 / 31732 bytes |
| DMA minimum / largest, before and after | 77444 / 31732 bytes |
| Queue peak / drops / logger errors | 1 of 16 / 0 / none |
| Writes, before to after | 7 to 1003 |
| File size, before to after | 579387 to 725745 bytes |
| Maximum write / flush / SD operation | 2182 / 7829 / 98061 us |
| Slow writes / suppressed / truncated | 0 / 0 / 0 |

The up-to-300-write test completed successfully with 283 commits. The scheduler
does not catch up with bursts after delays, so fewer than 300 is not a failure.
The NVS and SD intervals overlap for 29.974 seconds, showing sustained interleaved
activity. This does not imply PSRAM code executed while flash caches were disabled.
No reboot occurred during the test; boot remained 21 and uptime advanced.
The boot banner at initial console connection preceded the test.

Stress exercised an additional 512 bytes of writer stack, leaving 3476 bytes
of measured margin. This is a new deeper watermark with substantial headroom,
not unchanged stack use. Historical heap/block minima did not worsen.
The writer produced 993 stress SD records; the 996 added write operations also
include other records, so these counters are not expected to be identical.

The normal probe spanning this deliberate stress reported 60005 ms, 5991 memory
samples, maximum gap 65484 us, maximum scan 430 us, IMU 2875 samples, minimum
11.54 Hz and average 48.21 Hz. The 65.484 ms timer gap is real evidence of delayed
sampling during this flash-heavy test, not a 10 ms coverage guarantee. It does
not replace the earlier hooks-off ordinary-use IMU/performance comparison.
Serial data alone does not identify which flash operation caused the gap.

The runtime portion passes: no reset, errors, drops or unremoved dummy key,
with overlapping activity and adequate stack margin. summary_pending=0 indicates
the writer completed its final-record write and flush path. Inspection of
TEST_NVS_START, TEST_NVS_SD and TEST_NVS_END on the card remains pending, as do
terminal cleanup stack margin and the remaining storage, clock and breadcrumb gates.

Next single check: safely shut down the battery-equipped unit, copy the entire
/logs folder to the computer as a backup, and provide the copied current.log.
Disconnect USB and leave the unit stationary until automatic shutdown; the
current inactivity timeout is 60 seconds, plus a 30-second minimum USB-loss
grace period whose timers overlap. Wait for shutdown before removing the card.
Do not force power loss or remove the card while the board is running.
If it does not shut down, report that before removal.

The file will verify the stress records and the expected SESSION_END for boot 21,
and preserve existing history before deliberate rotation/pruning tests.
Do not run log test small or rotate yet. USB file retrieval is not implemented.
One card copy covers this inspection and the backup; repeated card removal is
not requested now. No firmware changes, build, flash or commit by the assistant.

### Serial evidence

The partial calibration payload line is omitted.

```text
2026-09-16 09:55:02.899 EVENT Console cleared.
2026-09-16 09:55:04.086 EVENT Connect requested. After open: DTR=true, RTS=false.
2026-09-16 09:55:06.081 EVENT Explicit signals applied. After open: DTR=true, RTS=false.
2026-09-16 09:55:06.082 EVENT Connected. Send status; compare with the previous reading if available.
2026-09-16 09:55:06.082 RX ESP-ROM:esp32s3-20210327
2026-09-16 09:55:06.082 RX Initial MQTT connection successful!
2026-09-16 09:55:06.082 RX [NET] WiFi=CONNECTED | MQTT=REMOTE CONNECTED
2026-09-16 09:55:06.082 RX --- Setup complete: CPU 240 MHz | heap 90508 | PSRAM 8336060 ---
2026-09-16 09:55:06.082 RX
2026-09-16 09:55:06.082 RX [TEST] Serial bench commands: off, on, status, log status. Send with CR or LF.
2026-09-16 09:55:21.431 TX log status [CRLF]
2026-09-16 09:55:21.433 RX [LOG] state=ready boot=21 session=boot-21 up_ms=39151 clock=synced setup=1 hooks=1 file_bytes=579387 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922495488 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 09:55:21.435 RX [LOG] measured=1 stack_min=3988 internal_min=84940 internal_largest=31732 dma_min=77444 dma_largest=31732 writes=7 slow=0 write_max_us=1877 flush_max_us=4705 sd_max_us=98061 rotations=0 pruned=0 oversized=0
2026-09-16 09:55:21.436 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 09:55:21.436 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 09:55:21.436 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 09:55:21.436 RX [LOG MEM] phase=before_clock up_us=1783897 free=172724 largest=110580 heap_min_boot=172724
2026-09-16 09:55:21.436 RX [LOG MEM] phase=after_clock up_us=1784593 free=167144 largest=102388 heap_min_boot=167036
2026-09-16 09:55:21.436 RX [LOG MEM] phase=before_writer up_us=1784649 free=167144 largest=102388 heap_min_boot=167036
2026-09-16 09:55:21.437 RX [LOG MEM] phase=writer_entry up_us=1784858 free=167144 largest=102388 heap_min_boot=167036
2026-09-16 09:55:21.437 RX [LOG MEM] phase=after_formatter up_us=1784943 free=167144 largest=102388 heap_min_boot=167036
2026-09-16 09:55:21.437 RX [LOG MEM] phase=before_mount up_us=1785499 free=167144 largest=102388 heap_min_boot=167036
2026-09-16 09:55:21.437 RX [LOG MEM] phase=after_mount up_us=1883438 free=129880 largest=65524 heap_min_boot=128668
2026-09-16 09:55:21.437 RX [LOG MEM] phase=before_current_open up_us=1890722 free=129880 largest=65524 heap_min_boot=128668
2026-09-16 09:55:21.437 RX [LOG MEM] phase=after_current_open up_us=1892010 free=129880 largest=65524 heap_min_boot=128668
2026-09-16 09:55:21.437 RX [LOG MEM] phase=storage_done up_us=1905445 free=129880 largest=65524 heap_min_boot=128668
2026-09-16 09:55:28.934 TX log test nvs [CRLF]
2026-09-16 09:55:28.941 RX [LOG TEST] NVS stress started: 30 s, up to 300 dummy-key commits; writer flushes test records
2026-09-16 09:55:58.944 RX [PROBE] window=normal run=1 ms=60005 heap_min_boot=84940 largest_min=31732 interval_ms=10 samples=5991 gap_max_us=65484 scan_max_us=430 timer=on imu_n=2875 imu_min_hz=11.54 imu_avg_hz=48.21
2026-09-16 09:55:58.944 RX [LOG TEST] NVS stress ended reason=complete key_removed=1; use log status for counts
2026-09-16 09:56:03.145 TX log status [CRLF]
2026-09-16 09:56:03.146 RX [LOG] state=ready boot=21 session=boot-21 up_ms=80864 clock=synced setup=1 hooks=1 file_bytes=725745 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922331648 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 09:56:03.146 RX [LOG] measured=1 stack_min=3476 internal_min=84940 internal_largest=31732 dma_min=77444 dma_largest=31732 writes=1003 slow=0 write_max_us=2182 flush_max_us=7829 sd_max_us=98061 rotations=0 pruned=0 oversized=0
2026-09-16 09:56:03.147 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4716 stack_final_margin=-1
2026-09-16 09:56:03.148 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=283 nvs_errors=0 sd_records=993 key_removed=1 first_nvs_ms=46660 last_nvs_ms=76650 first_sd_ms=46671 last_sd_ms=76645
2026-09-16 09:56:03.148 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 09:56:03.148 RX [LOG MEM] phase=before_clock up_us=1783897 free=172724 largest=110580 heap_min_boot=172724
2026-09-16 09:56:03.149 RX [LOG MEM] phase=after_clock up_us=1784593 free=167144 largest=102388 heap_min_boot=167036
2026-09-16 09:56:03.149 RX [LOG MEM] phase=before_writer up_us=1784649 free=167144 largest=102388 heap_min_boot=167036
2026-09-16 09:56:03.149 RX [LOG MEM] phase=writer_entry up_us=1784858 free=167144 largest=102388 heap_min_boot=167036
2026-09-16 09:56:03.149 RX [LOG MEM] phase=after_formatter up_us=1784943 free=167144 largest=102388 heap_min_boot=167036
2026-09-16 09:56:03.149 RX [LOG MEM] phase=before_mount up_us=1785499 free=167144 largest=102388 heap_min_boot=167036
2026-09-16 09:56:03.149 RX [LOG MEM] phase=after_mount up_us=1883438 free=129880 largest=65524 heap_min_boot=128668
2026-09-16 09:56:03.150 RX [LOG MEM] phase=before_current_open up_us=1890722 free=129880 largest=65524 heap_min_boot=128668
2026-09-16 09:56:03.150 RX [LOG MEM] phase=after_current_open up_us=1892010 free=129880 largest=65524 heap_min_boot=128668
2026-09-16 09:56:03.150 RX [LOG MEM] phase=storage_done up_us=1905445 free=129880 largest=65524 heap_min_boot=128668
```


## Card verification and shutdown pass; startup watchdogs noted - 2026-09-16, 10:00 capture

JP supplied F:/logs/current.log after the requested battery shutdown.
The card's /logs directory contained only current.log (728137 bytes).
A byte-identical backup is preserved in
[bench_data/sd_current_2026-09-16_1000.txt](bench_data/sd_current_2026-09-16_1000.txt).
SHA-256: 116d71c4a468f2907f382ccd57a28b6b7f2085c7cbfaed46271cdda2e88adec8.
The copy covers every file observed in /logs. The card was read only.

The file contains 2129 complete newline-terminated records. Boot 21 contains
1008 records with consecutive sequence numbers 1 through 1008.
There are no ERROR-level records in that boot.

Verified on card:
- TEST_NVS_START at 09:55:28.930, up_ms=46659, seq=9.
- Exactly 993 TEST_NVS_SD records, with sd_records consecutive from 1 to 993.
- TEST_NVS_END at 09:55:58.941, seq=1003: nvs_writes=283, nvs_errors=0,
  sd_records=993, key_removed=1. Its first/last NVS and SD timestamps match
  the serial summary exactly (46660/76650 and 46671/76645).
- Subsequent HEALTH records retain stack_min=3476, no drops and no slow writes.
- Final SESSION_END at 10:00:26.552, seq=1008, boot=21, up_ms=344280:
  reason=shutdown pending=0. The file ends cleanly after this record.

This verifies the stress records reached the card and the tested normal shutdown
persisted its final record. It does not test power-loss durability, deep sleep,
or capture the writer's final post-cleanup stack margin.
TEST_NVS_SD can precede the queued TEST_NVS_START in serialization order because
the writer writes its stress record directly while START is queued; use captured
uptime as well as sequence. The last SD record contains nvs_writes=282 and the
final summary 283, consistent with the last NVS commit occurring after that SD
record. Neither ordering difference implies missing records.

Separate finding, before either deliberate stress run:
- Boot 19 reports reset=task_watchdog, reset_code=6, around 09:43:24.
  Boot 18's last record is SETUP_COMPLETE at up_ms=8571.
- Boot 21 reports reset=task_watchdog, reset_code=6, around 09:54:42.
  Boot 20's last stored record is CLOCK_OFFSET at up_ms=7012.
  The retained main and writer breadcrumbs for boot 20 say idle at 8649 and
  8671 ms; these breadcrumbs do not identify the watchdog's triggering task.
- The new BOOT local timestamps are quality=approx. Treat wall-clock times as
  approximate; the reset reason and boot ordering are the stronger evidence.
- The boot-21 stress began at uptime 46659 and ran to completion without a reset.
  Boot 19's measured logging-ON pair also took place after its startup reset.

These startup resets were not visible in the supplied timed test captures.
The within-run performance and stress results stand, but overall startup stability
is unresolved. Older watchdog markers were already noted in the overnight review;
do not assign these to the PSRAM stack, SD writer, USB monitor, or firmware upload
without a captured panic/backtrace or controlled reproduction.

JP was asked whether he noticed a restart or freeze after flashing and before
opening the console around those times. Clarify this before deliberate reset or
pruning tests, which would obscure the unplanned restart history. The backup now
preserves that history. No firmware change, build, flash, commit or push was made.


### JP's startup-reset clarification

JP confirmed noticing a restart or freeze. He believes it happens when he closes
the VS Code serial monitor after flashing and says he then needs to reset the
board to connect to the web page. The manual-reset method and exact ordering
relative to the recorded watchdog are not yet established.

This matches the unresolved Step 0 report, before the SD logger was introduced:
closing the VS Code monitor froze the UI, while the browser with DTR=true and
RTS=false later passed repeated disconnects and a Live-cycle disconnect.
The serial-monitor transition is now the leading trigger to isolate, not proof
that the SD writer or PSRAM experiment caused the recorded watchdogs.

A read-only source check found that pinned core 3.1.3 HWCDC uses a default
100 ms TX timeout and has host/buffer wait paths. Firmware calls USBSerial.begin
without a custom TX timeout. These facts alone do not explain a watchdog reset
or establish the DTR/RTS levels VS Code applies. No USB settings were changed.

Next controlled check, without flashing: reinsert the backed-up card while the
board is off, power it, establish a working browser connection and capture status.
Disconnect the browser, open the VS Code serial monitor and send status, then
close only that monitor while USB stays plugged in. Watch for 30 seconds without
a manual reset; note freeze versus automatic restart. Reopen the browser with
DTR=true/RTS=false and send status if possible. If connection fails, report that
before resetting. Never open both serial clients at once.
This isolates the monitor transition from firmware upload and manual reset.
Rotation tests can follow once the startup-reset handling is understood.


## VS Code monitor-close freeze reproduced - 2026-09-16, boot 22

JP performed the controlled test without reflashing. Before closing the VS Code
monitor, status replied at uptime 387879 ms: boot 22, ready, Wi-Fi/MQTT connected,
hooks=1, active valid PSRAM writer, errors/drops zero. Internal_min=84728,
historical largest block=31732; stack used/margin=4204/3988.
The preceding complete normal probe measured 60004 ms, 6000 memory samples,
IMU average 49.11 Hz and minimum 27.04 Hz. NVS stress was inactive and had not
run in this boot.

After VS Code printed "Closed the serial port COM4", JP saw the spinner stop.
It remained visibly frozen for more than 30 seconds, with no observed automatic
restart. The web page then opened the same port successfully:
- 10:17:32.725 Connect requested, DTR=true and RTS=false.
- 10:17:35.453 explicit signals applied; Connected.
The UI remained frozen and no RX output appeared.

JP then sent both commands from the connected browser and reported no responses:
- 10:19:58.175 TX log status [CRLF]
- 10:20:15.064 TX status [CRLF]

This reproduces a persistent visible UI freeze plus a failed command/reply path
across the VS Code-to-browser transition. Browser open/write success establishes
host-side port access, not execution or receipt by the firmware. It does not
prove that both CPU cores or the SD writer stopped. There is no post-freeze
uptime or new boot marker to establish a reset. JP's known recovery is unplugging
and reconnecting USB, after which the board restarts.

Do not equate this persistent freeze with the earlier recorded task-watchdog
resets. The installed build sdkconfig enables the 5-second task watchdog with
panic and CPU0 idle checking; CPU1 idle checking is off. LoopCore=1 in the actual
sketch.yaml profile. A main-loop stall need not be caught by that particular
watchdog configuration. No conclusion about the blocked task follows without
additional evidence.

Read-only source/configuration checks:
- Core 3.1.3 HWCDC has a 100 ms default transmit timeout, timed lock acquisition,
  host/buffer wait logic and a USB Serial/JTAG interrupt handler. These paths
  are candidates for inspection, not a demonstrated cause of this freeze.
- Firmware calls USBSerial.begin(115200); no custom TX timeout was found.
- Installed extensions include Microsoft Serial Monitor 0.13.1 and Arduino Maker
  Workshop 1.1.9. Their package manifests expose no DTR/RTS matches in the checked
  configuration text; the precise monitor used and close-time signal sequence
  remain unverified. Workspace monitor settings contain no explicit DTR/RTS.
- The same VS Code-close symptom was reported during Step 0 before SD logging.
  Browser-only explicit DTR=true/RTS=false disconnect cycles previously passed.

No further repetition of the same freeze test is needed to establish this
reproducer. Recover via JP's usual USB unplug/replug procedure, then use the
tested browser-only serial workflow; keep VS Code for compilation and flashing.
The USB issue stays open for host/core investigation, without changing logger
memory settings or assuming the successful SD stress test failed.
No firmware, USB settings, watchdog settings, extension settings or commits changed.


## USB recovery confirmed; normal-limit rotation is next - 2026-09-16, 10:24

JP unplugged and reconnected the USB cable, opened the browser with explicit
DTR=true/RTS=false and confirmed the UI was running normally. status replied at
10:24:11.646 with uptime 74268 ms and boot=23, following the frozen boot 22.
Wi-Fi and MQTT were connected; the logger was ready, synced and hooks=1.

Recovery snapshot:
- file_bytes=734350, generation=1, newest=0, archives=0, rotations=0, pruned=0.
- queue=0/16, high=1, drops=0, suppressed=0, truncated=0, error=none, errno=0.
- internal_min=84752, historical largest=31732; DMA minimum/largest=77256/31732.
- Writer active, placement valid, PSRAM stack 8192, internal TCB 352.
  Used/margin=4204/3988 bytes. Final margin=-1 while the writer is active.
- writes=8, slow=0, write_max_us=2999, flush_max_us=4951, sd_max_us=105025.
  The broad SD maximum is not itself a slow-write count.
- NVS stress counters are zero and inactive after reboot. key_removed=0 here
  means this boot has not run the stress test; it does not reverse boot 21's
  verified cleanup.

Boot increment and low uptime confirm a restart between the readings.
The startup banner was delivered when the browser opened; that alone does not
locate the reset at browser-open time. Recovery does not fix the reproducible
VS Code monitor-close issue or identify its underlying cause.
Use VS Code for builds/flashing and the tested browser for serial monitoring.

The verified card backup is retained. Continue storage testing through the
working browser, with one forced rotation at normal limits. This ordinary
rotation test does not deliberately reset the board or enable small retention.
The USB issue remains open independently.

Next single test, same firmware and hooks=1:
1. Send log test rotate once.
2. Wait about three seconds, then send log status.
3. Leave the unit on the normal screen with USB/hotspot on for about 65 seconds.
4. Send log status again and return the full capture.
Expected from generation 1 with no existing archives: generation=2, newest=1,
archives=1, rotations=1, pruned=0, state=ready, error=none, drops=0.
The old current should become /logs/archive-00000001.log and a new current.log
should receive headers and subsequent health records. file_bytes should initially
be small, then increase with the next minute record. No deliberate deletion is
expected with normal limits and the reported free space. File/header inspection
remains a separate later check; status alone does not prove file contents.

Do not send log test small yet. No rebuild, reflash or card removal is needed
for this rotation test. No firmware edits or commits were made for this result.


## Normal-limit forced rotation runtime passes - 2026-09-16, boot 24, 14:43-14:45

JP powered the unit off during the break, then returned and ran one log test rotate
through the browser. Before rotation the logger was ready, hooks=1, generation=1,
no archives, file_bytes=737293 and rotations=pruned=0.

| Measurement | Before | After rotation | After next minute |
|-------------|--------|----------------|-------------------|
| generation / newest / archives | 1 / 0 / 0 | 2 / 1 / 1 | 2 / 1 / 1 |
| rotations / pruned | 0 / 0 | 1 / 0 | 1 / 0 |
| current file bytes | 737293 | 449 | 1008 |
| writes | 8 | 11 | 12 |
| writer maximum used / minimum margin | 4204 / 3988 | 4780 / 3412 | 4780 / 3412 |
| historical internal minimum / largest block | 84748 / 31732 | unchanged | unchanged |
| logger errors / drops / slow writes | none / 0 / 0 | none / 0 / 0 | none / 0 / 0 |

The command was queued at 14:43:46.660. Status at 14:43:58.865 showed one
completed rotation; status at 14:45:02.827 confirmed continued append growth
of 559 bytes. The counters are consistent with archive-00000001.log and a fresh
current.log with generation 2. This is a runtime/counter pass; file contents and
header correctness have not been directly inspected after this rotation.

The writer used 576 more stack bytes than before rotation, leaving 3412 bytes.
Placement remained valid and active (8192-byte PSRAM stack, internal 352-byte
TCB). Queue peak remained 1 of 16, with suppressed/truncated records zero.
DMA minimum/largest stayed 77252/31732. Write/flush maxima were 2893/4903 us;
the broad SD-operation maximum was 171513 us, while slow writes remained zero.
Normal IMU averages were 49.23 and 49.20 Hz; minima 35.67 and 31.13 Hz.
These afternoon figures are not a new pair with the morning performance baseline.
Boot stayed 24; there is no reset in this capture.

Next single test: temporary small-limit rotation and pruning on the backed-up
test card. DIAG_TEST_HOOKS stays 1. log test small selects 8192-byte current files,
three archives and 16384-byte free-space reserve. The content budget becomes
32768 bytes, so the existing roughly 737 KB archive can be deleted before the
archive count reaches three. Do not describe pruning as count-only.

JP must know this test deliberately deletes managed archives from the card.
The 10:00 copy is preserved in docs/bench_data/sd_current_2026-09-16_1000.txt;
later card history is not all included in that backup. If JP wants the intervening
card-only history retained, back it up before proceeding.
The test does not request deleting unrelated files or changing normal defaults.

Procedure: log test small, wait three seconds, log status. Then send
log test rotate four times separately, waiting about three seconds after each.
Send log status. Finally send log test normal, wait three seconds, log status.
Stop further commands if an error/disabled state occurs, and send the capture.
Expected: at most three archives under small limits, pruned greater than zero,
advancing generations and rotations, ready, no drops/errors and adequate stack
margin. Do not hard-code final generation: size-triggered rotation can also occur
if the current file grew while waiting. Restoring normal limits does not restore
deleted archives. File/name verification will still require later card inspection.

This test checks pruning under reduced limits and forced rotation. It does not
by itself prove natural size-triggered rotation at exactly 8192 or 2097152 bytes,
or preservation of unrelated filenames on disk. Those remain distinct checks.
No firmware edit, build, flash or commit was made for this result.

### Serial evidence

```text
2026-09-16 14:43:05.555 EVENT Console cleared.
2026-09-16 14:43:17.691 TX log status [CRLF]
2026-09-16 14:43:17.698 RX [LOG] state=ready boot=24 session=boot-24 up_ms=68379 clock=synced setup=1 hooks=1 file_bytes=737293 generation=1 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922331648 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 14:43:17.700 RX [LOG] measured=1 stack_min=3988 internal_min=84748 internal_largest=31732 dma_min=77252 dma_largest=31732 writes=8 slow=0 write_max_us=2893 flush_max_us=4903 sd_max_us=171513 rotations=0 pruned=0 oversized=0
2026-09-16 14:43:17.701 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4204 stack_final_margin=-1
2026-09-16 14:43:17.701 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 14:43:17.702 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 14:43:17.702 RX [LOG MEM] phase=before_clock up_us=1881890 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 14:43:17.702 RX [LOG MEM] phase=after_clock up_us=1882577 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:43:17.702 RX [LOG MEM] phase=before_writer up_us=1882633 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:43:17.702 RX [LOG MEM] phase=writer_entry up_us=1882844 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:43:17.702 RX [LOG MEM] phase=after_formatter up_us=1882927 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:43:17.703 RX [LOG MEM] phase=before_mount up_us=1883487 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:43:17.703 RX [LOG MEM] phase=after_mount up_us=2054886 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:43:17.703 RX [LOG MEM] phase=before_current_open up_us=2062178 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:43:17.703 RX [LOG MEM] phase=after_current_open up_us=2063670 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:43:17.703 RX [LOG MEM] phase=storage_done up_us=2077887 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:43:27.555 RX [PROBE] window=normal run=1 ms=60005 heap_min_boot=84748 largest_min=31732 interval_ms=10 samples=6001 gap_max_us=10790 scan_max_us=438 timer=on imu_n=2949 imu_min_hz=35.67 imu_avg_hz=49.23
2026-09-16 14:43:27.555 RX TX motion MQTT: Moving (periodic)
2026-09-16 14:43:27.640 RX Movement Stopped.
2026-09-16 14:43:46.657 TX log test rotate [CRLF]
2026-09-16 14:43:46.660 RX [LOG TEST] hook queued
2026-09-16 14:43:58.863 TX log status [CRLF]
2026-09-16 14:43:58.865 RX [LOG] state=ready boot=24 session=boot-24 up_ms=109547 clock=synced setup=1 hooks=1 file_bytes=449 generation=2 newest=1 archives=1 card_bytes=15931539456 free_bytes=15922298880 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 14:43:58.865 RX [LOG] measured=1 stack_min=3412 internal_min=84748 internal_largest=31732 dma_min=77252 dma_largest=31732 writes=11 slow=0 write_max_us=2893 flush_max_us=4903 sd_max_us=171513 rotations=1 pruned=0 oversized=0
2026-09-16 14:43:58.867 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4780 stack_final_margin=-1
2026-09-16 14:43:58.867 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 14:43:58.868 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 14:43:58.868 RX [LOG MEM] phase=before_clock up_us=1881890 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 14:43:58.869 RX [LOG MEM] phase=after_clock up_us=1882577 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:43:58.869 RX [LOG MEM] phase=before_writer up_us=1882633 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:43:58.869 RX [LOG MEM] phase=writer_entry up_us=1882844 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:43:58.869 RX [LOG MEM] phase=after_formatter up_us=1882927 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:43:58.871 RX [LOG MEM] phase=before_mount up_us=1883487 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:43:58.871 RX [LOG MEM] phase=after_mount up_us=2054886 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:43:58.871 RX [LOG MEM] phase=before_current_open up_us=2062178 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:43:58.871 RX [LOG MEM] phase=after_current_open up_us=2063670 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:43:58.871 RX [LOG MEM] phase=storage_done up_us=2077887 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:44:20.026 RX [PROBE] window=normal run=2 ms=60005 heap_min_boot=84748 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10854 scan_max_us=456 timer=on imu_n=2946 imu_min_hz=31.13 imu_avg_hz=49.20
2026-09-16 14:45:02.822 TX log status [CRLF]
2026-09-16 14:45:02.827 RX [LOG] state=ready boot=24 session=boot-24 up_ms=173511 clock=synced setup=1 hooks=1 file_bytes=1008 generation=2 newest=1 archives=1 card_bytes=15931539456 free_bytes=15922298880 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 14:45:02.829 RX [LOG] measured=1 stack_min=3412 internal_min=84748 internal_largest=31732 dma_min=77252 dma_largest=31732 writes=12 slow=0 write_max_us=2893 flush_max_us=4903 sd_max_us=171513 rotations=1 pruned=0 oversized=0
2026-09-16 14:45:02.830 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4780 stack_final_margin=-1
2026-09-16 14:45:02.830 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 14:45:02.831 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 14:45:02.831 RX [LOG MEM] phase=before_clock up_us=1881890 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 14:45:02.831 RX [LOG MEM] phase=after_clock up_us=1882577 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:45:02.831 RX [LOG MEM] phase=before_writer up_us=1882633 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:45:02.831 RX [LOG MEM] phase=writer_entry up_us=1882844 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:45:02.831 RX [LOG MEM] phase=after_formatter up_us=1882927 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:45:02.833 RX [LOG MEM] phase=before_mount up_us=1883487 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:45:02.833 RX [LOG MEM] phase=after_mount up_us=2054886 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:45:02.833 RX [LOG MEM] phase=before_current_open up_us=2062178 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:45:02.833 RX [LOG MEM] phase=after_current_open up_us=2063670 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:45:02.833 RX [LOG MEM] phase=storage_done up_us=2077887 free=129836 largest=65524 heap_min_boot=128704
```


## Small-limit rotation/pruning runtime passes - 2026-09-16, boot 24, 14:48-14:49

JP selected small limits, issued four separate forced rotations and restored
normal limits through the browser. All hooks were accepted. The logger stayed
ready in the same boot with no errors, drops, suppressed or truncated records.

| Measurement | After small command | After four rotations | After normal command |
|-------------|---------------------|----------------------|----------------------|
| generation / newest / archives | 2 / 1 / 1 | 6 / 5 / 3 | 6 / 5 / 3 |
| rotations / pruned | 1 / 0 | 5 / 2 | 5 / 2 |
| current file bytes | 2820 | 451 | 584 |
| writes | 16 | 29 | 30 |
| free card bytes | 15922298880 | 15922954240 | 15922954240 |
| writer maximum used / minimum margin | 4780 / 3412 | unchanged | unchanged |
| historical internal minimum / largest block | 84748 / 31732 | unchanged | unchanged |

The four commands advanced generation by four and kept exactly three archives,
with two managed deletions reported. This agrees with the source's content-budget
and count pruning: the large original archive exceeds the 32768-byte small
content budget, and a further oldest archive must be removed as new ones accrue.
Expected survivors are archives 3, 4 and 5; their exact names/content have not
yet been inspected on card. Do not claim unrelated-file preservation from these
counters alone. Space availability increased by 655360 bytes.

log test normal was accepted at 14:49:02.219; following status remained ready,
with a further test-hook write and file growth. Source handling restores the
normal file/count/reserve limits and clears simulated space/write faults.
Status does not print the live limits, so that restoration is supported by
accepted-command/source behavior rather than an explicit numeric limits readback.
No additional rotation or deletion was observed after that command.

Writer PSRAM placement remained valid and active; TCB stayed internal.
DMA minimum/largest remained 77252/31732. Queue peak was 1 of 16, slow writes
zero. Final write/flush/SD maxima were 3822/4951/171513 us.
Normal IMU averages were 49.21 and 49.25 Hz, minima 32.13 and 32.26 Hz.
This afternoon storage test is not a new comparison with the morning baseline.

The runtime pruning gate passes. Natural size-triggered rotation, on-card
header/content checks and preservation of unrelated names still need evidence.
The 10:00 backup remains preserved outside the card.

Next single fault test: simulated full-card write failure, without filling the
actual card. Keep the installed build (enabled=1, PSRAM=1, hooks=1), normal limits,
hotspot on and browser serial. Send log status, log test full once, wait three
seconds, log status. Expect state=disabled, error=test_full_write, errno=ENOSPC
(28 in this build), writer_lifecycle=parked, and a nonnegative final stack margin.
These are expected injected-failure results, not an unexpected test failure.

Then request Latest, confirm it displays, return to dashboard and send log status.
The companion must continue working with a safely stopped logger. No reset,
crash or memory-allocation failure is expected. Capture image_https and all LOG
lines. No manual card removal or reboot before returning this result.
Do not send log test normal to recover: a terminal error disables writes for the
rest of that boot. We will reboot afterward to check logging resumes.
No firmware edits, build, flash or commit for this result.

### Serial evidence

```text
2026-09-16 14:47:47.553 EVENT Console cleared.
2026-09-16 14:48:00.770 TX log test small [CRLF]
2026-09-16 14:48:00.772 RX [PROBE] window=normal run=5 ms=60003 heap_min_boot=84748 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10719 scan_max_us=862 timer=on imu_n=2947 imu_min_hz=32.13 imu_avg_hz=49.21
2026-09-16 14:48:00.772 RX [LOG TEST] hook queued
2026-09-16 14:48:09.028 TX log status [CRLF]
2026-09-16 14:48:09.029 RX [LOG] state=ready boot=24 session=boot-24 up_ms=359718 clock=synced setup=1 hooks=1 file_bytes=2820 generation=2 newest=1 archives=1 card_bytes=15931539456 free_bytes=15922298880 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 14:48:09.030 RX [LOG] measured=1 stack_min=3412 internal_min=84748 internal_largest=31732 dma_min=77252 dma_largest=31732 writes=16 slow=0 write_max_us=2893 flush_max_us=4951 sd_max_us=171513 rotations=1 pruned=0 oversized=0
2026-09-16 14:48:09.031 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4780 stack_final_margin=-1
2026-09-16 14:48:09.032 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 14:48:09.033 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 14:48:09.033 RX [LOG MEM] phase=before_clock up_us=1881890 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 14:48:09.033 RX [LOG MEM] phase=after_clock up_us=1882577 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:48:09.033 RX [LOG MEM] phase=before_writer up_us=1882633 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:48:09.033 RX [LOG MEM] phase=writer_entry up_us=1882844 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:48:09.033 RX [LOG MEM] phase=after_formatter up_us=1882927 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:48:09.033 RX [LOG MEM] phase=before_mount up_us=1883487 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:48:09.033 RX [LOG MEM] phase=after_mount up_us=2054886 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:48:09.033 RX [LOG MEM] phase=before_current_open up_us=2062178 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:48:09.033 RX [LOG MEM] phase=after_current_open up_us=2063670 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:48:09.033 RX [LOG MEM] phase=storage_done up_us=2077887 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:48:20.039 RX [PROBE] window=normal run=6 ms=60004 heap_min_boot=84748 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10762 scan_max_us=430 timer=on imu_n=2949 imu_min_hz=32.26 imu_avg_hz=49.25
2026-09-16 14:48:27.080 TX log test rotate [CRLF]
2026-09-16 14:48:27.086 RX [LOG TEST] hook queued
2026-09-16 14:48:32.512 TX log test rotate [CRLF]
2026-09-16 14:48:32.515 RX [LOG TEST] hook queued
2026-09-16 14:48:37.624 TX log test rotate [CRLF]
2026-09-16 14:48:37.628 RX [LOG TEST] hook queued
2026-09-16 14:48:43.059 TX log test rotate [CRLF]
2026-09-16 14:48:43.064 RX [LOG TEST] hook queued
2026-09-16 14:48:52.020 TX log status [CRLF]
2026-09-16 14:48:52.026 RX [LOG] state=ready boot=24 session=boot-24 up_ms=402716 clock=synced setup=1 hooks=1 file_bytes=451 generation=6 newest=5 archives=3 card_bytes=15931539456 free_bytes=15922954240 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 14:48:52.026 RX [LOG] measured=1 stack_min=3412 internal_min=84748 internal_largest=31732 dma_min=77252 dma_largest=31732 writes=29 slow=0 write_max_us=3766 flush_max_us=4951 sd_max_us=171513 rotations=5 pruned=2 oversized=0
2026-09-16 14:48:52.028 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4780 stack_final_margin=-1
2026-09-16 14:48:52.029 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 14:48:52.029 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 14:48:52.030 RX [LOG MEM] phase=before_clock up_us=1881890 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 14:48:52.030 RX [LOG MEM] phase=after_clock up_us=1882577 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:48:52.030 RX [LOG MEM] phase=before_writer up_us=1882633 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:48:52.031 RX [LOG MEM] phase=writer_entry up_us=1882844 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:48:52.031 RX [LOG MEM] phase=after_formatter up_us=1882927 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:48:52.031 RX [LOG MEM] phase=before_mount up_us=1883487 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:48:52.032 RX [LOG MEM] phase=after_mount up_us=2054886 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:48:52.032 RX [LOG MEM] phase=before_current_open up_us=2062178 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:48:52.032 RX [LOG MEM] phase=after_current_open up_us=2063670 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:48:52.033 RX [LOG MEM] phase=storage_done up_us=2077887 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:49:02.214 TX log test normal [CRLF]
2026-09-16 14:49:02.219 RX [LOG TEST] hook queued
2026-09-16 14:49:09.740 TX log status [CRLF]
2026-09-16 14:49:09.743 RX [LOG] state=ready boot=24 session=boot-24 up_ms=420434 clock=synced setup=1 hooks=1 file_bytes=584 generation=6 newest=5 archives=3 card_bytes=15931539456 free_bytes=15922954240 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 14:49:09.745 RX [LOG] measured=1 stack_min=3412 internal_min=84748 internal_largest=31732 dma_min=77252 dma_largest=31732 writes=30 slow=0 write_max_us=3822 flush_max_us=4951 sd_max_us=171513 rotations=5 pruned=2 oversized=0
2026-09-16 14:49:09.746 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4780 stack_final_margin=-1
2026-09-16 14:49:09.746 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 14:49:09.747 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 14:49:09.747 RX [LOG MEM] phase=before_clock up_us=1881890 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 14:49:09.747 RX [LOG MEM] phase=after_clock up_us=1882577 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:49:09.747 RX [LOG MEM] phase=before_writer up_us=1882633 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:49:09.747 RX [LOG MEM] phase=writer_entry up_us=1882844 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:49:09.747 RX [LOG MEM] phase=after_formatter up_us=1882927 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:49:09.747 RX [LOG MEM] phase=before_mount up_us=1883487 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:49:09.747 RX [LOG MEM] phase=after_mount up_us=2054886 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:49:09.748 RX [LOG MEM] phase=before_current_open up_us=2062178 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:49:09.748 RX [LOG MEM] phase=after_current_open up_us=2063670 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:49:09.748 RX [LOG MEM] phase=storage_done up_us=2077887 free=129836 largest=65524 heap_min_boot=128704
```


## Simulated full-card failure and terminal cleanup pass - 2026-09-16, boot 24, 14:53

JP sent log test full at 14:53:03.956. The hook was queued and the logger reported
disabled reason=test_full_write errno=28 at 14:53:03.977.
This is the deliberately injected ENOSPC write failure, not a physically full card.

| Measurement | Before fault | After fault and Latest |
|-------------|--------------|------------------------|
| State / error | ready / none | disabled / test_full_write, errno 28 |
| Writer lifecycle | active | parked |
| Writer used / minimum margin | 4780 / 3412 bytes | unchanged |
| Final post-cleanup stack margin | -1 | 3412 bytes |
| File bytes / writes | 2823 / 34 | 2956 / 35 |
| Generation / newest / archives | 6 / 5 / 3 | unchanged |
| Rotations / pruned | 5 / 2 | unchanged |
| Queue peak / drops / slow writes | 1 of 16 / 0 / 0 | unchanged |

The successful extra write is consistent with TEST_HOOK recorded before fault
injection. Subsequent status after Latest still showed 2956 bytes and 35 writes,
confirming no further logger writes reported while parked.
Placement stayed valid: PSRAM stack 8192 bytes, internal TCB 352 bytes.
Final margin 3412 is now measured after filesystem/formatter cleanup. The static
PSRAM stack and TCB intentionally remain allocated until reboot; parked is the
intended lifecycle, not a leaked running task or an attempt to free its own stack.

The companion continued processing touch and Latest:
- image_https run 1: 761 ms, 76 samples at 10 ms,
  largest_min=26612, heap_min_boot=36252.
- Maximum sampler gap/scan: 10636/502 us.
- Image 34262 bytes, decoded and displayed in 1274 ms.
- Returned to screen 1 without saving a preference.
- No reset in this capture; boot remained 24 and uptime advanced.

The sampled HTTPS block minimum exceeds 20480 by 6132 bytes.
The disabled logger's retained memory fields stay internal_min=84748 and
internal_largest=31732. They no longer update because the writer is parked;
the active Stage 0 probe is the source for post-fault image memory measurements.
The normal probe before Latest averaged 49.20 Hz IMU, minimum 28.52 Hz.
This is a fault-handling check, not a paired performance run.

The injected write-failure path, terminal stack capture and continued Latest
operation pass. Real media corruption/removal, power-loss durability and other
fault paths are not established by this simulation.

Next single check: reboot recovery without reflashing or removing the card.
Power down normally (disconnect USB on the battery board, leave stationary until
automatic shutdown), then power on/reconnect USB. Use the browser with explicit
DTR=true/RTS=false; wait for green MQTT, send log status, wait 65 seconds on the
normal screen and send log status again.

Expect a new boot, ready state, error=none/errno=0, active valid PSRAM writer,
generation=6, newest=5 and archives=3 retained, current file appended with fresh
boot records and then a minute health record. Runtime rotations/pruned counters
reset with the boot; that is not deletion of archives. final stack margin returns
to -1 for the new active writer. The simulated fault flag clears on reboot.
No additional SESSION_END is expected from boot 24 after the writer was disabled.
If disabled persists, return the error instead of sending more fault commands.
No firmware changes, build, flash or commit for this result.

### Serial evidence

The endpoint line is omitted.

```text
2026-09-16 14:52:48.682 EVENT Console cleared.
2026-09-16 14:52:52.562 TX log status [CRLF]
2026-09-16 14:52:52.564 RX [LOG] state=ready boot=24 session=boot-24 up_ms=643260 clock=synced setup=1 hooks=1 file_bytes=2823 generation=6 newest=5 archives=3 card_bytes=15931539456 free_bytes=15922954240 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 14:52:52.565 RX [LOG] measured=1 stack_min=3412 internal_min=84748 internal_largest=31732 dma_min=77252 dma_largest=31732 writes=34 slow=0 write_max_us=3822 flush_max_us=4951 sd_max_us=171513 rotations=5 pruned=2 oversized=0
2026-09-16 14:52:52.565 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4780 stack_final_margin=-1
2026-09-16 14:52:52.565 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 14:52:52.566 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 14:52:52.567 RX [LOG MEM] phase=before_clock up_us=1881890 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 14:52:52.567 RX [LOG MEM] phase=after_clock up_us=1882577 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:52:52.567 RX [LOG MEM] phase=before_writer up_us=1882633 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:52:52.567 RX [LOG MEM] phase=writer_entry up_us=1882844 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:52:52.567 RX [LOG MEM] phase=after_formatter up_us=1882927 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:52:52.567 RX [LOG MEM] phase=before_mount up_us=1883487 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:52:52.567 RX [LOG MEM] phase=after_mount up_us=2054886 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:52:52.567 RX [LOG MEM] phase=before_current_open up_us=2062178 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:52:52.567 RX [LOG MEM] phase=after_current_open up_us=2063670 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:52:52.567 RX [LOG MEM] phase=storage_done up_us=2077887 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:53:03.956 TX log test full [CRLF]
2026-09-16 14:53:03.963 RX [LOG TEST] hook queued
2026-09-16 14:53:03.977 RX [LOG] disabled reason=test_full_write errno=28; companion continues
2026-09-16 14:53:11.445 TX log status [CRLF]
2026-09-16 14:53:11.447 RX [LOG] state=disabled boot=24 session=boot-24 up_ms=662144 clock=synced setup=1 hooks=1 file_bytes=2956 generation=6 newest=5 archives=3 card_bytes=15931539456 free_bytes=15922954240 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=test_full_write errno=28
2026-09-16 14:53:11.448 RX [LOG] measured=1 stack_min=3412 internal_min=84748 internal_largest=31732 dma_min=77252 dma_largest=31732 writes=35 slow=0 write_max_us=3822 flush_max_us=4951 sd_max_us=171513 rotations=5 pruned=2 oversized=0
2026-09-16 14:53:11.448 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=parked stack_used_max=4780 stack_final_margin=3412
2026-09-16 14:53:11.448 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 14:53:11.450 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 14:53:11.450 RX [LOG MEM] phase=before_clock up_us=1881890 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 14:53:11.450 RX [LOG MEM] phase=after_clock up_us=1882577 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:53:11.450 RX [LOG MEM] phase=before_writer up_us=1882633 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:53:11.450 RX [LOG MEM] phase=writer_entry up_us=1882844 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:53:11.451 RX [LOG MEM] phase=after_formatter up_us=1882927 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:53:11.451 RX [LOG MEM] phase=before_mount up_us=1883487 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:53:11.451 RX [LOG MEM] phase=after_mount up_us=2054886 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:53:11.451 RX [LOG MEM] phase=before_current_open up_us=2062178 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:53:11.451 RX [LOG MEM] phase=after_current_open up_us=2063670 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:53:11.451 RX [LOG MEM] phase=storage_done up_us=2077887 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:53:15.152 RX Screen touched, resetting inactivity timer.
2026-09-16 14:53:15.153 RX Latest button clicked
2026-09-16 14:53:15.153 RX Initiating async latest image request...
2026-09-16 14:53:15.153 RX Preparing UI for new image request...
2026-09-16 14:53:15.153 RX Cleaning up image fetcher state...
2026-09-16 14:53:15.153 RX Screen 2 Loaded.
2026-09-16 14:53:15.224 RX === requestImage('latest') START ===
2026-09-16 14:53:15.225 RX Sending HTTP GET...
2026-09-16 14:53:15.225 RX [PROBE] window=normal run=11 ms=55078 heap_min_boot=84748 largest_min=31732 interval_ms=10 samples=5508 gap_max_us=10770 scan_max_us=517 timer=on imu_n=2701 imu_min_hz=28.52 imu_avg_hz=49.20
2026-09-16 14:53:15.988 RX [PROBE] window=image_https run=1 ms=761 heap_min_boot=36252 largest_min=26612 interval_ms=10 samples=76 gap_max_us=10636 scan_max_us=502 timer=on
2026-09-16 14:53:15.988 RX Response received in 762 ms, Content-Length: 34262
2026-09-16 14:53:15.988 RX Starting to receive image data...
2026-09-16 14:53:16.288 RX Image download complete (34262 bytes, 1135 ms since button press). Starting decode...
2026-09-16 14:53:16.427 RX JPEG decoded successfully into PSRAM.
2026-09-16 14:53:16.427 RX LVGL image source updated. Total 1274 ms from button press (budget 20000 ms).
2026-09-16 14:53:18.005 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 14:53:18.005 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 14:53:20.021 TX log status [CRLF]
2026-09-16 14:53:20.027 RX [LOG] state=disabled boot=24 session=boot-24 up_ms=670724 clock=synced setup=1 hooks=1 file_bytes=2956 generation=6 newest=5 archives=3 card_bytes=15931539456 free_bytes=15922954240 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=test_full_write errno=28
2026-09-16 14:53:20.028 RX [LOG] measured=1 stack_min=3412 internal_min=84748 internal_largest=31732 dma_min=77252 dma_largest=31732 writes=35 slow=0 write_max_us=3822 flush_max_us=4951 sd_max_us=171513 rotations=5 pruned=2 oversized=0
2026-09-16 14:53:20.029 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=parked stack_used_max=4780 stack_final_margin=3412
2026-09-16 14:53:20.029 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 14:53:20.029 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 14:53:20.031 RX [LOG MEM] phase=before_clock up_us=1881890 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 14:53:20.031 RX [LOG MEM] phase=after_clock up_us=1882577 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:53:20.031 RX [LOG MEM] phase=before_writer up_us=1882633 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:53:20.031 RX [LOG MEM] phase=writer_entry up_us=1882844 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:53:20.031 RX [LOG MEM] phase=after_formatter up_us=1882927 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:53:20.031 RX [LOG MEM] phase=before_mount up_us=1883487 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:53:20.031 RX [LOG MEM] phase=after_mount up_us=2054886 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:53:20.031 RX [LOG MEM] phase=before_current_open up_us=2062178 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:53:20.031 RX [LOG MEM] phase=after_current_open up_us=2063670 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 14:53:20.031 RX [LOG MEM] phase=storage_done up_us=2077887 free=129836 largest=65524 heap_min_boot=128704
```


## Reboot after simulated write failure passes - 2026-09-16, boot 25, 14:56-14:58

JP restarted normally and reconnected through the browser. The logger returned
to ready in boot 25, with error=none, errno=0, hooks=1, synced clock and active
valid PSRAM writer placement. The injected full-write flag did not persist.

| Measurement | First recovered status | Later status |
|-------------|------------------------|--------------|
| Boot / uptime | 25 / 14947 ms | 25 / 108619 ms |
| Generation / newest / archives | 6 / 5 / 3 | unchanged |
| Current file bytes | 4108 | 4664 |
| Writes | 7 | 8 |
| Rotations / pruned this boot | 0 / 0 | unchanged |
| Writer maximum used / minimum margin | 4604 / 3588 bytes | unchanged |
| Final stack margin | -1, active writer | unchanged |
| Historical internal minimum / largest | 84740 / 31732 bytes | unchanged |
| Errors / drops / slow writes | none / 0 / 0 | unchanged |

The first recovered file size exceeds the pre-restart 2956 bytes, and then grows
another 556 bytes with continued writes. This supports append recovery rather
than truncation. Exact file bytes and headers will be verified in a later card
inspection. Archive generation/count remain intact; the runtime rotation/pruning
counters reset as expected for a new boot.

A full normal probe recorded 60006 ms, 6001 samples at 10 ms, maximum gap
10861 us and scan 436 us. IMU average/minimum were 49.23/35.70 Hz.
Writer placement: 8192-byte PSRAM stack, internal TCB 352, active. The 3588-byte
margin is a new boot's watermark, not recovery of an old boot's watermark.
Final DMA minimum/largest were 77244/31732. Queue peak stayed 1 of 16,
suppressed/truncated counts zero, write/flush/SD maxima 2292/4987/171961 us.

Reboot recovery and continued appends pass on the serial evidence.
Normal limits apply after this reboot; hooks remain enabled for deliberate tests.
No firmware change, build, flash or commit was made.

Next single check: interruption after archive rename and before fresh current
creation, using log test rename. This simulates a rotation boundary without
deliberately removing power during an SD write.
1. Send log status, then log test rename once.
2. Wait for "writer paused at requested boundary; reset the board to continue".
   Send log status to capture the paused state. The UI should remain usable.
3. Disconnect USB and leave the battery unit stationary until normal shutdown.
   The hook's wait exits on the close request; the rotation path skips creating
   current.log when closing, leaving the intended missing-current state.
4. Power back on, reconnect the browser, wait for green MQTT and send log status.
5. Wait 65 seconds on the dashboard and send log status again.

Expected after reboot from generation 6/newest 5: current generation 7, newest
archive 6 and four archives, ready with no logger error/drop and resumed file
growth. The paused pre-reboot status may retain old inventory values because
the rotation has not completed; do not judge completion from those alone.
No commands other than status are needed while paused. If the pause message
does not appear or an error occurs, stop and send that output.
This tests missing-current boundary recovery; it is not a sudden power-loss
durability test. Normal shutdown is preferred here to USB unplug/replug alone,
since the battery can keep the board running.

### Serial evidence

The partial calibration payload line is omitted.

```text
2026-09-16 14:56:21.411 EVENT Console cleared.
2026-09-16 14:56:26.875 EVENT USB serial device available.
2026-09-16 14:56:37.669 EVENT Connect requested. After open: DTR=true, RTS=false.
2026-09-16 14:56:40.099 EVENT Explicit signals applied. After open: DTR=true, RTS=false.
2026-09-16 14:56:40.099 EVENT Connected. Send status; compare with the previous reading if available.
2026-09-16 14:56:40.100 RX Initial MQTT connection successful!
2026-09-16 14:56:40.100 RX Motion baseline reset: Accel=0.98 m/s², Gyro=7.94 °/s (averaged from 20 readings)
2026-09-16 14:56:40.100 RX [NET] WiFi=CONNECTED | MQTT=REMOTE CONNECTED
2026-09-16 14:56:40.100 RX [TEST] Serial bench commands: off, on, status, log status. Send with CR or LF.
2026-09-16 14:56:41.351 TX log status [CRLF]
2026-09-16 14:56:41.355 RX [LOG] state=ready boot=25 session=boot-25 up_ms=14947 clock=synced setup=1 hooks=1 file_bytes=4108 generation=6 newest=5 archives=3 card_bytes=15931539456 free_bytes=15922954240 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 14:56:41.356 RX [LOG] measured=1 stack_min=3588 internal_min=84740 internal_largest=31732 dma_min=77244 dma_largest=31732 writes=7 slow=0 write_max_us=2292 flush_max_us=4987 sd_max_us=171961 rotations=0 pruned=0 oversized=0
2026-09-16 14:56:41.357 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4604 stack_final_margin=-1
2026-09-16 14:56:41.357 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 14:56:41.358 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 14:56:41.358 RX [LOG MEM] phase=before_clock up_us=1781894 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 14:56:41.358 RX [LOG MEM] phase=after_clock up_us=1782584 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:56:41.358 RX [LOG MEM] phase=before_writer up_us=1782640 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:56:41.358 RX [LOG MEM] phase=writer_entry up_us=1782848 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:56:41.358 RX [LOG MEM] phase=after_formatter up_us=1782933 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:56:41.358 RX [LOG MEM] phase=before_mount up_us=1783491 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:56:41.358 RX [LOG MEM] phase=after_mount up_us=1955328 free=129836 largest=65524 heap_min_boot=128496
2026-09-16 14:56:41.359 RX [LOG MEM] phase=before_current_open up_us=1966480 free=129836 largest=65524 heap_min_boot=128496
2026-09-16 14:56:41.359 RX [LOG MEM] phase=after_current_open up_us=1967842 free=129836 largest=65524 heap_min_boot=128496
2026-09-16 14:56:41.359 RX [LOG MEM] phase=storage_done up_us=1983853 free=129836 largest=65524 heap_min_boot=128496
2026-09-16 14:58:12.599 TX log status [CRLF]
2026-09-16 14:58:12.601 RX [PROBE] window=normal run=1 ms=60006 heap_min_boot=84740 largest_min=31732 interval_ms=10 samples=6001 gap_max_us=10861 scan_max_us=436 timer=on imu_n=2948 imu_min_hz=35.70 imu_avg_hz=49.23
2026-09-16 14:58:12.601 RX [LOG] state=ready boot=25 session=boot-25 up_ms=106195 clock=synced setup=1 hooks=1 file_bytes=4664 generation=6 newest=5 archives=3 card_bytes=15931539456 free_bytes=15922954240 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 14:58:12.602 RX [LOG] measured=1 stack_min=3588 internal_min=84740 internal_largest=31732 dma_min=77244 dma_largest=31732 writes=8 slow=0 write_max_us=2292 flush_max_us=4987 sd_max_us=171961 rotations=0 pruned=0 oversized=0
2026-09-16 14:58:12.602 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4604 stack_final_margin=-1
2026-09-16 14:58:12.603 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 14:58:12.603 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 14:58:12.604 RX [LOG MEM] phase=before_clock up_us=1781894 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 14:58:12.604 RX [LOG MEM] phase=after_clock up_us=1782584 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:58:12.604 RX [LOG MEM] phase=before_writer up_us=1782640 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:58:12.604 RX [LOG MEM] phase=writer_entry up_us=1782848 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:58:12.604 RX [LOG MEM] phase=after_formatter up_us=1782933 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:58:12.604 RX [LOG MEM] phase=before_mount up_us=1783491 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:58:12.604 RX [LOG MEM] phase=after_mount up_us=1955328 free=129836 largest=65524 heap_min_boot=128496
2026-09-16 14:58:12.604 RX [LOG MEM] phase=before_current_open up_us=1966480 free=129836 largest=65524 heap_min_boot=128496
2026-09-16 14:58:12.604 RX [LOG MEM] phase=after_current_open up_us=1967842 free=129836 largest=65524 heap_min_boot=128496
2026-09-16 14:58:12.604 RX [LOG MEM] phase=storage_done up_us=1983853 free=129836 largest=65524 heap_min_boot=128496
2026-09-16 14:58:15.021 TX log status [CRLF]
2026-09-16 14:58:15.025 RX [LOG] state=ready boot=25 session=boot-25 up_ms=108619 clock=synced setup=1 hooks=1 file_bytes=4664 generation=6 newest=5 archives=3 card_bytes=15931539456 free_bytes=15922954240 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 14:58:15.025 RX [LOG] measured=1 stack_min=3588 internal_min=84740 internal_largest=31732 dma_min=77244 dma_largest=31732 writes=8 slow=0 write_max_us=2292 flush_max_us=4987 sd_max_us=171961 rotations=0 pruned=0 oversized=0
2026-09-16 14:58:15.027 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4604 stack_final_margin=-1
2026-09-16 14:58:15.027 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 14:58:15.028 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 14:58:15.028 RX [LOG MEM] phase=before_clock up_us=1781894 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 14:58:15.028 RX [LOG MEM] phase=after_clock up_us=1782584 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:58:15.028 RX [LOG MEM] phase=before_writer up_us=1782640 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:58:15.028 RX [LOG MEM] phase=writer_entry up_us=1782848 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:58:15.030 RX [LOG MEM] phase=after_formatter up_us=1782933 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:58:15.030 RX [LOG MEM] phase=before_mount up_us=1783491 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 14:58:15.030 RX [LOG MEM] phase=after_mount up_us=1955328 free=129836 largest=65524 heap_min_boot=128496
2026-09-16 14:58:15.030 RX [LOG MEM] phase=before_current_open up_us=1966480 free=129836 largest=65524 heap_min_boot=128496
2026-09-16 14:58:15.030 RX [LOG MEM] phase=after_current_open up_us=1967842 free=129836 largest=65524 heap_min_boot=128496
2026-09-16 14:58:15.030 RX [LOG MEM] phase=storage_done up_us=1983853 free=129836 largest=65524 heap_min_boot=128496
```


## Interrupted-rename recovery runtime passes - 2026-09-16, boot 26, 15:02-15:03

JP supplied post-restart status, then explicitly confirmed he sent log test rename
and saw "writer paused at requested boundary" before powering down.
The pre-restart command/pause text is not in this capture; hook execution is
supported by JP's confirmation, with the recovered state measured below.

| Measurement | First recovered status | Later status |
|-------------|------------------------|--------------|
| Boot / uptime | 26 / 24725 ms | 26 / 91318 ms |
| State / error / drops | ready / none / 0 | unchanged |
| Generation / newest / archives | 7 / 6 / 4 | unchanged |
| Current file bytes / writes | 1259 / 8 | 1815 / 9 |
| Rotations / pruned in this boot | 0 / 0 | unchanged |
| Writer used / margin | 4604 / 3588 bytes | unchanged |
| Historical internal minimum / largest | 84912 / 31732 bytes | unchanged |

Generation 7, newest archive 6 and four archives match recovery after preserving
generation 6 and leaving no current file at the injected boundary. The new file
grew by 556 bytes and logging remained ready. The serial runtime check passes;
the missing-current state was not directly inspected on card, and post-recovery
header/archive contents remain for later file verification.

Writer placement was valid and active: 8192-byte PSRAM stack, internal TCB 352,
final margin=-1 while running. DMA minimum/largest=77416/31732, queue peak 1 of 16,
no suppressed/truncated records or slow writes. Write/flush/SD maxima were
2231/5085/161617 us. The normal probe covered 60001 ms, 6000 samples at 10 ms,
gap_max_us=11019, scan_max_us=1173, IMU average/minimum 49.19/33.33 Hz.
No new reset occurs within the supplied recovered-boot capture.

Next single check is partial-header salvage using log test partial, normal limits,
the existing enabled PSRAM/hooks build and browser monitoring. From generation 7:
the hook archives the current as archive 7, creates the next current with the
literal incomplete prefix "local=unknown time=unk", flushes it and pauses.
Normal shutdown may append shutdown/queued records after that prefix, but does
not write a valid FILE_OPEN header at the start. The file therefore remains a
nonempty invalid-header case, not an empty-current case or an abrupt-power-loss test.

Procedure: log status, log test partial once, wait for the boundary-pause message,
log status and save this pre-shutdown console text. Disconnect USB, let the
stationary battery board shut down normally, then power on/reconnect browser.
After green MQTT, log status, wait 65 seconds, log status. Supply both captures.
Expected: the invalid current is preserved as archive 8, a fresh current has
generation 9, newest=8, archives=6, logging ready without errors/drops and file
growth resumed. Normal retention should not prune at these counts/sizes.
Exact preserved invalid-header bytes and new headers still need later disk inspection.
No rebuild or card removal. Stop and report if the pause message does not appear
or a logger error occurs. No firmware edits, build, flash or commit for this result.


## Partial-header salvage runtime passes - 2026-09-16, boots 26 to 27, 15:06-15:08

The capture includes both sides of the test. Before injection: boot 26,
generation 7, newest 6, four archives, current 2934 bytes and logger ready.
log test partial was sent at 15:06:18.800, followed by the boundary-pause message
at 15:06:18.870. Paused status showed file_bytes=22; generation/inventory fields
still reflected the incomplete rotation's earlier snapshot, as expected.
The source writes and flushes the literal partial prefix "local=unknown time=unk".

| Measurement | Paused boot 26 | Recovered boot 27 | Later boot 27 |
|-------------|----------------|-------------------|---------------|
| State / errors / drops | ready / none / 0 | ready / none / 0 | unchanged |
| Generation / newest / archives | old snapshot 7 / 6 / 4 | 9 / 8 / 6 | unchanged |
| Current file bytes | 22 | 1284 | 1840 |
| Writes | 14 | 8 | 9 |
| Writer used / margin | 4780 / 3412 | 4604 / 3588 | unchanged |
| Historical internal minimum / largest | 84912 / 31732 | 84788 / 31732 | unchanged |

The USB device was lost at 15:06:41 and available again at 15:07:18.
This is consistent with the requested disconnect/restart sequence; the browser
read error during cable removal is not an SD logger error. No serial trace
captures shutdown itself. The file may contain additional shutdown records after
the injected partial prefix; exact bytes will be checked on card.

Recovered generation 9, newest archive 8 and six archives match preservation of
the prior current as archive 7 and the invalid-header file as archive 8.
There was no pruning reported in the recovered boot. Current file grew 556 bytes
and the logger remained healthy. This is a runtime recovery pass; preservation,
prefix bytes, new FILE_OPEN/BOOT headers and exact survivors still need direct
card inspection before declaring the on-disk gate complete.

Placement remained valid with an active 8192-byte PSRAM stack and internal
352-byte TCB. The recovered stack margin stayed 3588 bytes; final margin=-1
while running. Queue peak stayed 1 of 16, with no suppressed/truncated records
or slow writes. Final DMA minimum/largest=77292/31732.
Write/flush/SD maxima after restart: 2205/5745/163582 us.
Normal IMU average/minimum before restart: 49.18/35.64 Hz; afterward 49.15/29.41 Hz.
Recovered normal probe: 60002 ms, 6000 samples, max gap 10891 us, max scan 429 us.
These are separate boot observations, not a paired performance comparison.

Next single check: power down normally on battery, then copy the entire /logs
folder from the card for inspection and backup. Need current.log and all six
archives, expected archive-00000003.log through archive-00000008.log.
Inspect archive 8's damaged leading header without repairing it, archive 7's
prior current, generation 9 FILE_OPEN and BOOT records, per-boot sequences across
files, absence of managed archives 1 and 2, and final shutdown.
Generation-7 records may include BOOT context=new for the interrupted-rename
recovery. Identify actual events from files; do not infer exact data from counters.

Keep original card files unchanged. A folder path accessible to the assistant or
a zip of the copied folder suffices. No rebuild or firmware change is needed.
This combines disk verification and backup after multiple storage tests.
Empty-current recovery, natural size-triggered rotation, unrelated-name deletion
safety, clock/breadcrumb/deep-sleep and other remaining gates are not silently
accepted from this result. No build, flash or commit for this turn.

### Serial evidence

The partial calibration payload line is omitted.

```text
2026-09-16 15:05:51.873 EVENT Console cleared.
2026-09-16 15:06:08.059 TX log status [CRLF]
2026-09-16 15:06:08.065 RX [LOG] state=ready boot=26 session=boot-26 up_ms=232142 clock=synced setup=1 hooks=1 file_bytes=2934 generation=7 newest=6 archives=4 card_bytes=15931539456 free_bytes=15922921472 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 15:06:08.066 RX [LOG] measured=1 stack_min=3588 internal_min=84912 internal_largest=31732 dma_min=77416 dma_largest=31732 writes=11 slow=0 write_max_us=2231 flush_max_us=5085 sd_max_us=161617 rotations=0 pruned=0 oversized=0
2026-09-16 15:06:08.068 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4604 stack_final_margin=-1
2026-09-16 15:06:08.068 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 15:06:08.068 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 15:06:08.068 RX [LOG MEM] phase=before_clock up_us=1781898 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 15:06:08.068 RX [LOG MEM] phase=after_clock up_us=1782588 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:06:08.068 RX [LOG MEM] phase=before_writer up_us=1782644 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:06:08.068 RX [LOG MEM] phase=writer_entry up_us=1782853 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:06:08.069 RX [LOG MEM] phase=after_formatter up_us=1782937 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:06:08.069 RX [LOG MEM] phase=before_mount up_us=1783497 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:06:08.069 RX [LOG MEM] phase=after_mount up_us=1944990 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 15:06:08.069 RX [LOG MEM] phase=before_current_open up_us=1961629 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 15:06:08.069 RX [LOG MEM] phase=after_current_open up_us=1963223 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 15:06:08.069 RX [LOG MEM] phase=storage_done up_us=1985625 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 15:06:18.800 TX log test partial [CRLF]
2026-09-16 15:06:18.804 RX [LOG TEST] hook queued
2026-09-16 15:06:18.870 RX [LOG TEST] writer paused at requested boundary; reset the board to continue
2026-09-16 15:06:24.864 RX [PROBE] window=normal run=4 ms=60000 heap_min_boot=84912 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10889 scan_max_us=1065 timer=on imu_n=2945 imu_min_hz=35.64 imu_avg_hz=49.18
2026-09-16 15:06:33.218 TX log status [CRLF]
2026-09-16 15:06:33.224 RX [LOG] state=ready boot=26 session=boot-26 up_ms=257301 clock=synced setup=1 hooks=1 file_bytes=22 generation=7 newest=6 archives=4 card_bytes=15931539456 free_bytes=15922921472 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 15:06:33.224 RX [LOG] measured=1 stack_min=3412 internal_min=84912 internal_largest=31732 dma_min=77416 dma_largest=31732 writes=14 slow=0 write_max_us=3908 flush_max_us=6463 sd_max_us=161617 rotations=0 pruned=0 oversized=0
2026-09-16 15:06:33.226 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4780 stack_final_margin=-1
2026-09-16 15:06:33.226 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 15:06:33.228 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 15:06:33.228 RX [LOG MEM] phase=before_clock up_us=1781898 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 15:06:33.228 RX [LOG MEM] phase=after_clock up_us=1782588 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:06:33.228 RX [LOG MEM] phase=before_writer up_us=1782644 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:06:33.228 RX [LOG MEM] phase=writer_entry up_us=1782853 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:06:33.228 RX [LOG MEM] phase=after_formatter up_us=1782937 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:06:33.228 RX [LOG MEM] phase=before_mount up_us=1783497 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:06:33.229 RX [LOG MEM] phase=after_mount up_us=1944990 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 15:06:33.229 RX [LOG MEM] phase=before_current_open up_us=1961629 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 15:06:33.229 RX [LOG MEM] phase=after_current_open up_us=1963223 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 15:06:33.229 RX [LOG MEM] phase=storage_done up_us=1985625 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 15:06:39.245 RX Movement Detected! (Accel: 0.06, Gyro: 36.08)
2026-09-16 15:06:39.246 RX TX motion MQTT: Moving (immediate)
2026-09-16 15:06:41.251 ERROR Read: The device has been lost.
2026-09-16 15:06:41.252 EVENT read stream ended
2026-09-16 15:06:41.257 EVENT Disconnected. Keep USB plugged in and check the board. Reconnect here and send status to compare uptime.
2026-09-16 15:06:41.304 EVENT USB serial device removed.
2026-09-16 15:07:18.233 EVENT USB serial device available.
2026-09-16 15:07:31.577 EVENT Connect requested. After open: DTR=true, RTS=false.
2026-09-16 15:07:34.126 EVENT Explicit signals applied. After open: DTR=true, RTS=false.
2026-09-16 15:07:34.126 EVENT Connected. Send status; compare with the previous reading if available.
2026-09-16 15:07:34.126 RX Initial MQTT connection successful!
2026-09-16 15:07:34.126 RX Motion baseline reset: Accel=0.98 m/s², Gyro=7.60 °/s (averaged from 20 readings)
2026-09-16 15:07:34.126 RX [NET] WiFi=CONNECTED | MQTT=REMOTE CONNECTED
2026-09-16 15:07:34.126 RX [TEST] Serial bench commands: off, on, status, log status. Send with CR or LF.
2026-09-16 15:07:38.302 TX log status [CRLF]
2026-09-16 15:07:38.304 RX [LOG] state=ready boot=27 session=boot-27 up_ms=20351 clock=synced setup=1 hooks=1 file_bytes=1284 generation=9 newest=8 archives=6 card_bytes=15931539456 free_bytes=15922855936 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 15:07:38.304 RX [LOG] measured=1 stack_min=3588 internal_min=84788 internal_largest=31732 dma_min=77292 dma_largest=31732 writes=8 slow=0 write_max_us=2205 flush_max_us=3849 sd_max_us=163582 rotations=0 pruned=0 oversized=0
2026-09-16 15:07:38.306 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4604 stack_final_margin=-1
2026-09-16 15:07:38.306 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 15:07:38.306 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 15:07:38.306 RX [LOG MEM] phase=before_clock up_us=1781895 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 15:07:38.306 RX [LOG MEM] phase=after_clock up_us=1782585 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:07:38.307 RX [LOG MEM] phase=before_writer up_us=1782641 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:07:38.307 RX [LOG MEM] phase=writer_entry up_us=1782850 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:07:38.308 RX [LOG MEM] phase=after_formatter up_us=1782934 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:07:38.308 RX [LOG MEM] phase=before_mount up_us=1783493 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:07:38.308 RX [LOG MEM] phase=after_mount up_us=1946957 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 15:07:38.308 RX [LOG MEM] phase=before_current_open up_us=1971392 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 15:07:38.308 RX [LOG MEM] phase=after_current_open up_us=1973755 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 15:07:38.308 RX [LOG MEM] phase=storage_done up_us=2000498 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 15:07:44.985 RX Movement Detected! (Accel: 0.05, Gyro: 1.06)
2026-09-16 15:07:44.986 RX TX motion MQTT: Moving (immediate)
2026-09-16 15:08:14.990 RX TX motion MQTT: Moving (periodic)
2026-09-16 15:08:14.995 RX Movement Stopped.
2026-09-16 15:08:26.345 RX [PROBE] window=normal run=1 ms=60002 heap_min_boot=84788 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10891 scan_max_us=429 timer=on imu_n=2943 imu_min_hz=29.41 imu_avg_hz=49.15
2026-09-16 15:08:35.548 TX log status [CRLF]
2026-09-16 15:08:35.552 RX [LOG] state=ready boot=27 session=boot-27 up_ms=77601 clock=synced setup=1 hooks=1 file_bytes=1840 generation=9 newest=8 archives=6 card_bytes=15931539456 free_bytes=15922855936 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 15:08:35.553 RX [LOG] measured=1 stack_min=3588 internal_min=84788 internal_largest=31732 dma_min=77292 dma_largest=31732 writes=9 slow=0 write_max_us=2205 flush_max_us=5745 sd_max_us=163582 rotations=0 pruned=0 oversized=0
2026-09-16 15:08:35.554 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4604 stack_final_margin=-1
2026-09-16 15:08:35.554 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 15:08:35.555 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 15:08:35.555 RX [LOG MEM] phase=before_clock up_us=1781895 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 15:08:35.555 RX [LOG MEM] phase=after_clock up_us=1782585 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:08:35.555 RX [LOG MEM] phase=before_writer up_us=1782641 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:08:35.555 RX [LOG MEM] phase=writer_entry up_us=1782850 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:08:35.556 RX [LOG MEM] phase=after_formatter up_us=1782934 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:08:35.556 RX [LOG MEM] phase=before_mount up_us=1783493 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:08:35.556 RX [LOG MEM] phase=after_mount up_us=1946957 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 15:08:35.556 RX [LOG MEM] phase=before_current_open up_us=1971392 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 15:08:35.556 RX [LOG MEM] phase=after_current_open up_us=1973755 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 15:08:35.556 RX [LOG MEM] phase=storage_done up_us=2000498 free=129836 largest=65524 heap_min_boot=128704
```


## 2026-09-16 15:11 card copy: storage recovery contents verified

JP supplied E:\sdcard\logs after normal shutdown. All seven files (15656 bytes)
were copied byte for byte to [the evidence folder](bench_data/sd_logs_2026-09-16_1511/README.md).
SHA-256 hashes were verified after copying. Originals were not modified.

| File | Bytes | Verified contents |
|---|---:|---|
| archive-00000003.log | 584 | Valid generation 3 headers; boot 24 sequences 19-21 |
| archive-00000004.log | 584 | Valid generation 4 headers; boot 24 sequences 22-24 |
| archive-00000005.log | 584 | Valid generation 5 headers; boot 24 sequences 25-27 |
| archive-00000006.log | 6474 | Generation 6; boot 24 sequences 28-35, then boot 25 sequences 1-12 with context=append |
| archive-00000007.log | 3627 | Generation 7, reason=new; boot 26 sequences 1-13 after interrupted rename |
| archive-00000008.log | 153 | Deliberate 22-byte partial prefix followed directly by boot 26 sequence 14 shutdown record |
| current.log | 3650 | Generation 9, reason=corrupt_header, context=boot_after_salvage; boot 27 sequences 1-13 |

The surviving names 3 through 8 match the pruning and recovery sequence.
Archives 1 and 2 are absent as expected after the small-limit pruning test.
The retained boot-24 range 19-35 is contiguous across archives 3 through 6.
Boots 25 and 27 have contiguous complete ranges in their retained files.
Boot 26 has sequences 1-13 in archive 7 and sequence 14 embedded after the
deliberate malformed prefix in archive 8. No other malformed record prefix was
found. FILE_OPEN and BOOT begin with the required common fields in the valid files.

Archive 6 confirms append after the simulated full-write failure: boot 25
continues the same generation instead of replacing boot 24. Boot 24 ends with
TEST_HOOK sequence 35, as expected when the next write disables the logger.
Boot 25 ends at the rename test hook. Its current file was already closed and
renamed before the pause, so no SESSION_END was appended there.

Archive 7 confirms fresh-file creation after interrupted rename. Archive 8
preserves the exact partial prefix, followed by SESSION_END at 15:07:12.526
with reason=shutdown and pending=0. This is expected: normal shutdown releases
the test pause and appends a close record. It is not a valid header or evidence
of an uncontrolled power-loss test. Boot 27 correctly salvaged it rather than
disabling logging.

Current ends with SESSION_END at 15:11:53.054, reason=shutdown, pending=0.
Its health records retain zero drops, suppression and truncation; internal largest
31732 bytes and writer margin 3588 bytes. Initial unknown clock records are followed
by SNTP sync and Montreal offset -0400. This confirms cold-boot unknown-to-synced
ordering, not DST, approximate-time or reset breadcrumb gates.

Outcome: on-card checks pass for retained forced rotations, pruning outcomes,
reboot append, interrupted rename, partial-header salvage and boot-27 shutdown.
Unrelated-file protection, natural size rotation, incomplete tails, truly empty
current recovery and remaining storage/clock/reset gates are still pending.
Stage 1 is not accepted yet.

Next single test: empty-current recovery using log test header, then log test panic
while the writer is paused. The intentional software reset avoids normal shutdown
appending a record into the supposedly empty file. Capture log status before the
hook, at the pause, after restart and after 65 seconds. The queued TEST_PANIC record
may not reach disk because the writer is paused; retained breadcrumbs and reset
reason will need inspection in the next combined card copy.


## 2026-09-16 15:23-15:26 empty-current hook and intentional panic: runtime recovery passed

JP supplied the complete browser capture with header pause, intentional abort and
restart. No additional visual assessment was supplied in this message.

| Check | Result |
|---|---|
| Before hook | Boot 28, ready, generation 9, newest 8, six archives, 5316 bytes |
| Header hook | Queued at 15:23:57.710; paused at 15:23:57.768 |
| Paused status | Cached file_bytes=5447, generation 9 and inventory; writes=9 |
| Deliberate reset | log test panic at 15:24:42.298; abort on core 1 and Rebooting captured |
| Recovery | Boot 29, ready, generation 10, newest 9, seven archives, synced |
| Continued logging | 1397 to 1953 bytes; writes 8 to 9 |
| Memory and stack | internal_min=84748, internal_largest=31732; writer used=4604, margin=3588 |
| Logger counters | No errors, drops, suppression or truncation; queue high=1 |
| MQTT startup probe | 524 ms, 52 samples, largest_min=31732 |
| Normal probe | 60001 ms, 6001 samples, gap_max_us=10517, scan_max_us=436; IMU avg=49.27 Hz, min=27.01 Hz |

Correction to the prior test instruction: file_bytes=0 was not a valid serial
expectation for this pause. createCurrent resets the private sizeBytes to zero
before testPause(Header), but snapshot.size is refreshed by rawWrite only after
a write. The header hook has not written anything at its pause. The displayed
5447 bytes is the prior file's size after the TEST_HOOK record (5316 + 131).
Generation and archive inventory likewise retain the unfinished rotation's old
snapshot. Unlike the partial-header hook, this hook has no prefix write to refresh
the size. No firmware change was made for this reporting limitation.

The captured pause, deliberate abort and successful restart support runtime
recovery from this boundary. Boot 29 retained generation 10 and seven archives,
with continued minute logging. On-card FILE_OPEN reason=empty_recovery, SDK reset
classification and retained breadcrumbs still need verification in the next
combined card copy. ROM rst:0xc is captured; it alone does not substitute for
the SDK reset reason in BOOT. The queued TEST_PANIC record may not reach disk
because the writer was deliberately paused.

The second post-restart status was about 54 seconds after the first, rather
than 65, but it already includes the next minute record; no repeat is needed.
Startup also printed i2c driver install error and ESP_IOExpander init ESP_FAIL.
Setup, QMI8658, Wi-Fi and MQTT subsequently completed. Preserve these messages;
this capture does not establish their cause or a new regression.

Next single test is the existing controlled watchdog hook on the running board.
Keep the card installed, USB and hotspot on, hooks=1 and PSRAM stack=1.
Capture log status, then send log test watchdog once. Wait for its deliberate
watchdog restart; reconnect the browser if needed, capture log status, wait
65 seconds and capture it again. Report if no restart occurs within 30 seconds
before taking recovery actions. Reset class and breadcrumb contents remain for
the later card inspection; the helper's breadcrumb may be superseded by normal
main-loop updates, so a specific retained phase is not assumed.
Stage 1 acceptance remains pending. No firmware changes, build, flash or commit.

### Raw browser capture

```text
2026-09-16 15:23:43.002 EVENT Console cleared.
2026-09-16 15:23:47.431 TX log status [CRLF]
2026-09-16 15:23:47.433 RX [LOG] state=ready boot=28 session=boot-28 up_ms=79119 clock=synced setup=1 hooks=1 file_bytes=5316 generation=9 newest=8 archives=6 card_bytes=15931539456 free_bytes=15922855936 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 15:23:47.434 RX [LOG] measured=1 stack_min=3588 internal_min=84728 internal_largest=31732 dma_min=77232 dma_largest=31732 writes=8 slow=0 write_max_us=2402 flush_max_us=5708 sd_max_us=168545 rotations=0 pruned=0 oversized=0
2026-09-16 15:23:47.434 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4604 stack_final_margin=-1
2026-09-16 15:23:47.435 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 15:23:47.436 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 15:23:47.436 RX [LOG MEM] phase=before_clock up_us=1781893 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 15:23:47.436 RX [LOG MEM] phase=after_clock up_us=1782579 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:23:47.436 RX [LOG MEM] phase=before_writer up_us=1782636 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:23:47.437 RX [LOG MEM] phase=writer_entry up_us=1782847 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:23:47.437 RX [LOG MEM] phase=after_formatter up_us=1782929 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:23:47.437 RX [LOG MEM] phase=before_mount up_us=1783489 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:23:47.437 RX [LOG MEM] phase=after_mount up_us=1951914 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 15:23:47.437 RX [LOG MEM] phase=before_current_open up_us=1968593 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 15:23:47.437 RX [LOG MEM] phase=after_current_open up_us=1971035 free=129644 largest=65524 heap_min_boot=128704
2026-09-16 15:23:47.437 RX [LOG MEM] phase=storage_done up_us=1992559 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 15:23:57.705 TX log test header [CRLF]
2026-09-16 15:23:57.710 RX [LOG TEST] hook queued
2026-09-16 15:23:57.768 RX [LOG TEST] writer paused at requested boundary; reset the board to continue
2026-09-16 15:24:13.146 TX log status [CRLF]
2026-09-16 15:24:13.148 RX [LOG] state=ready boot=28 session=boot-28 up_ms=104835 clock=synced setup=1 hooks=1 file_bytes=5447 generation=9 newest=8 archives=6 card_bytes=15931539456 free_bytes=15922855936 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 15:24:13.148 RX [LOG] measured=1 stack_min=3412 internal_min=84728 internal_largest=31732 dma_min=77232 dma_largest=31732 writes=9 slow=0 write_max_us=2402 flush_max_us=5708 sd_max_us=168545 rotations=0 pruned=0 oversized=0
2026-09-16 15:24:13.150 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4780 stack_final_margin=-1
2026-09-16 15:24:13.150 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 15:24:13.150 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 15:24:13.150 RX [LOG MEM] phase=before_clock up_us=1781893 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 15:24:13.152 RX [LOG MEM] phase=after_clock up_us=1782579 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:24:13.152 RX [LOG MEM] phase=before_writer up_us=1782636 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:24:13.152 RX [LOG MEM] phase=writer_entry up_us=1782847 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:24:13.152 RX [LOG MEM] phase=after_formatter up_us=1782929 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:24:13.152 RX [LOG MEM] phase=before_mount up_us=1783489 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:24:13.152 RX [LOG MEM] phase=after_mount up_us=1951914 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 15:24:13.152 RX [LOG MEM] phase=before_current_open up_us=1968593 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 15:24:13.152 RX [LOG MEM] phase=after_current_open up_us=1971035 free=129644 largest=65524 heap_min_boot=128704
2026-09-16 15:24:13.152 RX [LOG MEM] phase=storage_done up_us=1992559 free=129836 largest=65524 heap_min_boot=128704
2026-09-16 15:24:27.622 RX [PROBE] window=normal run=1 ms=60002 heap_min_boot=84728 largest_min=31732 interval_ms=10 samples=6001 gap_max_us=10742 scan_max_us=672 timer=on imu_n=2947 imu_min_hz=29.35 imu_avg_hz=49.21
2026-09-16 15:24:42.298 TX log test panic [CRLF]
2026-09-16 15:24:42.710 RX
2026-09-16 15:24:42.710 RX abort() was called at PC 0x4200b72f on core 1
2026-09-16 15:24:42.710 RX
2026-09-16 15:24:42.710 RX
2026-09-16 15:24:42.710 RX Backtrace: 0x403768ea:0x3fcc5b40 0x4037fcf9:0x3fcc5b60 0x403867f5:0x3fcc5b80 0x4200b72f:0x3fcc5c00 0x4200df63:0x3fcc5d50 0x42006d30:0x3fcc5d90 0x420489c8:0x3fcc5dd0 0x40380a1a:0x3fcc5df0
2026-09-16 15:24:42.710 RX
2026-09-16 15:24:42.710 RX
2026-09-16 15:24:42.710 RX
2026-09-16 15:24:42.710 RX
2026-09-16 15:24:42.710 RX ELF file SHA256: 53c8d3b75
2026-09-16 15:24:42.710 RX
2026-09-16 15:24:42.710 RX Rebooting...
2026-09-16 15:24:42.710 RX ESP-ROM:esp32s3-20210327
2026-09-16 15:24:42.710 RX Build:Mar 27 2021
2026-09-16 15:24:42.711 RX rst:0xc (RTC_SW_CPU_RST),boot:0x2b (SPI_FAST_FLASH_BOOT)
2026-09-16 15:24:42.711 RX Saved PC:0x4037c5f2
2026-09-16 15:24:42.713 RX SPIWP:0xee
2026-09-16 15:24:42.713 RX mode:DIO, clock div:1
2026-09-16 15:24:42.715 RX load:0x3fce2820,len:0x1188
2026-09-16 15:24:42.717 RX load:0x403c8700,len:0x4
2026-09-16 15:24:42.719 RX load:0x403c8704,len:0xbf0
2026-09-16 15:24:42.721 RX load:0x403cb700,len:0x30e4
2026-09-16 15:24:42.729 RX entry 0x403c88ac
2026-09-16 15:24:43.333 RX
2026-09-16 15:24:43.333 RX === Companion boot === CPU 240 MHz | heap 210116 | PSRAM 8385672
2026-09-16 15:24:43.333 RX [PROBE] sampler=ready interval_ms=10 caps=INTERNAL task=esp_timer
2026-09-16 15:24:43.387 RX
2026-09-16 15:24:43.387 RX --- Board is starting up ---
2026-09-16 15:24:43.389 RX PMIC init OK.
2026-09-16 15:24:43.389 RX Performing full ADC subsystem reset to ensure clean state...
2026-09-16 15:24:43.600 RX PMIC initialization complete
2026-09-16 15:24:43.601 RX E (483) i2c: i2c driver install error
2026-09-16 15:24:43.601 RX E (484) ESP_IOExpander: [ESP_IOExpander.cpp:47] init(): Check error -1 (ESP_FAIL)
2026-09-16 15:24:43.974 RX Display: QSPI clock 20000000 Hz
2026-09-16 15:24:44.031 RX LVGL initialization complete
2026-09-16 15:24:44.031 RX   Button event handlers provided by SquareLine wrappers
2026-09-16 15:24:44.031 RX   Screen event handlers registered
2026-09-16 15:24:44.031 RX   Screen memory event handlers registered
2026-09-16 15:24:44.031 RX   Motion icon configured
2026-09-16 15:24:44.031 RX   Button 2 (Back) custom handler registered
2026-09-16 15:24:44.032 RX [ScreenMem] Restoring to saved screen ID: 1
2026-09-16 15:24:44.032 RX PSRAM found: 8MB
2026-09-16 15:24:44.050 RX QMI8658 Initialized.
2026-09-16 15:24:44.050 RX Getting initial motion state...
2026-09-16 15:24:44.050 RX Startup stabilization: 19 intervals remaining (Accel: 0.00, Gyro: 0.00)
2026-09-16 15:24:44.070 RX Startup stabilization: 18 intervals remaining (Accel: 0.00, Gyro: 0.00)
2026-09-16 15:24:44.095 RX Startup stabilization: 17 intervals remaining (Accel: 0.00, Gyro: 0.00)
2026-09-16 15:24:44.110 RX Startup stabilization: 16 intervals remaining (Accel: 0.00, Gyro: 0.00)
2026-09-16 15:24:44.130 RX Startup stabilization: 15 intervals remaining (Accel: 0.00, Gyro: 0.00)
2026-09-16 15:24:44.153 RX Accelerometer and Gyroscope configured for continuous reading
2026-09-16 15:24:44.674 RX Motion baseline reset: Accel=0.98 m/s², Gyro=8.35 °/s (averaged from 20 readings)
2026-09-16 15:24:44.674 RX [Calib] ----------------------------------------
2026-09-16 15:24:44.674 RX [Calib] Loading Calibration from NVS...
2026-09-16 15:24:44.675 RX [Calib] [OK] Gravity & Scale loaded:
2026-09-16 15:24:44.675 RX     Scale Factor: 1.0030
2026-09-16 15:24:44.675 RX     Gravity Vec:  [ 0.0500,  0.0780, -0.9987]
2026-09-16 15:24:44.676 RX [Calib] [OK] Rotation Matrix loaded:
2026-09-16 15:24:44.676 RX     Row 0 (Vert): [ 0.1812, -0.9811, -0.0676]
2026-09-16 15:24:44.676 RX     Row 1 (Horz): [-0.9822, -0.1771, -0.0630]
2026-09-16 15:24:44.676 RX     Row 2 (Up):   [ 0.0498,  0.0778, -0.9957]
2026-09-16 15:24:44.676 RX [Calib] ----------------------------------------
2026-09-16 15:24:44.677 RX USB Power Connected - Sleep disabled
2026-09-16 15:24:44.678 RX USB power detected - sleep disabled
2026-09-16 15:24:44.679 RX Forcing full UI refresh before WiFi connection...
2026-09-16 15:24:44.878 RX --- Initializing WiFi ---
2026-09-16 15:24:44.927 RX Attempting to connect to primary network: iphone-jp
2026-09-16 15:24:44.927 RX Starting non-blocking scan for SSID: iphone-jp
2026-09-16 15:24:45.039 RX Startup stabilization: 14 intervals remaining (Accel: 0.00, Gyro: 0.10)
2026-09-16 15:24:45.039 RX === Calibrating Gyro Bias (please keep stationary) ===
2026-09-16 15:24:45.039 RX [LOG] ready file=/logs/current.log boot=29 queue_bytes=8192 stack_bytes=8192 core=0
2026-09-16 15:24:45.100 RX   Gyro bias calculated from 200 samples (sensor coordinates):
2026-09-16 15:24:45.100 RX     x=6.219 °/s, y=-5.324 °/s, z=0.067 °/s
2026-09-16 15:24:45.100 RX === Inclinometer Initialized ===
2026-09-16 15:24:45.100 RX   acc_inertial: vert=0.034 horiz=0.037 up=0.978
2026-09-16 15:24:45.100 RX   Initial Pitch: 2.02°
2026-09-16 15:24:45.100 RX   Initial Roll: 2.19°
2026-09-16 15:24:45.102 RX [NET] WiFi=OFFLINE | MQTT=DISCONNECTED
2026-09-16 15:24:45.164 RX Startup stabilization: 13 intervals remaining (Accel: 0.00, Gyro: 0.02)
2026-09-16 15:24:45.221 RX Startup stabilization: 12 intervals remaining (Accel: 0.00, Gyro: 0.29)
2026-09-16 15:24:45.279 RX Startup stabilization: 11 intervals remaining (Accel: 0.00, Gyro: 0.21)
2026-09-16 15:24:45.335 RX Startup stabilization: 10 intervals remaining (Accel: 0.00, Gyro: 0.22)
2026-09-16 15:24:45.400 RX Startup stabilization: 9 intervals remaining (Accel: 0.00, Gyro: 0.25)
2026-09-16 15:24:45.452 RX Startup stabilization: 8 intervals remaining (Accel: 0.00, Gyro: 0.34)
2026-09-16 15:24:45.509 RX Startup stabilization: 7 intervals remaining (Accel: 0.00, Gyro: 0.23)
2026-09-16 15:24:45.566 RX Startup stabilization: 6 intervals remaining (Accel: 0.00, Gyro: 0.44)
2026-09-16 15:24:45.626 RX Startup stabilization: 5 intervals remaining (Accel: 0.00, Gyro: 0.15)
2026-09-16 15:24:45.683 RX Startup stabilization: 4 intervals remaining (Accel: 0.00, Gyro: 0.32)
2026-09-16 15:24:45.750 RX Startup stabilization: 3 intervals remaining (Accel: 0.00, Gyro: 0.06)
2026-09-16 15:24:45.797 RX Startup stabilization: 2 intervals remaining (Accel: 0.00, Gyro: 0.59)
2026-09-16 15:24:45.855 RX Startup stabilization: 1 intervals remaining (Accel: 0.00, Gyro: 0.11)
2026-09-16 15:24:45.910 RX Startup stabilization: 0 intervals remaining (Accel: 0.00, Gyro: 0.45)
2026-09-16 15:24:47.778 RX
2026-09-16 15:24:47.778 RX Scan complete. Found 9 networks.
2026-09-16 15:24:47.778 RX Specified SSID found.
2026-09-16 15:24:47.781 RX Connecting to WiFi
2026-09-16 15:24:47.781 RX .
2026-09-16 15:24:48.290 RX WiFi connected.
2026-09-16 15:24:48.290 RX SSID: iphone-jp
2026-09-16 15:24:48.290 RX IP: 172.20.10.2
2026-09-16 15:24:48.291 RX INFO: Wi-Fi Power Save disabled for stability.
2026-09-16 15:24:48.291 RX Connection successful!
2026-09-16 15:24:48.291 RX WiFi connection established successfully.
2026-09-16 15:24:48.291 RX Allowing network stack to stabilize...
2026-09-16 15:24:48.295 RX [NET] WiFi=CONNECTED | MQTT=DISCONNECTED
2026-09-16 15:24:50.382 RX WiFi initialization complete
2026-09-16 15:24:50.382 RX --- Initializing MQTT ---
2026-09-16 15:24:50.382 RX Attempting initial MQTT connection...
2026-09-16 15:24:51.007 RX [PROBE] window=mqtt_connect run=1 ms=524 heap_min_boot=84748 largest_min=31732 interval_ms=10 samples=52 gap_max_us=10625 scan_max_us=358 timer=on
2026-09-16 15:24:51.013 RX [MQTT] Calibration sent: {"car_unit":true,"new_calib":false,"scale":1.0030,"gravity":[0.0500,0.0780,-0.9987],"rotation":[[0.1812,-0.9811,-0.0676],[-0.9822,-0.1771,-0.0630],[0.0498,0.0778,-0.9957]]}
2026-09-16 15:24:51.013 RX Initial MQTT connection successful!
2026-09-16 15:24:51.434 RX Motion baseline reset: Accel=0.98 m/s², Gyro=8.23 °/s (averaged from 20 readings)
2026-09-16 15:24:51.434 RX [NET] WiFi=CONNECTED | MQTT=REMOTE CONNECTED
2026-09-16 15:24:51.478 RX --- Setup complete: CPU 240 MHz | heap 90316 | PSRAM 8336052 ---
2026-09-16 15:24:51.478 RX
2026-09-16 15:24:51.478 RX [TEST] Serial bench commands: off, on, status, log status. Send with CR or LF.
2026-09-16 15:25:07.520 TX log status [CRLF]
2026-09-16 15:25:07.522 RX [LOG] state=ready boot=29 session=boot-29 up_ms=24426 clock=synced setup=1 hooks=1 file_bytes=1397 generation=10 newest=9 archives=7 card_bytes=15931539456 free_bytes=15922823168 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 15:25:07.522 RX [LOG] measured=1 stack_min=3588 internal_min=84748 internal_largest=31732 dma_min=77252 dma_largest=31732 writes=8 slow=0 write_max_us=2191 flush_max_us=5189 sd_max_us=98417 rotations=0 pruned=0 oversized=0
2026-09-16 15:25:07.524 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4604 stack_final_margin=-1
2026-09-16 15:25:07.524 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 15:25:07.526 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 15:25:07.526 RX [LOG MEM] phase=before_clock up_us=1781726 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 15:25:07.526 RX [LOG MEM] phase=after_clock up_us=1782433 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:25:07.526 RX [LOG MEM] phase=before_writer up_us=1782496 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:25:07.526 RX [LOG MEM] phase=writer_entry up_us=1782702 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:25:07.526 RX [LOG MEM] phase=after_formatter up_us=1782781 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:25:07.526 RX [LOG MEM] phase=before_mount up_us=1783290 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:25:07.526 RX [LOG MEM] phase=after_mount up_us=1881596 free=129692 largest=65524 heap_min_boot=128624
2026-09-16 15:25:07.526 RX [LOG MEM] phase=before_current_open up_us=1910877 free=129692 largest=65524 heap_min_boot=128624
2026-09-16 15:25:07.526 RX [LOG MEM] phase=after_current_open up_us=1913401 free=129692 largest=65524 heap_min_boot=128624
2026-09-16 15:25:07.526 RX [LOG MEM] phase=storage_done up_us=1943793 free=129692 largest=65524 heap_min_boot=128624
2026-09-16 15:25:51.478 RX [PROBE] window=normal run=1 ms=60001 heap_min_boot=84748 largest_min=31732 interval_ms=10 samples=6001 gap_max_us=10517 scan_max_us=436 timer=on imu_n=2951 imu_min_hz=27.01 imu_avg_hz=49.27
2026-09-16 15:26:01.765 TX log status [CRLF]
2026-09-16 15:26:01.770 RX [LOG] state=ready boot=29 session=boot-29 up_ms=78675 clock=synced setup=1 hooks=1 file_bytes=1953 generation=10 newest=9 archives=7 card_bytes=15931539456 free_bytes=15922823168 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 15:26:01.770 RX [LOG] measured=1 stack_min=3588 internal_min=84748 internal_largest=31732 dma_min=77252 dma_largest=31732 writes=9 slow=0 write_max_us=2191 flush_max_us=5189 sd_max_us=98417 rotations=0 pruned=0 oversized=0
2026-09-16 15:26:01.771 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4604 stack_final_margin=-1
2026-09-16 15:26:01.772 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 15:26:01.773 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 15:26:01.773 RX [LOG MEM] phase=before_clock up_us=1781726 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 15:26:01.773 RX [LOG MEM] phase=after_clock up_us=1782433 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:26:01.773 RX [LOG MEM] phase=before_writer up_us=1782496 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:26:01.773 RX [LOG MEM] phase=writer_entry up_us=1782702 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:26:01.775 RX [LOG MEM] phase=after_formatter up_us=1782781 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:26:01.775 RX [LOG MEM] phase=before_mount up_us=1783290 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 15:26:01.775 RX [LOG MEM] phase=after_mount up_us=1881596 free=129692 largest=65524 heap_min_boot=128624
2026-09-16 15:26:01.775 RX [LOG MEM] phase=before_current_open up_us=1910877 free=129692 largest=65524 heap_min_boot=128624
2026-09-16 15:26:01.775 RX [LOG MEM] phase=after_current_open up_us=1913401 free=129692 largest=65524 heap_min_boot=128624
2026-09-16 15:26:01.775 RX [LOG MEM] phase=storage_done up_us=1943793 free=129692 largest=65524 heap_min_boot=128624
```


## 2026-09-16 17:44-17:45 controlled watchdog: restart and initial recovery passed

JP supplied the watchdog capture. The deliberately unfed diag_wdt task on CPU 1
was identified by task_wdt about 5.003 seconds after the command (browser timestamps).
At that instant CPU 0 and CPU 1 were in IDLE0 and IDLE1. This is the intended
helper-task timeout, not evidence that the SD writer starved an idle task.
The captured abort and restart are expected test behavior.

| Check | Result |
|---|---|
| Before test | Boot 29, uptime 8368688 ms (about 2 h 19 min), ready, generation 10, 79562 bytes |
| Trigger | log test watchdog at 17:44:22.296 |
| Watchdog | diag_wdt (CPU 1) at 17:44:27.299, followed by abort and reboot |
| Recovery | Boot 30, logger ready at 17:44:30.065; MQTT green at 17:44:36.471 |
| First recovered status | 17:44:54.287, ready, synced, 80950 bytes, writes=7 |
| Inventory | Generation 10, newest archive 9, seven archives retained |
| Memory | internal_min=84744, internal_largest=31732; DMA min/largest=77248/31732 |
| Writer | Valid PSRAM placement, used=4604, margin=3588, active |
| Counters | No errors, drops, suppression or truncation; queue high=1, slow writes=0 |
| MQTT startup probe | 531 ms, 53 samples, largest_min=31732 |
| Normal probe | 60000 ms, 6000 samples, gap_max_us=10891, scan_max_us=514; IMU avg=49.17 Hz, min=36.88 Hz |

The capture ends at the normal probe at 17:45:36.514. That proves the main
application continued running for the measurement window, but it is not a logger
write counter. The requested second post-restart log status is missing.
Next: send log status once on the same running board and report whether the screen
works normally. Compare file_bytes against 80950 and writes against 7. Do not repeat
the watchdog fault. If another reboot or reflash has occurred, record that instead
of treating new-boot counters as comparable.

The initial recovery passes; continued post-restart logging is pending the second
status. SD BOOT reset classification, TEST_WATCHDOG persistence and RTC breadcrumb
validity still need the later combined card inspection. ROM rst:0xc alone cannot
distinguish the SDK panic and task-watchdog classes. The helper's breadcrumb may
be superseded by normal main-loop updates before the timeout.

The same startup i2c install and ESP_IOExpander ESP_FAIL messages seen after the
previous intentional panic appear again. Setup, IMU initialization and MQTT then
complete. No new cause is inferred and no visual confirmation was supplied yet.
This deliberate watchdog test does not explain the earlier spontaneous startup
watchdogs or the separate persistent VS Code monitor-close freeze.

Repository HEAD was JP's 285a131 at analysis time, with diagnostics and test hooks
disabled in the source configuration for normal use. This capture explicitly
reports the installed test firmware as hooks=1 with an active PSRAM logger.
Keep source configuration unchanged; no build or flash is needed for the one
remaining status reading. No firmware changes or commit were made for this result.
Stage 1 acceptance remains pending.

### Raw browser capture

```text
2026-09-16 17:44:02.415 EVENT Console cleared.
2026-09-16 17:44:11.611 TX log status [CRLF]
2026-09-16 17:44:11.617 RX [LOG] state=ready boot=29 session=boot-29 up_ms=8368688 clock=synced setup=1 hooks=1 file_bytes=79562 generation=10 newest=9 archives=7 card_bytes=15931539456 free_bytes=15922757632 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 17:44:11.618 RX [LOG] measured=1 stack_min=3588 internal_min=84656 internal_largest=31732 dma_min=77160 dma_largest=31732 writes=147 slow=0 write_max_us=3434 flush_max_us=6482 sd_max_us=98417 rotations=0 pruned=0 oversized=0
2026-09-16 17:44:11.619 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4604 stack_final_margin=-1
2026-09-16 17:44:11.620 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 17:44:11.620 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 17:44:11.620 RX [LOG MEM] phase=before_clock up_us=1781726 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 17:44:11.620 RX [LOG MEM] phase=after_clock up_us=1782433 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:44:11.620 RX [LOG MEM] phase=before_writer up_us=1782496 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:44:11.621 RX [LOG MEM] phase=writer_entry up_us=1782702 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:44:11.621 RX [LOG MEM] phase=after_formatter up_us=1782781 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:44:11.621 RX [LOG MEM] phase=before_mount up_us=1783290 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:44:11.621 RX [LOG MEM] phase=after_mount up_us=1881596 free=129692 largest=65524 heap_min_boot=128624
2026-09-16 17:44:11.621 RX [LOG MEM] phase=before_current_open up_us=1910877 free=129692 largest=65524 heap_min_boot=128624
2026-09-16 17:44:11.622 RX [LOG MEM] phase=after_current_open up_us=1913401 free=129692 largest=65524 heap_min_boot=128624
2026-09-16 17:44:11.622 RX [LOG MEM] phase=storage_done up_us=1943793 free=129692 largest=65524 heap_min_boot=128624
2026-09-16 17:44:22.296 TX log test watchdog [CRLF]
2026-09-16 17:44:27.299 RX E (8393920) task_wdt: Task watchdog got triggered. The following tasks/users did not reset the watchdog in time:
2026-09-16 17:44:27.301 RX E (8393920) task_wdt:  - diag_wdt (CPU 1)
2026-09-16 17:44:27.305 RX E (8393920) task_wdt: Tasks currently running:
2026-09-16 17:44:27.308 RX E (8393920) task_wdt: CPU 0: IDLE0
2026-09-16 17:44:27.311 RX E (8393920) task_wdt: CPU 1: IDLE1
2026-09-16 17:44:27.314 RX E (8393920) task_wdt: Aborting.
2026-09-16 17:44:27.318 RX E (8393920) task_wdt: Print CPU 1 backtrace
2026-09-16 17:44:27.734 RX
2026-09-16 17:44:27.734 RX
2026-09-16 17:44:27.734 RX
2026-09-16 17:44:27.734 RX
2026-09-16 17:44:27.734 RX Backtrace: 0x4037c5ef:0x3fcc2750 0x4204d611:0x3fcc2770 0x40381b73:0x3fcc2790 0x40380a1a:0x3fcc27b0
2026-09-16 17:44:27.734 RX
2026-09-16 17:44:27.734 RX
2026-09-16 17:44:27.734 RX
2026-09-16 17:44:27.734 RX
2026-09-16 17:44:27.734 RX ELF file SHA256: 53c8d3b75
2026-09-16 17:44:27.734 RX
2026-09-16 17:44:27.734 RX Rebooting...
2026-09-16 17:44:27.734 RX ESP-ROM:esp32s3-20210327
2026-09-16 17:44:27.734 RX Build:Mar 27 2021
2026-09-16 17:44:27.735 RX rst:0xc (RTC_SW_CPU_RST),boot:0x2b (SPI_FAST_FLASH_BOOT)
2026-09-16 17:44:27.735 RX Saved PC:0x4204c928
2026-09-16 17:44:27.737 RX SPIWP:0xee
2026-09-16 17:44:27.737 RX mode:DIO, clock div:1
2026-09-16 17:44:27.739 RX load:0x3fce2820,len:0x1188
2026-09-16 17:44:27.741 RX load:0x403c8700,len:0x4
2026-09-16 17:44:27.743 RX load:0x403c8704,len:0xbf0
2026-09-16 17:44:27.746 RX load:0x403cb700,len:0x30e4
2026-09-16 17:44:27.753 RX entry 0x403c88ac
2026-09-16 17:44:28.357 RX
2026-09-16 17:44:28.357 RX === Companion boot === CPU 240 MHz | heap 210116 | PSRAM 8385672
2026-09-16 17:44:28.357 RX [PROBE] sampler=ready interval_ms=10 caps=INTERNAL task=esp_timer
2026-09-16 17:44:28.412 RX
2026-09-16 17:44:28.412 RX --- Board is starting up ---
2026-09-16 17:44:28.413 RX PMIC init OK.
2026-09-16 17:44:28.413 RX Performing full ADC subsystem reset to ensure clean state...
2026-09-16 17:44:28.625 RX PMIC initialization complete
2026-09-16 17:44:28.625 RX E (483) i2c: i2c driver install error
2026-09-16 17:44:28.625 RX E (484) ESP_IOExpander: [ESP_IOExpander.cpp:47] init(): Check error -1 (ESP_FAIL)
2026-09-16 17:44:28.999 RX Display: QSPI clock 20000000 Hz
2026-09-16 17:44:29.055 RX LVGL initialization complete
2026-09-16 17:44:29.056 RX   Button event handlers provided by SquareLine wrappers
2026-09-16 17:44:29.056 RX   Screen event handlers registered
2026-09-16 17:44:29.056 RX   Screen memory event handlers registered
2026-09-16 17:44:29.056 RX   Motion icon configured
2026-09-16 17:44:29.056 RX   Button 2 (Back) custom handler registered
2026-09-16 17:44:29.056 RX [ScreenMem] Restoring to saved screen ID: 1
2026-09-16 17:44:29.056 RX PSRAM found: 8MB
2026-09-16 17:44:29.075 RX QMI8658 Initialized.
2026-09-16 17:44:29.075 RX Getting initial motion state...
2026-09-16 17:44:29.076 RX Startup stabilization: 19 intervals remaining (Accel: 0.00, Gyro: 0.00)
2026-09-16 17:44:29.096 RX Startup stabilization: 18 intervals remaining (Accel: 0.00, Gyro: 0.00)
2026-09-16 17:44:29.116 RX Startup stabilization: 17 intervals remaining (Accel: 0.00, Gyro: 0.00)
2026-09-16 17:44:29.136 RX Startup stabilization: 16 intervals remaining (Accel: 0.00, Gyro: 0.00)
2026-09-16 17:44:29.156 RX Startup stabilization: 15 intervals remaining (Accel: 0.00, Gyro: 0.00)
2026-09-16 17:44:29.178 RX Accelerometer and Gyroscope configured for continuous reading
2026-09-16 17:44:29.699 RX Motion baseline reset: Accel=0.97 m/s², Gyro=7.67 °/s (averaged from 20 readings)
2026-09-16 17:44:29.699 RX [Calib] ----------------------------------------
2026-09-16 17:44:29.699 RX [Calib] Loading Calibration from NVS...
2026-09-16 17:44:29.700 RX [Calib] [OK] Gravity & Scale loaded:
2026-09-16 17:44:29.700 RX     Scale Factor: 1.0030
2026-09-16 17:44:29.700 RX     Gravity Vec:  [ 0.0500,  0.0780, -0.9987]
2026-09-16 17:44:29.701 RX [Calib] [OK] Rotation Matrix loaded:
2026-09-16 17:44:29.701 RX     Row 0 (Vert): [ 0.1812, -0.9811, -0.0676]
2026-09-16 17:44:29.701 RX     Row 1 (Horz): [-0.9822, -0.1771, -0.0630]
2026-09-16 17:44:29.701 RX     Row 2 (Up):   [ 0.0498,  0.0778, -0.9957]
2026-09-16 17:44:29.701 RX [Calib] ----------------------------------------
2026-09-16 17:44:29.702 RX USB Power Connected - Sleep disabled
2026-09-16 17:44:29.703 RX USB power detected - sleep disabled
2026-09-16 17:44:29.704 RX Forcing full UI refresh before WiFi connection...
2026-09-16 17:44:29.904 RX --- Initializing WiFi ---
2026-09-16 17:44:29.953 RX Attempting to connect to primary network: iphone-jp
2026-09-16 17:44:29.953 RX Starting non-blocking scan for SSID: iphone-jp
2026-09-16 17:44:30.065 RX Startup stabilization: 14 intervals remaining (Accel: 0.00, Gyro: 0.06)
2026-09-16 17:44:30.065 RX === Calibrating Gyro Bias (please keep stationary) ===
2026-09-16 17:44:30.065 RX [LOG] ready file=/logs/current.log boot=30 queue_bytes=8192 stack_bytes=8192 core=0
2026-09-16 17:44:30.137 RX   Gyro bias calculated from 200 samples (sensor coordinates):
2026-09-16 17:44:30.137 RX     x=5.462 °/s, y=-5.441 °/s, z=0.083 °/s
2026-09-16 17:44:30.137 RX === Inclinometer Initialized ===
2026-09-16 17:44:30.137 RX   acc_inertial: vert=0.035 horiz=0.032 up=0.969
2026-09-16 17:44:30.137 RX   Initial Pitch: 2.05°
2026-09-16 17:44:30.137 RX   Initial Roll: 1.89°
2026-09-16 17:44:30.139 RX [NET] WiFi=OFFLINE | MQTT=DISCONNECTED
2026-09-16 17:44:30.201 RX Startup stabilization: 13 intervals remaining (Accel: 0.00, Gyro: 0.03)
2026-09-16 17:44:30.258 RX Startup stabilization: 12 intervals remaining (Accel: 0.00, Gyro: 0.59)
2026-09-16 17:44:30.315 RX Startup stabilization: 11 intervals remaining (Accel: 0.00, Gyro: 0.71)
2026-09-16 17:44:30.372 RX Startup stabilization: 10 intervals remaining (Accel: 0.00, Gyro: 0.20)
2026-09-16 17:44:30.488 RX Startup stabilization: 9 intervals remaining (Accel: 0.00, Gyro: 0.32)
2026-09-16 17:44:30.488 RX Startup stabilization: 8 intervals remaining (Accel: 0.00, Gyro: 0.04)
2026-09-16 17:44:30.545 RX Startup stabilization: 7 intervals remaining (Accel: 0.00, Gyro: 0.07)
2026-09-16 17:44:30.602 RX Startup stabilization: 6 intervals remaining (Accel: 0.00, Gyro: 0.15)
2026-09-16 17:44:30.661 RX Startup stabilization: 5 intervals remaining (Accel: 0.00, Gyro: 0.16)
2026-09-16 17:44:30.718 RX Startup stabilization: 4 intervals remaining (Accel: 0.00, Gyro: 0.28)
2026-09-16 17:44:30.776 RX Startup stabilization: 3 intervals remaining (Accel: 0.00, Gyro: 0.19)
2026-09-16 17:44:30.837 RX Startup stabilization: 2 intervals remaining (Accel: 0.00, Gyro: 0.20)
2026-09-16 17:44:30.890 RX Startup stabilization: 1 intervals remaining (Accel: 0.00, Gyro: 0.02)
2026-09-16 17:44:30.946 RX Startup stabilization: 0 intervals remaining (Accel: 0.00, Gyro: 0.19)
2026-09-16 17:44:32.813 RX
2026-09-16 17:44:32.813 RX Scan complete. Found 9 networks.
2026-09-16 17:44:32.813 RX Specified SSID found.
2026-09-16 17:44:32.815 RX Connecting to WiFi
2026-09-16 17:44:32.815 RX .
2026-09-16 17:44:33.322 RX WiFi connected.
2026-09-16 17:44:33.322 RX SSID: iphone-jp
2026-09-16 17:44:33.322 RX IP: 172.20.10.2
2026-09-16 17:44:33.323 RX INFO: Wi-Fi Power Save disabled for stability.
2026-09-16 17:44:33.323 RX Connection successful!
2026-09-16 17:44:33.323 RX WiFi connection established successfully.
2026-09-16 17:44:33.323 RX Allowing network stack to stabilize...
2026-09-16 17:44:33.326 RX [NET] WiFi=CONNECTED | MQTT=DISCONNECTED
2026-09-16 17:44:35.412 RX WiFi initialization complete
2026-09-16 17:44:35.412 RX --- Initializing MQTT ---
2026-09-16 17:44:35.412 RX Attempting initial MQTT connection...
2026-09-16 17:44:36.044 RX [PROBE] window=mqtt_connect run=1 ms=531 heap_min_boot=84744 largest_min=31732 interval_ms=10 samples=53 gap_max_us=10092 scan_max_us=155 timer=on
2026-09-16 17:44:36.049 RX [MQTT] Calibration sent: {"car_unit":true,"new_calib":false,"scale":1.0030,"gravity":[0.0500,0.0780,-0.9987],"rotation":[[0.1812,-0.9811,-0.0676],[-0.9822,-0.1771,-0.0630],[0.0498,0.0778,-0.9957]]}
2026-09-16 17:44:36.049 RX Initial MQTT connection successful!
2026-09-16 17:44:36.471 RX Motion baseline reset: Accel=0.97 m/s², Gyro=7.74 °/s (averaged from 20 readings)
2026-09-16 17:44:36.471 RX [NET] WiFi=CONNECTED | MQTT=REMOTE CONNECTED
2026-09-16 17:44:36.514 RX --- Setup complete: CPU 240 MHz | heap 90312 | PSRAM 8336040 ---
2026-09-16 17:44:36.514 RX
2026-09-16 17:44:36.514 RX [TEST] Serial bench commands: off, on, status, log status. Send with CR or LF.
2026-09-16 17:44:54.284 TX log status [CRLF]
2026-09-16 17:44:54.287 RX [LOG] state=ready boot=30 session=boot-30 up_ms=26167 clock=synced setup=1 hooks=1 file_bytes=80950 generation=10 newest=9 archives=7 card_bytes=15931539456 free_bytes=15922757632 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 17:44:54.287 RX [LOG] measured=1 stack_min=3588 internal_min=84744 internal_largest=31732 dma_min=77248 dma_largest=31732 writes=7 slow=0 write_max_us=2912 flush_max_us=5331 sd_max_us=111532 rotations=0 pruned=0 oversized=0
2026-09-16 17:44:54.289 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4604 stack_final_margin=-1
2026-09-16 17:44:54.289 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 17:44:54.290 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 17:44:54.290 RX [LOG MEM] phase=before_clock up_us=1783261 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 17:44:54.290 RX [LOG MEM] phase=after_clock up_us=1783951 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:44:54.290 RX [LOG MEM] phase=before_writer up_us=1784007 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:44:54.290 RX [LOG MEM] phase=writer_entry up_us=1784216 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:44:54.291 RX [LOG MEM] phase=after_formatter up_us=1784305 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:44:54.291 RX [LOG MEM] phase=before_mount up_us=1784859 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:44:54.291 RX [LOG MEM] phase=after_mount up_us=1896278 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:44:54.291 RX [LOG MEM] phase=before_current_open up_us=1916517 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:44:54.291 RX [LOG MEM] phase=after_current_open up_us=1918046 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:44:54.292 RX [LOG MEM] phase=storage_done up_us=1945596 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:45:36.514 RX [PROBE] window=normal run=1 ms=60000 heap_min_boot=84744 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10891 scan_max_us=514 timer=on imu_n=2944 imu_min_hz=36.88 imu_avg_hz=49.17
```


## 2026-09-16 17:47 watchdog follow-up: continued logging passed

JP supplied the missing same-boot status at 17:47:52.469.

| Measurement | Initial boot-30 status | Follow-up |
|---|---:|---:|
| Uptime, ms | 26167 | 204353 |
| Current file, bytes | 80950 | 82629 |
| Writes | 7 | 10 |
| Largest internal block, bytes | 31732 | 31732 |
| Minimum internal free heap, bytes | 84744 | 84744 |
| Writer stack margin, bytes | 3588 | 3588 |
| Queue high-water / drops | 1 / 0 | 1 / 0 |

The file gained 1679 bytes and three writes during 178186 ms between statuses.
Logger remained ready, synced and active with valid PSRAM stack placement,
generation 10, newest archive 9 and seven archives. Error=none, errno=0,
suppressed=0, truncated=0 and slow=0. Writer used stayed 4604 bytes.
Normal probes at 17:46:36 and 17:47:36 reported IMU averages 49.22 and 49.21 Hz,
minima 30.30 and 27.02 Hz, and largest_min=31732. JP subsequently confirmed
that the board is working well after the watchdog restart.

Outcome: controlled watchdog trigger, restart and continued runtime logging pass.
No repeat is needed. On-card reset classification and retained breadcrumbs remain
pending the later combined card copy; this is not Stage 1 acceptance.

Next single test: spring DST and restoration of real time, using the installed
hooks=1 firmware. Keep USB and hotspot on, take log status, send log test spring,
wait 20 seconds, take log status, send log test sync, wait 60 seconds, then take
log status. Do not reflash or change source settings for this test.
The hook stops SNTP and sets the board clock to March 8, 2026 01:59:50 EST.
Ten seconds later the Montreal rule should advance to 03:00:00 EDT.
Status exposes clock=approx during injection and clock=synced after real SNTP
returns. It does not expose the local timestamp or offset transition, which must
be verified in the SD records later. Browser timestamps stay on the PC clock.
Do not infer DST success from status alone. If real sync has not returned, retain
that as pending rather than repeatedly injecting another clock test.

No firmware changes, build, flash, commit or push were performed.


## 2026-09-16 17:53-17:55 spring clock hook: runtime and real-time restoration passed

JP supplied the spring-test capture from boot 30. The logger stayed ready and
the clock quality followed synced -> approx -> synced. The spring hook was
accepted at 17:53:41.401; the approx status followed about 33 seconds later,
beyond the hook's ten-second DST boundary. The sync hook was accepted at
17:54:33.656; synced was confirmed at 17:55:39.995. This bounds restoration by
the later status; it does not measure the precise SNTP completion time.

| Measurement | Before spring | During test | After real sync |
|---|---:|---:|---:|
| Uptime, ms | 541785 | 586078 | 671888 |
| Clock quality | synced | approx | synced |
| Current file, bytes | 85436 | 86456 | 87852 |
| Writes | 15 | 19 | 23 |
| Largest internal block, bytes | 31732 | 31732 | 31732 |
| Minimum internal free heap, bytes | 84744 | 84744 | 84744 |
| Writer stack margin, bytes | 3588 | 3588 | 3588 |

Generation 10, newest archive 9 and seven archives stayed unchanged despite the
clock jump. Zero errors, drops, suppression, truncation or slow writes; queue
high-water stayed 1. Writer placement remained valid, used=4604 bytes.
Normal IMU averages around the test were 49.26, 49.18 and 49.20 Hz.
No visual assessment accompanied this capture.

Outcome: clock-test activation, continued logging and return to real synchronized
time pass. The actual Montreal spring offset transition is not visible in serial
status. Verify CLOCK_TEST, CLOCK_OFFSET source=test (-0500 to -0400), timestamp
ordering and CLOCK_TEST_END/CLOCK_SYNC in the later combined SD copy.
Do not mark full DST verification complete from clock=approx alone.

Next single test: the matching autumn transition with log test autumn, wait
20 seconds and log status, then log test sync, wait 60 seconds and log status.
Take an initial log status too. The hook sets November 1, 2026 01:59:50 EDT;
after ten seconds local time should return to 01:00:00 EST. The numeric offset
must distinguish the repeated hour in the SD records. Keep the card installed.
No firmware change, build, flash, commit or push. Stage 1 acceptance remains pending.

### Raw browser capture

```text
2026-09-16 17:53:27.903 EVENT Console cleared.
2026-09-16 17:53:29.892 TX log status [CRLF]
2026-09-16 17:53:29.894 RX [LOG] state=ready boot=30 session=boot-30 up_ms=541785 clock=synced setup=1 hooks=1 file_bytes=85436 generation=10 newest=9 archives=7 card_bytes=15931539456 free_bytes=15922757632 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 17:53:29.894 RX [LOG] measured=1 stack_min=3588 internal_min=84744 internal_largest=31732 dma_min=77248 dma_largest=31732 writes=15 slow=0 write_max_us=2912 flush_max_us=5331 sd_max_us=111532 rotations=0 pruned=0 oversized=0
2026-09-16 17:53:29.896 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4604 stack_final_margin=-1
2026-09-16 17:53:29.896 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 17:53:29.896 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 17:53:29.896 RX [LOG MEM] phase=before_clock up_us=1783261 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 17:53:29.898 RX [LOG MEM] phase=after_clock up_us=1783951 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:53:29.898 RX [LOG MEM] phase=before_writer up_us=1784007 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:53:29.898 RX [LOG MEM] phase=writer_entry up_us=1784216 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:53:29.898 RX [LOG MEM] phase=after_formatter up_us=1784305 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:53:29.898 RX [LOG MEM] phase=before_mount up_us=1784859 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:53:29.898 RX [LOG MEM] phase=after_mount up_us=1896278 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:53:29.898 RX [LOG MEM] phase=before_current_open up_us=1916517 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:53:29.898 RX [LOG MEM] phase=after_current_open up_us=1918046 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:53:29.898 RX [LOG MEM] phase=storage_done up_us=1945596 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:53:36.526 RX [PROBE] window=normal run=9 ms=60001 heap_min_boot=84744 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10814 scan_max_us=428 timer=on imu_n=2950 imu_min_hz=27.75 imu_avg_hz=49.26
2026-09-16 17:53:41.398 TX log test spring [CRLF]
2026-09-16 17:53:41.401 RX [LOG TEST] clock hook applied
2026-09-16 17:54:14.181 TX log status [CRLF]
2026-09-16 17:54:14.186 RX [LOG] state=ready boot=30 session=boot-30 up_ms=586078 clock=approx setup=1 hooks=1 file_bytes=86456 generation=10 newest=9 archives=7 card_bytes=15931539456 free_bytes=15922757632 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 17:54:14.186 RX [LOG] measured=1 stack_min=3588 internal_min=84744 internal_largest=31732 dma_min=77248 dma_largest=31732 writes=19 slow=0 write_max_us=2912 flush_max_us=5516 sd_max_us=111532 rotations=0 pruned=0 oversized=0
2026-09-16 17:54:14.188 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4604 stack_final_margin=-1
2026-09-16 17:54:14.189 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 17:54:14.189 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 17:54:14.189 RX [LOG MEM] phase=before_clock up_us=1783261 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 17:54:14.190 RX [LOG MEM] phase=after_clock up_us=1783951 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:54:14.190 RX [LOG MEM] phase=before_writer up_us=1784007 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:54:14.191 RX [LOG MEM] phase=writer_entry up_us=1784216 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:54:14.191 RX [LOG MEM] phase=after_formatter up_us=1784305 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:54:14.191 RX [LOG MEM] phase=before_mount up_us=1784859 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:54:14.191 RX [LOG MEM] phase=after_mount up_us=1896278 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:54:14.192 RX [LOG MEM] phase=before_current_open up_us=1916517 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:54:14.192 RX [LOG MEM] phase=after_current_open up_us=1918046 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:54:14.192 RX [LOG MEM] phase=storage_done up_us=1945596 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:54:33.651 TX log test sync [CRLF]
2026-09-16 17:54:33.656 RX [LOG TEST] clock hook applied
2026-09-16 17:54:36.527 RX [PROBE] window=normal run=10 ms=60001 heap_min_boot=84744 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10794 scan_max_us=928 timer=on imu_n=2944 imu_min_hz=28.56 imu_avg_hz=49.18
2026-09-16 17:55:36.531 RX [PROBE] window=normal run=11 ms=60004 heap_min_boot=84744 largest_min=31732 interval_ms=10 samples=6001 gap_max_us=10825 scan_max_us=511 timer=on imu_n=2945 imu_min_hz=29.41 imu_avg_hz=49.20
2026-09-16 17:55:39.993 TX log status [CRLF]
2026-09-16 17:55:39.995 RX [LOG] state=ready boot=30 session=boot-30 up_ms=671888 clock=synced setup=1 hooks=1 file_bytes=87852 generation=10 newest=9 archives=7 card_bytes=15931539456 free_bytes=15922757632 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 17:55:39.995 RX [LOG] measured=1 stack_min=3588 internal_min=84744 internal_largest=31732 dma_min=77248 dma_largest=31732 writes=23 slow=0 write_max_us=2912 flush_max_us=5550 sd_max_us=111532 rotations=0 pruned=0 oversized=0
2026-09-16 17:55:39.996 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4604 stack_final_margin=-1
2026-09-16 17:55:39.997 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 17:55:39.998 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 17:55:39.998 RX [LOG MEM] phase=before_clock up_us=1783261 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 17:55:39.998 RX [LOG MEM] phase=after_clock up_us=1783951 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:55:39.998 RX [LOG MEM] phase=before_writer up_us=1784007 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:55:39.998 RX [LOG MEM] phase=writer_entry up_us=1784216 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:55:39.998 RX [LOG MEM] phase=after_formatter up_us=1784305 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:55:39.998 RX [LOG MEM] phase=before_mount up_us=1784859 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:55:39.998 RX [LOG MEM] phase=after_mount up_us=1896278 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:55:39.998 RX [LOG MEM] phase=before_current_open up_us=1916517 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:55:39.998 RX [LOG MEM] phase=after_current_open up_us=1918046 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:55:39.998 RX [LOG MEM] phase=storage_done up_us=1945596 free=129692 largest=65524 heap_min_boot=128588
```


## 2026-09-16 17:57-17:59 autumn clock hook: runtime and restoration passed

JP supplied the autumn capture from the same boot 30. Clock quality followed
synced -> approx -> synced and logging stayed ready throughout.

| Measurement | Before autumn | During test | After real sync |
|---|---:|---:|---:|
| Uptime, ms | 780523 | 816553 | 913915 |
| Current file, bytes | 88414 | 89278 | 90798 |
| Writes | 24 | 27 | 32 |
| Clock quality | synced | approx | synced |
| Largest internal block, bytes | 31732 | 31732 | 31732 |
| Minimum free internal heap, bytes | 84744 | 84744 | 84744 |
| Writer stack margin, bytes | 3588 | 3588 | 3588 |

The hook was accepted at 17:57:33.434; the approx status followed about 31 seconds
later, beyond the ten-second transition boundary. The sync hook was accepted at
17:58:14.851; real sync was confirmed by 17:59:42.016. These browser timestamps
bound confirmation; they do not measure exact SNTP completion.
Generation 10, newest archive 9 and seven archives were unchanged. No errors,
drops, suppression, truncation or slow writes; queue high-water remained 1.
Valid PSRAM stack placement and used=4604 bytes were unchanged.
The file grew 2384 bytes with eight further writes across the captured test.

Normal probes reported IMU averages 49.30, 49.21 and 49.24 Hz, with minima
28.53, 31.24 and 32.21 Hz. Two consecutive minute summaries arrived with the
same browser receive timestamp; each reports about 60000 ms, 6000 samples and
roughly 10-11 ms maximum sampler gap. Receive batching alone does not establish
a firmware pause. No visual assessment accompanied this capture.

Outcome: autumn hook activation, continued logging and restored synchronization
pass their runtime checks. Actual repeated-hour timestamps and the numeric
offset transition remain pending SD inspection, alongside spring DST, boot-29
empty-header recovery and panic/watchdog reset classification and breadcrumbs.

Next: one combined card inspection after normal shutdown on the battery board.
Keep all originals unchanged and copy the complete /logs folder into a new
PC folder, for example E:\sdcard\2026-09-16-evening\logs. Do not overwrite the
earlier copied folder. Supply its path. Expect current.log plus archives 3-9
if no further rotation/pruning occurred. Verify generation-10 FILE_OPEN
reason=empty_recovery, boot-29 panic and boot-30 task_watchdog BOOT records,
validated prior breadcrumbs, both CLOCK_OFFSET source=test transitions,
CLOCK_TEST_END/CLOCK_SYNC, sequence continuity and final SESSION_END.
An absent TEST_PANIC record is expected to be possible while the writer was
paused; do not require it to prove the deliberate panic captured on serial.
Preserve byte-identical copies and hashes before further destructive tests.

Stage 1 acceptance remains pending. No firmware changes, build, flash, commit
or push were performed for this result.

### Raw browser capture

```text
2026-09-16 17:57:12.469 EVENT Console cleared.
2026-09-16 17:57:28.623 TX log status [CRLF]
2026-09-16 17:57:28.627 RX [LOG] state=ready boot=30 session=boot-30 up_ms=780523 clock=synced setup=1 hooks=1 file_bytes=88414 generation=10 newest=9 archives=7 card_bytes=15931539456 free_bytes=15922757632 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 17:57:28.627 RX [LOG] measured=1 stack_min=3588 internal_min=84744 internal_largest=31732 dma_min=77248 dma_largest=31732 writes=24 slow=0 write_max_us=2912 flush_max_us=5550 sd_max_us=111532 rotations=0 pruned=0 oversized=0
2026-09-16 17:57:28.628 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4604 stack_final_margin=-1
2026-09-16 17:57:28.629 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 17:57:28.630 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 17:57:28.630 RX [LOG MEM] phase=before_clock up_us=1783261 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 17:57:28.630 RX [LOG MEM] phase=after_clock up_us=1783951 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:57:28.630 RX [LOG MEM] phase=before_writer up_us=1784007 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:57:28.630 RX [LOG MEM] phase=writer_entry up_us=1784216 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:57:28.630 RX [LOG MEM] phase=after_formatter up_us=1784305 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:57:28.631 RX [LOG MEM] phase=before_mount up_us=1784859 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:57:28.631 RX [LOG MEM] phase=after_mount up_us=1896278 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:57:28.631 RX [LOG MEM] phase=before_current_open up_us=1916517 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:57:28.631 RX [LOG MEM] phase=after_current_open up_us=1918046 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:57:28.631 RX [LOG MEM] phase=storage_done up_us=1945596 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:57:33.427 TX log test autumn [CRLF]
2026-09-16 17:57:33.434 RX [LOG TEST] clock hook applied
2026-09-16 17:57:36.540 RX [PROBE] window=normal run=13 ms=60004 heap_min_boot=84744 largest_min=31732 interval_ms=10 samples=6001 gap_max_us=10894 scan_max_us=736 timer=on imu_n=2952 imu_min_hz=28.53 imu_avg_hz=49.30
2026-09-16 17:58:04.651 TX log status [CRLF]
2026-09-16 17:58:04.656 RX [LOG] state=ready boot=30 session=boot-30 up_ms=816553 clock=approx setup=1 hooks=1 file_bytes=89278 generation=10 newest=9 archives=7 card_bytes=15931539456 free_bytes=15922757632 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 17:58:04.657 RX [LOG] measured=1 stack_min=3588 internal_min=84744 internal_largest=31732 dma_min=77248 dma_largest=31732 writes=27 slow=0 write_max_us=2912 flush_max_us=5550 sd_max_us=111532 rotations=0 pruned=0 oversized=0
2026-09-16 17:58:04.658 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4604 stack_final_margin=-1
2026-09-16 17:58:04.659 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 17:58:04.659 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 17:58:04.659 RX [LOG MEM] phase=before_clock up_us=1783261 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 17:58:04.659 RX [LOG MEM] phase=after_clock up_us=1783951 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:58:04.659 RX [LOG MEM] phase=before_writer up_us=1784007 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:58:04.659 RX [LOG MEM] phase=writer_entry up_us=1784216 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:58:04.659 RX [LOG MEM] phase=after_formatter up_us=1784305 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:58:04.660 RX [LOG MEM] phase=before_mount up_us=1784859 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:58:04.660 RX [LOG MEM] phase=after_mount up_us=1896278 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:58:04.660 RX [LOG MEM] phase=before_current_open up_us=1916517 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:58:04.660 RX [LOG MEM] phase=after_current_open up_us=1918046 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:58:04.660 RX [LOG MEM] phase=storage_done up_us=1945596 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:58:14.848 TX log test sync [CRLF]
2026-09-16 17:58:14.851 RX [LOG TEST] clock hook applied
2026-09-16 17:59:36.541 RX [PROBE] window=normal run=14 ms=60001 heap_min_boot=84744 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10423 scan_max_us=433 timer=on imu_n=2947 imu_min_hz=31.24 imu_avg_hz=49.21
2026-09-16 17:59:36.541 RX [PROBE] window=normal run=15 ms=60002 heap_min_boot=84744 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10977 scan_max_us=1133 timer=on imu_n=2948 imu_min_hz=32.21 imu_avg_hz=49.24
2026-09-16 17:59:42.014 TX log status [CRLF]
2026-09-16 17:59:42.016 RX [LOG] state=ready boot=30 session=boot-30 up_ms=913915 clock=synced setup=1 hooks=1 file_bytes=90798 generation=10 newest=9 archives=7 card_bytes=15931539456 free_bytes=15922757632 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-16 17:59:42.018 RX [LOG] measured=1 stack_min=3588 internal_min=84744 internal_largest=31732 dma_min=77248 dma_largest=31732 writes=32 slow=0 write_max_us=2912 flush_max_us=5550 sd_max_us=111532 rotations=0 pruned=0 oversized=0
2026-09-16 17:59:42.019 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4604 stack_final_margin=-1
2026-09-16 17:59:42.019 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 17:59:42.020 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 17:59:42.020 RX [LOG MEM] phase=before_clock up_us=1783261 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 17:59:42.020 RX [LOG MEM] phase=after_clock up_us=1783951 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:59:42.020 RX [LOG MEM] phase=before_writer up_us=1784007 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:59:42.020 RX [LOG MEM] phase=writer_entry up_us=1784216 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:59:42.020 RX [LOG MEM] phase=after_formatter up_us=1784305 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:59:42.020 RX [LOG MEM] phase=before_mount up_us=1784859 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 17:59:42.020 RX [LOG MEM] phase=after_mount up_us=1896278 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:59:42.020 RX [LOG MEM] phase=before_current_open up_us=1916517 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:59:42.020 RX [LOG MEM] phase=after_current_open up_us=1918046 free=129692 largest=65524 heap_min_boot=128588
2026-09-16 17:59:42.020 RX [LOG MEM] phase=storage_done up_us=1945596 free=129692 largest=65524 heap_min_boot=128588
```


## 2026-09-16 evening card inspection: clocks and reset evidence verified; empty-file coverage corrected

JP supplied F:\2026-09-16-evening\logs after normal shutdown.
All eight files, 109508 bytes, were preserved byte for byte with verified SHA-256
hashes in [the evening evidence folder](bench_data/sd_logs_2026-09-16_1802/README.md).
Originals were not modified. Archives 3 through 8 exactly match the earlier
15:11 backup. Archive 9 is 5447 bytes and current.log is 92055 bytes.

Retained sequences are contiguous per boot across files: boot 24 sequences
19-35, boot 25 1-12, boot 26 1-14, boot 27 1-13, boot 28 1-9, boot 29 1-148
and boot 30 1-35. Boot 26 sequence 14 remains embedded after the deliberately
malformed 22-byte prefix in archive 8. No additional malformed prefixes were found.
BOOT event timestamps can precede FILE_OPEN because BOOT carries its setup-time
stamp; sequence is writer ordering, not a guarantee of monotonic event uptime.

### Verified reset and clock records

| Check | SD evidence |
|---|---|
| Panic classification | Boot 29 seq 2: reset=panic, reset_code=4 |
| Panic breadcrumb | Boot 29 seq 3: main valid=1, prior_boot=28, phase=test_panic, operation=1 |
| Panic writer breadcrumb | Boot 29 seq 4: writer valid=1, prior_boot=28, phase=idle |
| Watchdog trigger record | Boot 29 seq 148: TEST_WATCHDOG source=serial |
| Watchdog classification | Boot 30 seq 1: reset=task_watchdog, reset_code=6, context=append |
| Watchdog breadcrumbs | Boot 30 seq 2-3: both valid=1, prior_boot=29; main=test_watchdog, writer=idle |
| Spring injection | Boot 30 seq 17: 2026-03-08 01:59:50.000 -05:00, time=approx, clock_source=test |
| Spring transition | Seq 19: 03:00:00.976 -04:00, CLOCK_OFFSET from=-0500 to=-0400, source=test |
| Spring restoration | Seq 21 unknown CLOCK_TEST_END, seq 22 synced CLOCK_SYNC, 1892 ms apart by uptime |
| Autumn injection | Seq 26: 2026-11-01 01:59:50.000 -04:00, time=approx, clock_source=test |
| Autumn transition | Seq 27: 01:00:00.057 -05:00, CLOCK_OFFSET from=-0400 to=-0500, source=test |
| Autumn restoration | Seq 28 unknown CLOCK_TEST_END, seq 29 synced CLOCK_SYNC, 1008 ms apart by uptime; seq 30 restores -0400 |
| Final close | Boot 30 seq 35: SESSION_END at 18:02:11.586 -04:00, reason=shutdown, pending=0 |

The spring boundary record occurs 10976 ms after injection; autumn 10057 ms.
The writer polls the offset, so these are observed boundary records, not a
claim of sub-millisecond transition detection. Spring's initial -0400 to -0500
record is the deliberate jump back to March before the actual -0500 to -0400
DST transition. Numeric offsets disambiguate the repeated autumn hour.
Both DST transitions, test labeling and real-time restoration pass on-card checks.

Both reboot BOOT records are time=approx, followed by real SNTP time=synced.
Boot 29 CLOCK_SYNC reports correction_ms=1950 and boot 30 reports 77582.
The approximate boot-30 wall time is therefore about 77.6 seconds behind the
later synchronized estimate. This validates the need for clock quality, uptime
and sequence fields; pre-sync wall times must not be treated as precise.
The source of that retained-clock error is not established by this capture.
Approx-to-synced recovery and correction recording pass; deep-sleep and brownout
retention are separate, untested paths. Earlier power-on records cover unknown
followed by synced. These intentional reset results do not explain unrelated
startup watchdogs or the VS Code port-close freeze.

### Correction: existing-empty-file recovery was not exercised

Current's first line is generation=10 reason=new, followed by BOOT context=boot.
It is NOT reason=empty_recovery. The generation-9 file was archived successfully,
but the newly created empty current file was absent when boot 29 mounted the card.
The source uses reason=new only for a missing current path. This run therefore
passes interrupted-create/missing-file recovery, not recovery of a surviving
zero-byte file. The earlier runtime pass remains valid but its coverage is narrower.

Source review explains a likely mechanism: createCurrent opens with O_CREAT,
sets private sizeBytes=0, then pauses before writing. testPause calls flushFile,
but flushFile returns immediately when dirty=false. Creation alone does not mark
dirty, so no fsync is issued at this empty boundary. Unpersisted creation metadata
is consistent with the observed missing file; the filesystem internals were not
traced. The partial-header hook does write and flush bytes and is a different case.
No firmware change was made. A later deterministic empty-file fixture or a
reviewed hook-only durability correction is needed before closing this gate.
The absent TEST_PANIC event is expected because its queued record could not be
written during the pause; the reset class and retained test_panic breadcrumb
provide the requested evidence.

### Next single test

With the card already removed, test no-card startup and terminal cleanup using
the installed PSRAM-stack/hooks=1 build. Leave the card out, power on with USB
and hotspot, connect the browser and send log status. Expect disabled with
mount_failed_or_no_card, and a parked writer with a captured final stack margin.
Press Latest once, wait for the image and return to the dashboard, then send
log status. Capture the image probe and both statuses and report screen behavior.
Disabled logger memory snapshots may be stale; use PROBE for the image/TLS
window's 20480-byte gate. No card insertion while powered on, no rebuild or source
configuration change. The source defaults were changed by JP for normal use,
but the installed test firmware in the supplied capture still has hooks=1.

Natural size-triggered rotation, surviving-empty-file recovery, incomplete tails,
unrelated-file preservation, other storage-failure cases and deep-sleep close
remain pending where required by the plan. Stage 1 is not accepted yet.
No firmware changes, build, flash, commit or push.


## 2026-09-16 18:09 no-card startup and Latest: passed

JP supplied both statuses and the Latest capture, and confirmed the board is OK.

| Check | Result |
|---|---|
| Session | Boot 31, hooks=1, PSRAM writer |
| Logger | disabled, mount_failed_or_no_card, errno=5; expected with card absent |
| Cleanup | writer_lifecycle=parked, placement_valid=1, stack_final_margin=4116 |
| Writer stack | 8192 bytes allocated, 4076 maximum used, 4116 margin |
| File activity | Zero writes, no file open attempted, zero rotations/pruning |
| Queue | Empty, high=0, drops=0, suppressed=0, truncated=0 |
| Latest | 34123 bytes, successful decode and LVGL update, 4722 ms total |
| HTTPS window | 4190 ms, 419 samples at 10 ms, largest_min=26612 |
| HTTPS free heap | heap_min_boot=36236 |
| Sampler | gap_max_us=10398, scan_max_us=153 during HTTPS |
| Normal operation | IMU average=49.25 Hz, minimum=38.28 Hz over 798 samples |

Before and after Latest, the logger remained disabled and its static PSRAM task
remained parked, with the same final stack margin. The 102388-byte internal
largest value in LOG is the stopped writer's old snapshot, not the image-time
minimum. The independent PROBE value 26612 passes the unchanged 20480 gate.
The no-card mount/cleanup path did not prevent touch, retrieval, decoding or
return to screen 1 without a preference save. JP confirmed normal visual behavior.

The 4.722-second total is longer than earlier successful image requests, with
4.190 seconds in HTTPS GET before the response. The capture does not separate
DNS, TCP, TLS and server wait, so no network or firmware cause is assigned.
It is within the existing 20-second image budget and no failure is shown.
This is a functional no-card check, not a paired performance comparison.

Outcome: no-card handling and terminal PSRAM-writer cleanup pass. The SD task
is suspended with its static stack and TCB retained until reboot, as designed.
This does not independently test unsupported filesystems, corrupt cards,
deep-sleep cleanup or every other terminal path.

Next: prepare a deterministic surviving-zero-byte current.log fixture on the
card while it is already removed. First obtain JP's actual card root path;
F:\2026-09-16-evening\logs is supplied evidence and must not be assumed to
be the firmware's mounted /logs directory. Do not modify that evidence copy.
After the root is confirmed, verify and back up the current original, preserve
it under the next free managed archive name, create a closed zero-byte current.log
without truncating any existing file, and verify the fixture before JP ejects
the card and boots the board. The existing header/panic hook is not repeated.
No card mutation has been performed yet. Stage 1 acceptance remains pending.
No firmware changes, build, flash, commit or push for this result.

### Raw browser capture

```text
2026-09-16 18:09:08.494 EVENT Console cleared.
2026-09-16 18:09:14.690 TX log status [CRLF]
2026-09-16 18:09:14.691 RX [LOG] state=disabled boot=31 session=boot-31 up_ms=70190 clock=synced setup=1 hooks=1 file_bytes=0 generation=0 newest=0 archives=0 card_bytes=0 free_bytes=0 queue=0/16 high=0 drops=0 suppressed=0 truncated=0 error=mount_failed_or_no_card errno=5
2026-09-16 18:09:14.693 RX [LOG] measured=1 stack_min=4116 internal_min=166252 internal_largest=102388 dma_min=158756 dma_largest=102388 writes=0 slow=0 write_max_us=0 flush_max_us=0 sd_max_us=29369 rotations=0 pruned=0 oversized=0
2026-09-16 18:09:14.693 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=parked stack_used_max=4076 stack_final_margin=4116
2026-09-16 18:09:14.693 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 18:09:14.696 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 18:09:14.696 RX [LOG MEM] phase=before_clock up_us=1785268 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 18:09:14.696 RX [LOG MEM] phase=after_clock up_us=1785958 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 18:09:14.696 RX [LOG MEM] phase=before_writer up_us=1786014 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 18:09:14.696 RX [LOG MEM] phase=writer_entry up_us=1786223 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 18:09:14.696 RX [LOG MEM] phase=after_formatter up_us=1786308 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 18:09:14.696 RX [LOG MEM] phase=before_mount up_us=1786865 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 18:09:14.696 RX [LOG MEM] phase=after_mount up_us=1816586 free=167072 largest=102388 heap_min_boot=166252
2026-09-16 18:09:14.696 RX [LOG MEM] phase=before_current_open captured=0
2026-09-16 18:09:14.696 RX [LOG MEM] phase=after_current_open captured=0
2026-09-16 18:09:14.696 RX [LOG MEM] phase=storage_done up_us=1817024 free=167072 largest=102388 heap_min_boot=166252
2026-09-16 18:09:20.129 RX Screen touched, resetting inactivity timer.
2026-09-16 18:09:20.129 RX Latest button clicked
2026-09-16 18:09:20.129 RX Initiating async latest image request...
2026-09-16 18:09:20.129 RX Preparing UI for new image request...
2026-09-16 18:09:20.129 RX Cleaning up image fetcher state...
2026-09-16 18:09:20.129 RX Screen 2 Loaded.
2026-09-16 18:09:20.201 RX === requestImage('latest') START ===
2026-09-16 18:09:20.201 RX Initiating HTTPS GET: https://photojpl.synology.me:9835/esp32/latest?token=***
2026-09-16 18:09:20.201 RX Sending HTTP GET...
2026-09-16 18:09:20.202 RX [PROBE] window=normal run=1 ms=16301 heap_min_boot=85836 largest_min=31732 interval_ms=10 samples=1630 gap_max_us=10362 scan_max_us=287 timer=on imu_n=798 imu_min_hz=38.28 imu_avg_hz=49.25
2026-09-16 18:09:24.393 RX [PROBE] window=image_https run=1 ms=4190 heap_min_boot=36236 largest_min=26612 interval_ms=10 samples=419 gap_max_us=10398 scan_max_us=153 timer=on
2026-09-16 18:09:24.393 RX Response received in 4190 ms, Content-Length: 34123
2026-09-16 18:09:24.393 RX Starting to receive image data...
2026-09-16 18:09:24.712 RX Image download complete (34123 bytes, 4583 ms since button press). Starting decode...
2026-09-16 18:09:24.850 RX JPEG decoded successfully into PSRAM.
2026-09-16 18:09:24.851 RX LVGL image source updated. Total 4722 ms from button press (budget 20000 ms).
2026-09-16 18:09:27.417 RX Screen 2 Unloading: Freeing buffer and resetting rotation to 90 degrees.
2026-09-16 18:09:27.417 RX [ScreenMem] Returned to screen 1; no preference save needed
2026-09-16 18:09:31.537 TX log status [CRLF]
2026-09-16 18:09:31.542 RX [LOG] state=disabled boot=31 session=boot-31 up_ms=87041 clock=synced setup=1 hooks=1 file_bytes=0 generation=0 newest=0 archives=0 card_bytes=0 free_bytes=0 queue=0/16 high=0 drops=0 suppressed=0 truncated=0 error=mount_failed_or_no_card errno=5
2026-09-16 18:09:31.543 RX [LOG] measured=1 stack_min=4116 internal_min=166252 internal_largest=102388 dma_min=158756 dma_largest=102388 writes=0 slow=0 write_max_us=0 flush_max_us=0 sd_max_us=29369 rotations=0 pruned=0 oversized=0
2026-09-16 18:09:31.544 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=parked stack_used_max=4076 stack_final_margin=4116
2026-09-16 18:09:31.544 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-16 18:09:31.545 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-16 18:09:31.545 RX [LOG MEM] phase=before_clock up_us=1785268 free=172680 largest=110580 heap_min_boot=172680
2026-09-16 18:09:31.545 RX [LOG MEM] phase=after_clock up_us=1785958 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 18:09:31.545 RX [LOG MEM] phase=before_writer up_us=1786014 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 18:09:31.546 RX [LOG MEM] phase=writer_entry up_us=1786223 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 18:09:31.546 RX [LOG MEM] phase=after_formatter up_us=1786308 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 18:09:31.547 RX [LOG MEM] phase=before_mount up_us=1786865 free=167100 largest=102388 heap_min_boot=166992
2026-09-16 18:09:31.547 RX [LOG MEM] phase=after_mount up_us=1816586 free=167072 largest=102388 heap_min_boot=166252
2026-09-16 18:09:31.547 RX [LOG MEM] phase=before_current_open captured=0
2026-09-16 18:09:31.547 RX [LOG MEM] phase=after_current_open captured=0
2026-09-16 18:09:31.547 RX [LOG MEM] phase=storage_done up_us=1817024 free=167072 largest=102388 heap_min_boot=166252
```


## 2026-09-16 surviving-empty-file fixture prepared on confirmed SD card

JP confirmed E:\sdcard is the actual card. Windows reports it as a volume
mount-point junction. Its /logs contained the same eight files as the evening
copy; all eight matched the verified project backups byte for byte before mutation.

Preserved the 92055-byte current.log as the unused archive-00000010.log.
Its SHA-256 remains 48bf15790e1c50656fac1985763a67283b60f005d99e848665a4f0679720fa6c.
Created current.log with exclusive creation, flushed it with os.fsync and closed
the handle. Rechecked its size as zero and reverified archive 10 against the
original bytes. No log content was discarded or overwritten; copied evidence
was left unchanged. Fixture metadata is in
[sd_empty_fixture_2026-09-16.json](bench_data/sd_empty_fixture_2026-09-16.json).

Next JP safely ejects the card in Windows, powers the board fully off before
insertion, then boots with USB and hotspot on. Capture log status, wait 65 seconds
and capture it again. Expect ready, generation 11, newest archive 10, eight
archives, positive current size and subsequent growth, no unexpected errors or
drops. Installed test firmware is unchanged; no compile/flash is needed.
Do not send header, panic or rotation hooks for this fixture.

The next combined card inspection must confirm FILE_OPEN reason=empty_recovery.
A ready logger alone would also be compatible with missing-file recovery, so
keep that content check pending until actual header inspection.
Stage 1 remains unaccepted. No firmware changes, build, flash, commit or push.


## 2026-09-16 18:19-18:20 prepared zero-byte fixture: runtime recovery passed

JP supplied boot-32 statuses after booting the verified, closed zero-byte fixture.

| Measurement | 18:19:20 status | 18:20:01 status |
|---|---:|---:|
| Uptime, ms | 31929 | 72978 |
| Current bytes | 1257 | 1814 |
| Writes | 8 | 9 |
| Generation / newest archive / archive count | 11 / 10 / 8 | 11 / 10 / 8 |
| Internal minimum free / largest block, bytes | 84768 / 31732 | 84768 / 31732 |
| Writer used / margin, bytes | 4604 / 3588 | 4604 / 3588 |

Both statuses show ready, synced, hooks=1, valid active PSRAM writer placement,
empty queue, high-water=1, zero errors/drops/suppression/truncation/slow writes.
The file gained 557 bytes and one write. The interval was about 41 seconds,
not 65, but crossed the first minute health record, so no repeat is needed.
The normal probe covered 60003 ms and 6000 samples; largest_min=31732,
gap_max_us=10825, scan_max_us=970, IMU average=49.22 Hz and minimum=32.22 Hz.
No separate visual assessment was supplied with these statuses.

The prepared-file runtime check passes with the expected generation and archives.
The final FILE_OPEN reason=empty_recovery content check remains pending; do not
substitute status counters for this distinction after the previous hook's reason=new.

Next single test: natural size-triggered rotation using small limits, without
another card removal. Take an initial log status. Only proceed if generation=11
and file_bytes is below 7000; otherwise send that reading first so the fixture's
unsaved header is not accidentally pruned. Send log test small and log status.
Keep the board on USB, hotspot on, and idle for 15 minutes so minute HEALTH
records fill the 8192-byte current file. Then send log test normal and log status.
No forced rotate, NVS stress or other hooks during this interval.
The small limit retains three archives and can prune the older, already backed-up
logs. Generation 11 should remain as archive 11 after the first natural rotation
and its empty_recovery header can be inspected together with the new reason=size
header later. Source processTest can force reason=test if the file is already at
the new limit; the initial size guard avoids this for the requested run.

Require a generation increase, rotation counter increase, ready state and no
unexpected errors/drops. Confirm reason=size from SD at the next combined copy.
Restore normal limits even if no rotation is observed; report rather than adding
another stress hook. No rebuild, card mutation or firmware change performed here.
Stage 1 acceptance remains pending. No commit or push.


## 2026-09-17 08:05-08:22 small-limit test: disabled on content budget, not hotspot

JP reported the hotspot was initially off and asked whether to repeat.
Do not repeat the same small-limit command. The starting file had grown overnight
to 469869 bytes in boot 32, uptime 49619908 ms (about 13 h 47 min), above the
procedure's below-7000-byte precondition. The logger was ready with no errors or
drops before the hook. This is useful overnight logging evidence, but no cross-day
performance comparison is made.

| Measurement | Before small hook | After failure / final |
|---|---:|---:|
| Current bytes | 469869 | 470005 |
| Generation | 11 | 11 |
| Archives | 8 | 0 |
| Writes | 835 | 836 |
| Rotations | 0 | 0 |
| Pruned | 0 | 8 |
| Logger state | ready | disabled |
| Error | none | reserve_exhausted, errno=28 |
| Writer lifecycle | active | parked |
| Stack used / margin | 4604 / 3588 | 4780 / 3412 |
| Internal minimum / largest | 84644 / 31732 | 84644 / 31732 |

log test small was queued at 08:05:59.241 and the logger reported failure at
08:05:59.445. It first wrote the 136-byte TEST_HOOK record, then selected an
8192-byte file limit and three archives. prune checks a total managed-content
budget of (3 + 1) * 8192 = 32768 bytes, including the current file and incoming
reserve. The 470005-byte current alone exceeds that budget. All eight older
archives were pruned, but enoughContent remained false, causing reserve_exhausted
before current was renamed. No rotation completed. The file remained at 470005
bytes through 08:22:10; writes stayed 836.

This was NOT physical card exhaustion: free_bytes=15922593792 after pruning.
The failure label covers the content-budget failure too. The hotspot does not
control this calculation. It was a test-limit transition applied to a large
existing file; it does not demonstrate failure of normal size-triggered rotation.
The hook currently lacks a preflight size guard; the prior manual guard was not
satisfied. A future reviewed hook guard could reject this transition before
pruning, but no firmware change is authorized or implemented in this result.

The pruned archives 3-9 have verified copies in the evening backup, and archive
10's bytes match its backed-up current.log (92055 bytes, hash recorded in the
fixture manifest). The current generation-11 file, including its still-unverified
empty_recovery header and overnight records, was retained per runtime status.
Back it up and inspect it before any further pruning experiment.

The writer parked with final margin=3412, zero drops/suppression/truncation.
The main application continued: MQTT connected at 08:06:35, its probe measured
2514 ms and largest_min=31732, and later minute probes and motion events appeared.
These are not a separate owner visual confirmation. The disabled logger's
measurement snapshot no longer tracks subsequent heap changes.
log test normal at 08:21:56 was rejected as unavailable or busy because commands
require logger Ready. Thus it did NOT restore limits or restart logging.

Outcome: planned natural-size rotation did not run; retain this failure record.
Immediate next step: normal power-off/on with card installed, hotspot and USB on
for startup, then log status only. Runtime test limits reset to normal on reboot;
expect ready, generation 11 retained, zero archives if none were created elsewhere,
and file growth from the retained 470005 bytes. Do not send small or rotate.
If mount/recovery fails, stop and inspect the card. Once recovery is verified,
preserve the current file before preparing a fresh small-file rotation test.
Stage 1 acceptance remains pending. No firmware change, build, flash, commit or push.

### Raw browser capture

```text
2026-09-17 08:05:44.009 EVENT Console cleared.
2026-09-17 08:05:47.417 TX log status [CRLF]
2026-09-17 08:05:47.418 RX [LOG] state=ready boot=32 session=boot-32 up_ms=49619908 clock=synced setup=1 hooks=1 file_bytes=469869 generation=11 newest=10 archives=8 card_bytes=15931539456 free_bytes=15922266112 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-17 08:05:47.420 RX [LOG] measured=1 stack_min=3588 internal_min=84644 internal_largest=31732 dma_min=77148 dma_largest=31732 writes=835 slow=0 write_max_us=6715 flush_max_us=11297 sd_max_us=162584 rotations=0 pruned=0 oversized=0
2026-09-17 08:05:47.420 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4604 stack_final_margin=-1
2026-09-17 08:05:47.421 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-17 08:05:47.421 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-17 08:05:47.423 RX [LOG MEM] phase=before_clock up_us=1785268 free=172680 largest=110580 heap_min_boot=172680
2026-09-17 08:05:47.423 RX [LOG MEM] phase=after_clock up_us=1785958 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:05:47.423 RX [LOG MEM] phase=before_writer up_us=1786014 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:05:47.423 RX [LOG MEM] phase=writer_entry up_us=1786223 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:05:47.423 RX [LOG MEM] phase=after_formatter up_us=1786308 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:05:47.423 RX [LOG MEM] phase=before_mount up_us=1786863 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:05:47.423 RX [LOG MEM] phase=after_mount up_us=1949328 free=129836 largest=65524 heap_min_boot=128704
2026-09-17 08:05:47.423 RX [LOG MEM] phase=before_current_open up_us=1967583 free=129836 largest=65524 heap_min_boot=128704
2026-09-17 08:05:47.423 RX [LOG MEM] phase=after_current_open up_us=1969460 free=129836 largest=65524 heap_min_boot=128704
2026-09-17 08:05:47.423 RX [LOG MEM] phase=storage_done up_us=2017802 free=129836 largest=65524 heap_min_boot=128704
2026-09-17 08:05:53.114 RX [PROBE] window=normal run=828 ms=60001 heap_min_boot=84644 largest_min=40948 interval_ms=10 samples=6000 gap_max_us=11361 scan_max_us=1442 timer=on imu_n=2922 imu_min_hz=34.41 imu_avg_hz=48.86
2026-09-17 08:05:59.238 TX log test small [CRLF]
2026-09-17 08:05:59.241 RX [LOG TEST] hook queued
2026-09-17 08:05:59.445 RX [LOG] disabled reason=reserve_exhausted errno=28; companion continues
2026-09-17 08:06:12.234 TX log status [CRLF]
2026-09-17 08:06:12.242 RX [LOG] state=disabled boot=32 session=boot-32 up_ms=49644734 clock=synced setup=1 hooks=1 file_bytes=470005 generation=11 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922593792 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=reserve_exhausted errno=28
2026-09-17 08:06:12.243 RX [LOG] measured=1 stack_min=3412 internal_min=84644 internal_largest=31732 dma_min=77148 dma_largest=31732 writes=836 slow=0 write_max_us=6715 flush_max_us=11297 sd_max_us=162584 rotations=0 pruned=8 oversized=0
2026-09-17 08:06:12.244 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=parked stack_used_max=4780 stack_final_margin=3412
2026-09-17 08:06:12.246 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-17 08:06:12.246 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-17 08:06:12.247 RX [LOG MEM] phase=before_clock up_us=1785268 free=172680 largest=110580 heap_min_boot=172680
2026-09-17 08:06:12.247 RX [LOG MEM] phase=after_clock up_us=1785958 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:06:12.247 RX [LOG MEM] phase=before_writer up_us=1786014 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:06:12.247 RX [LOG MEM] phase=writer_entry up_us=1786223 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:06:12.247 RX [LOG MEM] phase=after_formatter up_us=1786308 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:06:12.247 RX [LOG MEM] phase=before_mount up_us=1786863 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:06:12.247 RX [LOG MEM] phase=after_mount up_us=1949328 free=129836 largest=65524 heap_min_boot=128704
2026-09-17 08:06:12.247 RX [LOG MEM] phase=before_current_open up_us=1967583 free=129836 largest=65524 heap_min_boot=128704
2026-09-17 08:06:12.247 RX [LOG MEM] phase=after_current_open up_us=1969460 free=129836 largest=65524 heap_min_boot=128704
2026-09-17 08:06:12.247 RX [LOG MEM] phase=storage_done up_us=2017802 free=129836 largest=65524 heap_min_boot=128704
2026-09-17 08:06:32.816 RX [PROBE] window=normal run=829 ms=39702 heap_min_boot=84644 largest_min=40948 interval_ms=10 samples=3971 gap_max_us=12551 scan_max_us=1127 timer=on imu_n=1930 imu_min_hz=35.26 imu_avg_hz=48.89
2026-09-17 08:06:35.331 RX [PROBE] window=mqtt_connect run=4 ms=2514 heap_min_boot=84644 largest_min=31732 interval_ms=10 samples=251 gap_max_us=10058 scan_max_us=121 timer=on
2026-09-17 08:06:35.337 RX [MQTT] Calibration sent: {"car_unit":true,"new_calib":false,"scale":1.0030,"gravity":[0.0500,0.0780,-0.9987],"rotation":[[0.1812,-0.9811,-0.0676],[-0.9822,-0.1771,-0.0630],[0.0498,0.0778,-0.9957]]}
2026-09-17 08:06:35.343 RX [NET] WiFi=CONNECTED | MQTT=REMOTE CONNECTED
2026-09-17 08:07:35.344 RX [PROBE] window=normal run=830 ms=60006 heap_min_boot=84644 largest_min=31732 interval_ms=10 samples=6001 gap_max_us=10805 scan_max_us=906 timer=on imu_n=2949 imu_min_hz=27.04 imu_avg_hz=49.24
2026-09-17 08:09:35.341 RX [PROBE] window=normal run=831 ms=60001 heap_min_boot=84644 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10861 scan_max_us=429 timer=on imu_n=2948 imu_min_hz=30.30 imu_avg_hz=49.24
2026-09-17 08:09:35.341 RX [PROBE] window=normal run=832 ms=60000 heap_min_boot=84644 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10607 scan_max_us=720 timer=on imu_n=2949 imu_min_hz=28.53 imu_avg_hz=49.24
2026-09-17 08:10:35.344 RX [PROBE] window=normal run=833 ms=60005 heap_min_boot=84644 largest_min=31732 interval_ms=10 samples=6001 gap_max_us=10843 scan_max_us=486 timer=on imu_n=2945 imu_min_hz=30.29 imu_avg_hz=49.19
2026-09-17 08:11:35.344 RX [PROBE] window=normal run=834 ms=60001 heap_min_boot=84644 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10702 scan_max_us=526 timer=on imu_n=2950 imu_min_hz=36.88 imu_avg_hz=49.25
2026-09-17 08:13:35.349 RX [PROBE] window=normal run=835 ms=60003 heap_min_boot=84644 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10583 scan_max_us=432 timer=on imu_n=2949 imu_min_hz=30.28 imu_avg_hz=49.26
2026-09-17 08:13:35.349 RX [PROBE] window=normal run=836 ms=60006 heap_min_boot=84644 largest_min=31732 interval_ms=10 samples=6001 gap_max_us=10987 scan_max_us=1149 timer=on imu_n=2949 imu_min_hz=31.22 imu_avg_hz=49.23
2026-09-17 08:14:35.349 RX [PROBE] window=normal run=837 ms=60002 heap_min_boot=84644 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10805 scan_max_us=742 timer=on imu_n=2947 imu_min_hz=37.01 imu_avg_hz=49.22
2026-09-17 08:15:35.350 RX [PROBE] window=normal run=838 ms=60002 heap_min_boot=84644 largest_min=31732 interval_ms=10 samples=6001 gap_max_us=10835 scan_max_us=633 timer=on imu_n=2947 imu_min_hz=31.25 imu_avg_hz=49.21
2026-09-17 08:17:35.350 RX [PROBE] window=normal run=839 ms=60004 heap_min_boot=84644 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10511 scan_max_us=428 timer=on imu_n=2951 imu_min_hz=27.79 imu_avg_hz=49.28
2026-09-17 08:17:35.351 RX [PROBE] window=normal run=840 ms=60001 heap_min_boot=84644 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10826 scan_max_us=424 timer=on imu_n=2950 imu_min_hz=36.99 imu_avg_hz=49.26
2026-09-17 08:18:04.327 RX Movement Detected! (Accel: 0.00, Gyro: 20.20)
2026-09-17 08:18:04.328 RX TX motion MQTT: Moving (immediate)
2026-09-17 08:18:34.335 RX TX motion MQTT: Moving (periodic)
2026-09-17 08:18:34.349 RX Movement Stopped.
2026-09-17 08:19:35.351 RX [PROBE] window=normal run=841 ms=60003 heap_min_boot=84644 largest_min=31732 interval_ms=10 samples=6001 gap_max_us=10862 scan_max_us=432 timer=on imu_n=2951 imu_min_hz=29.41 imu_avg_hz=49.28
2026-09-17 08:19:35.351 RX [PROBE] window=normal run=842 ms=60001 heap_min_boot=84644 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10803 scan_max_us=904 timer=on imu_n=2949 imu_min_hz=34.44 imu_avg_hz=49.25
2026-09-17 08:20:35.351 RX [PROBE] window=normal run=843 ms=60001 heap_min_boot=84644 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10872 scan_max_us=864 timer=on imu_n=2954 imu_min_hz=38.28 imu_avg_hz=49.30
2026-09-17 08:21:35.353 RX [PROBE] window=normal run=844 ms=60004 heap_min_boot=84644 largest_min=31732 interval_ms=10 samples=6001 gap_max_us=10901 scan_max_us=792 timer=on imu_n=2949 imu_min_hz=26.94 imu_avg_hz=49.25
2026-09-17 08:21:56.078 TX log test normal [CRLF]
2026-09-17 08:21:56.085 RX [LOG TEST] unavailable or busy
2026-09-17 08:22:10.666 TX log status [CRLF]
2026-09-17 08:22:10.669 RX [LOG] state=disabled boot=32 session=boot-32 up_ms=50603204 clock=synced setup=1 hooks=1 file_bytes=470005 generation=11 newest=0 archives=0 card_bytes=15931539456 free_bytes=15922593792 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=reserve_exhausted errno=28
2026-09-17 08:22:10.669 RX [LOG] measured=1 stack_min=3412 internal_min=84644 internal_largest=31732 dma_min=77148 dma_largest=31732 writes=836 slow=0 write_max_us=6715 flush_max_us=11297 sd_max_us=162584 rotations=0 pruned=8 oversized=0
2026-09-17 08:22:10.670 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=parked stack_used_max=4780 stack_final_margin=3412
2026-09-17 08:22:10.671 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-17 08:22:10.671 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-17 08:22:10.671 RX [LOG MEM] phase=before_clock up_us=1785268 free=172680 largest=110580 heap_min_boot=172680
2026-09-17 08:22:10.671 RX [LOG MEM] phase=after_clock up_us=1785958 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:22:10.671 RX [LOG MEM] phase=before_writer up_us=1786014 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:22:10.671 RX [LOG MEM] phase=writer_entry up_us=1786223 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:22:10.672 RX [LOG MEM] phase=after_formatter up_us=1786308 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:22:10.672 RX [LOG MEM] phase=before_mount up_us=1786863 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:22:10.672 RX [LOG MEM] phase=after_mount up_us=1949328 free=129836 largest=65524 heap_min_boot=128704
2026-09-17 08:22:10.672 RX [LOG MEM] phase=before_current_open up_us=1967583 free=129836 largest=65524 heap_min_boot=128704
2026-09-17 08:22:10.672 RX [LOG MEM] phase=after_current_open up_us=1969460 free=129836 largest=65524 heap_min_boot=128704
2026-09-17 08:22:10.672 RX [LOG MEM] phase=storage_done up_us=2017802 free=129836 largest=65524 heap_min_boot=128704
```


## 2026-09-17 08:26 reboot after small-limit failure: logging recovered

JP supplied the first status after normal power-off/on with the same card.

| Check | Result |
|---|---|
| Boot / uptime | 33 / 17526 ms |
| Logger | ready, error=none, errno=0 |
| Current | generation 11, 470855 bytes |
| Archives | newest=0, count=0, consistent with preceding pruning |
| Writes / rotations / pruned | 5 / 0 / 0 (new-boot counters) |
| Queue | 0/16, high=1, drops=0, suppressed=0, truncated=0 |
| Memory | internal_min=84752, internal_largest=31732; DMA min/largest=77256/31732 |
| Writer | valid active PSRAM stack, used=4204, margin=3988 |
| Clock | unknown at this early status; MQTT already connected |
| Timings | write_max_us=2766, flush_max_us=2500, sd_max_us=168771, slow=0 |

The current file grew from 470005 before reboot to 470855 bytes: 850 bytes of
new writes while retaining generation 11. Runtime recovery passes and confirms
logging resumed with normal startup configuration after the temporary limit fault.
This is not a new rotation pass or proof of later minute-record growth.
The lower stack used/high-water reflects a new boot that has not exercised prior
peak paths, not a firmware memory optimization.

Clock unknown at about 17.5 seconds is an early, pre-confirmed-SNTP observation.
MQTT connectivity does not itself establish time synchronization. Do not declare
clock failure from this single early status; the next capture/card records can
show whether sync completes. No owner visual assessment accompanied this status.

Next is one combined card-reader visit: normal shutdown, inspect and back up the
preserved generation-11 current (including the empty_recovery header, overnight
records and failed small-limit hook), then prepare a fresh small file for the
next rotation attempt. JP should put the card in the reader only after the board
is fully off and confirm it is accessible at E:\sdcard. Do not access or mutate
a stale mount path before that confirmation. Do not send small, rotate or other
fault hooks meanwhile. Once the current file is backed up and its header verified,
preserve it under an unused archive name and prepare the fresh file without
discarding data. Future pruning may remove that on-card archive after backup.

No firmware changes, build, flash, commit or push. No card mutation performed
in this turn. Stage 1 acceptance remains pending.


## 2026-09-17 08:28 card inspection: empty recovery verified; rotation retry prepared

JP confirmed the card back at E:\sdcard. The actual mounted /logs contained only
current.log, 472389 bytes. Its byte-identical backup and hash are in
[sd_logs_2026-09-17_0828](bench_data/sd_logs_2026-09-17_0828/README.md).
SHA-256: 0eb39ffc9594cdfcea686ae61ce79a9ffcdf7e2fcaf2577f3d4ab5f01a2b5efe.

The first record is FILE_OPEN generation=11 reason=empty_recovery in boot 32.
This closes the surviving-zero-byte-file content check; the earlier hook-only
reason=new result remains a distinct missing-file recovery test.
All retained records have valid common prefixes. Boot 32 sequences 1-836 and
boot 33 sequences 1-10 are contiguous. The final boot-32 record is the
deliberate TEST_HOOK at 08:05:59.227; no SESSION_END is expected after its
logger was disabled. The same file then appends boot 33 with context=append.

Boot 33's CLOCK_SYNC occurred at uptime 19372 ms, local 08:26:36.680, followed
by CLOCK_OFFSET -0400. Thus the early clock=unknown status was transient.
Its next two HEALTH records continued normally, and the final record is
SESSION_END at 08:28:58.218 -04:00, reason=shutdown, pending=0.
Reboot append, time synchronization and clean shutdown after recovery are
confirmed on disk. These results do not mark the natural-size rotation gate passed.

After backup and hash verification, preserved the entire original current.log
as the unused archive-00000011.log on the card. Created a fresh current.log
with exclusive creation, flushed with os.fsync, closed and rechecked its zero
size. Reverified archive 11 against the original backup bytes. No content was
discarded or overwritten. Fixture metadata is in the backup folder's fixture.json.
Expected next startup: generation 12, newest archive 11, one archive.

Next JP safely ejects the card, inserts only into the fully powered-off board,
boots with USB/hotspot on, and sends log status promptly. Paste that first status
before ANY small/rotate command. This splits the starting-size check from mutation
to avoid repeating the overnight large-file transition. No 15-minute wait yet.
After reviewing that starting status, give the small-mode step and confirm
its immediate status before the idle-fill interval. Small mode will probably
prune archive 11 because it exceeds the tiny content budget; its complete bytes
are now safely backed up and its header is verified. Do not count such pruning
as loss of unpreserved evidence.

No firmware change, build, flash, commit or push. Card fixture preparation and
documentation only. Stage 1 acceptance remains pending.


## 2026-09-17 08:33 rotation retry: starting state verified

JP's status at 08:33:35.530 shows boot 34, uptime 20553 ms, ready and synced.
Current is 1276 bytes, generation 12; newest archive 11, count 1, as prepared.
Writes=8, rotations=0, pruned=0, queue=0/16, high=1, drops=0, error=none,
errno=0, suppressed=0, truncated=0, slow=0. Internal minimum=84748,
largest=31732, DMA minimum/largest=77252/31732. PSRAM placement is valid,
writer active, maximum used=4604 and margin=3588.
No visual assessment accompanied this status.

This starting file meets the below-7000-byte guard. Next send log test small,
wait two seconds, send log status, and paste the immediate result before starting
the 15-minute idle-fill period. Require ready with no unexpected errors/drops.
Archive 11 may be pruned on a later storage-space check; its verified backup is
preserved. An immediate status can still show that archive until pruning occurs.
Do not force a rotation. If this instruction is resumed much later, obtain a
fresh status before enabling small mode; this is not approval to shrink a file
that has grown beyond the guard.

No firmware changes, build, flash, card mutation, commit or push.
Natural-size rotation and Stage 1 acceptance remain pending.


## 2026-09-17 08:35 small-limit immediate checkpoint: healthy

JP sent log test small at 08:34:58.536; the hook was queued at 08:34:58.539.
At 08:35:08.108, boot 34 remained ready and synced, uptime 113135 ms.
Current=1966 bytes, generation=12, newest=11, archives=1, writes=10,
rotations=0 and pruned=0. No errors/drops/suppression/truncation/slow writes.
Queue high=1, internal minimum=84748, largest=31732, valid active PSRAM writer,
used=4604 and margin=3588. The earlier normal probe reports IMU average
49.18 Hz and minimum 35.23 Hz over 2945 samples.

The hook was accepted without the prior failure. Limits are not printed in
status; their values follow the inspected hook implementation, not independent
numeric readback. Archive 11 has not yet been pruned in this snapshot, consistent
with the next space check occurring later. No natural rotation has occurred yet.

Next keep USB and hotspot on, board idle for 15 minutes from this checkpoint
(about 08:50). Send log status BEFORE restoring limits, then log test normal,
wait two seconds and send log status again. Paste the capture including both
statuses. Do not send forced rotate, stress, reboot or another fault hook.
The pre-restore reading distinguishes natural rotation from the restore command.
Expect generation 13 or later and a positive rotation count, ready state and no
unexpected errors/drops. Older archive pruning is expected and its backup exists.
Even if rotation has not occurred, restore normal limits and report rather than
extending the run or injecting another hook. On-card reason=size remains a later
content check. No firmware change, build, flash, commit or push.


## 2026-09-17 08:49 natural size-rotation retry: runtime passed

JP supplied the idle-fill capture with status before restoring normal limits.

| Measurement | Small-mode checkpoint 08:35 | Before restore 08:49:25 | After restore 08:49:38 |
|---|---:|---:|---:|
| Boot | 34 | 34 | 34 |
| Current generation | 12 | 13 | 13 |
| Current bytes | 1966 | 2696 | 2829 |
| Writes | 10 | 27 | 28 |
| Rotations | 0 | 1 | 1 |
| Pruned | 0 | 1 | 1 |
| Newest archive / count | 11 / 1 | 12 / 1 | 12 / 1 |
| Internal largest, bytes | 31732 | 31732 | 31732 |
| Writer used / margin, bytes | 4604 / 3588 | 4812 / 3380 | 4812 / 3380 |

The rotation already existed before log test normal, with no forced-rotation
command in the supplied run. Generation increased to 13 and rotation count to 1
under the normal minute-record workload. The backed-up old archive 11 was pruned;
the surviving newest archive is 12. The interval from the 08:35 status to the
pre-restore status is 857652 ms (about 14 min 18 s), enough to show completion;
there is no need to repeat for exactly 15 minutes.

log test normal was accepted at 08:49:30.677. The next status remained ready and
synced, adding one write and 133 bytes, with rotations unchanged. Source handling
restores ordinary limits; status does not expose the numeric settings.
No errors, drops, suppression, truncation or slow writes, queue high=1.
Internal minimum remained 84748; DMA minimum/largest 77252/31732.
PSRAM placement remained valid and the writer active. Stack high-water use grew
by 208 bytes for this path, leaving 3380 bytes; this is the newly exercised peak,
not persistent per-rotation allocation. Final parked margin is not measured while
the writer remains active.

Thirteen supplied normal windows (runs 4-16) report IMU averages 49.22-49.25 Hz,
minimum 26.33 Hz across their instantaneous minima, largest_min=31732 throughout.
Maximum sampler gap among these windows is 11010 us. Some browser receive
timestamps batch adjacent minute reports; the device windows remain about 60 s.
No separate visual assessment accompanied the capture.

Outcome: natural size-triggered rotation passes its runtime check. Confirm
FILE_OPEN reason=size, archive-12 size at or below 8192, record continuity across
archive 12 and current generation 13, and normal shutdown in the next card copy.
No additional idle-fill repetition is needed.

Next combine that content inspection and preparation of the remaining incomplete-
tail test in one reader visit: normal shutdown, card into reader, then confirm
E:\sdcard is accessible. Back up and verify the originals before any fixture
changes. Do not alter files or run other hooks on the board meanwhile.
An incomplete-tail fixture should append a clearly labeled deliberate partial
record without a newline to a backed-up valid current file, leaving the valid
header intact. Do not prepare it until card access is confirmed.
This is a different recovery branch from the already verified empty/partial header
tests. Natural rotation content verification and other remaining Stage 1 gates
still apply; Stage 1 is not accepted yet.
No firmware changes, build, flash, commit or push.

### Raw browser capture

```text
2026-09-17 08:36:45.734 EVENT Console cleared.
2026-09-17 08:38:23.990 RX [PROBE] window=normal run=4 ms=60002 heap_min_boot=84748 largest_min=31732 interval_ms=10 samples=6001 gap_max_us=10839 scan_max_us=550 timer=on imu_n=2947 imu_min_hz=26.33 imu_avg_hz=49.22
2026-09-17 08:38:23.990 RX [PROBE] window=normal run=5 ms=60006 heap_min_boot=84748 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10634 scan_max_us=497 timer=on imu_n=2947 imu_min_hz=29.37 imu_avg_hz=49.22
2026-09-17 08:39:23.992 RX [PROBE] window=normal run=6 ms=60003 heap_min_boot=84748 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10899 scan_max_us=1056 timer=on imu_n=2949 imu_min_hz=27.05 imu_avg_hz=49.25
2026-09-17 08:40:23.992 RX [PROBE] window=normal run=7 ms=60002 heap_min_boot=84748 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10636 scan_max_us=438 timer=on imu_n=2948 imu_min_hz=27.04 imu_avg_hz=49.23
2026-09-17 08:41:23.997 RX [PROBE] window=normal run=8 ms=60006 heap_min_boot=84748 largest_min=31732 interval_ms=10 samples=6001 gap_max_us=11010 scan_max_us=1148 timer=on imu_n=2947 imu_min_hz=31.21 imu_avg_hz=49.22
2026-09-17 08:42:24.000 RX [PROBE] window=normal run=9 ms=60005 heap_min_boot=84748 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10856 scan_max_us=1016 timer=on imu_n=2949 imu_min_hz=29.41 imu_avg_hz=49.24
2026-09-17 08:43:23.999 RX [PROBE] window=normal run=10 ms=60000 heap_min_boot=84748 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10807 scan_max_us=699 timer=on imu_n=2948 imu_min_hz=33.29 imu_avg_hz=49.23
2026-09-17 08:44:23.999 RX [PROBE] window=normal run=11 ms=60001 heap_min_boot=84748 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10656 scan_max_us=436 timer=on imu_n=2947 imu_min_hz=35.72 imu_avg_hz=49.23
2026-09-17 08:45:23.998 RX [PROBE] window=normal run=12 ms=60000 heap_min_boot=84748 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10858 scan_max_us=714 timer=on imu_n=2947 imu_min_hz=33.13 imu_avg_hz=49.22
2026-09-17 08:46:24.002 RX [PROBE] window=normal run=13 ms=60005 heap_min_boot=84748 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10989 scan_max_us=1099 timer=on imu_n=2948 imu_min_hz=30.27 imu_avg_hz=49.23
2026-09-17 08:47:24.002 RX [PROBE] window=normal run=14 ms=60002 heap_min_boot=84748 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10912 scan_max_us=1065 timer=on imu_n=2948 imu_min_hz=30.32 imu_avg_hz=49.23
2026-09-17 08:48:24.002 RX [PROBE] window=normal run=15 ms=60001 heap_min_boot=84748 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10694 scan_max_us=447 timer=on imu_n=2949 imu_min_hz=35.31 imu_avg_hz=49.24
2026-09-17 08:49:24.002 RX [PROBE] window=normal run=16 ms=60001 heap_min_boot=84748 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10729 scan_max_us=524 timer=on imu_n=2947 imu_min_hz=32.26 imu_avg_hz=49.22
2026-09-17 08:49:25.723 TX log status [CRLF]
2026-09-17 08:49:25.726 RX [LOG] state=ready boot=34 session=boot-34 up_ms=970787 clock=synced setup=1 hooks=1 file_bytes=2696 generation=13 newest=12 archives=1 card_bytes=15931539456 free_bytes=15923019776 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-17 08:49:25.727 RX [LOG] measured=1 stack_min=3380 internal_min=84748 internal_largest=31732 dma_min=77252 dma_largest=31732 writes=27 slow=0 write_max_us=2572 flush_max_us=5605 sd_max_us=164827 rotations=1 pruned=1 oversized=0
2026-09-17 08:49:25.727 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4812 stack_final_margin=-1
2026-09-17 08:49:25.728 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-17 08:49:25.729 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-17 08:49:25.729 RX [LOG MEM] phase=before_clock up_us=1785269 free=172680 largest=110580 heap_min_boot=172680
2026-09-17 08:49:25.729 RX [LOG MEM] phase=after_clock up_us=1785959 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:49:25.729 RX [LOG MEM] phase=before_writer up_us=1786015 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:49:25.729 RX [LOG MEM] phase=writer_entry up_us=1786224 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:49:25.729 RX [LOG MEM] phase=after_formatter up_us=1786308 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:49:25.730 RX [LOG MEM] phase=before_mount up_us=1786867 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:49:25.730 RX [LOG MEM] phase=after_mount up_us=1951584 free=129836 largest=65524 heap_min_boot=128640
2026-09-17 08:49:25.730 RX [LOG MEM] phase=before_current_open up_us=1958501 free=129836 largest=65524 heap_min_boot=128640
2026-09-17 08:49:25.730 RX [LOG MEM] phase=after_current_open up_us=1960356 free=129836 largest=65524 heap_min_boot=128640
2026-09-17 08:49:25.730 RX [LOG MEM] phase=storage_done up_us=1983413 free=129836 largest=65524 heap_min_boot=128640
2026-09-17 08:49:30.674 TX log test normal [CRLF]
2026-09-17 08:49:30.677 RX [LOG TEST] hook queued
2026-09-17 08:49:38.051 TX log status [CRLF]
2026-09-17 08:49:38.052 RX [LOG] state=ready boot=34 session=boot-34 up_ms=983114 clock=synced setup=1 hooks=1 file_bytes=2829 generation=13 newest=12 archives=1 card_bytes=15931539456 free_bytes=15923019776 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-17 08:49:38.054 RX [LOG] measured=1 stack_min=3380 internal_min=84748 internal_largest=31732 dma_min=77252 dma_largest=31732 writes=28 slow=0 write_max_us=2572 flush_max_us=5605 sd_max_us=164827 rotations=1 pruned=1 oversized=0
2026-09-17 08:49:38.055 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4812 stack_final_margin=-1
2026-09-17 08:49:38.055 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-17 08:49:38.056 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-17 08:49:38.056 RX [LOG MEM] phase=before_clock up_us=1785269 free=172680 largest=110580 heap_min_boot=172680
2026-09-17 08:49:38.056 RX [LOG MEM] phase=after_clock up_us=1785959 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:49:38.056 RX [LOG MEM] phase=before_writer up_us=1786015 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:49:38.056 RX [LOG MEM] phase=writer_entry up_us=1786224 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:49:38.056 RX [LOG MEM] phase=after_formatter up_us=1786308 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:49:38.058 RX [LOG MEM] phase=before_mount up_us=1786867 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 08:49:38.058 RX [LOG MEM] phase=after_mount up_us=1951584 free=129836 largest=65524 heap_min_boot=128640
2026-09-17 08:49:38.058 RX [LOG MEM] phase=before_current_open up_us=1958501 free=129836 largest=65524 heap_min_boot=128640
2026-09-17 08:49:38.058 RX [LOG MEM] phase=after_current_open up_us=1960356 free=129836 largest=65524 heap_min_boot=128640
2026-09-17 08:49:38.058 RX [LOG MEM] phase=storage_done up_us=1983413 free=129836 largest=65524 heap_min_boot=128640
```


## 2026-09-17 08:52 card inspection: natural rotation verified; tail fixture prepared

JP confirmed the card available at E:\sdcard. It contained archive-00000012.log
(8136 bytes) and current.log (4087 bytes). Both originals were copied byte for
byte, with verified hashes, to
[sd_logs_2026-09-17_0852](bench_data/sd_logs_2026-09-17_0852/README.md).

Archive 12 contains boot-34 sequences 1-21; its header is generation 12
reason=empty_recovery. Current starts with sequence 22, FILE_OPEN generation=13
reason=size, then BOOT context=rotation at sequence 23. It continues through
sequence 31. All 31 sequences are contiguous across the two files.
Archive length 8136 is below the temporary 8192-byte limit. The next HEALTH
record did not fit, so headers were written to the new file before that record.
The first HEALTH in current retains its pre-rotation snapshot (size=8136,
generation=12); later records show generation 13. This is event snapshot timing,
not a failed generation change.
Final SESSION_END is 08:52:17.219 -04:00, shutdown, pending=0.
Natural-size rotation now passes its content check as well as the runtime check.

After backup verification, appended exactly 56 bytes to the card current.log:
DIAG_TEST_PARTIAL_RECORD source=card_fixture incomplete=
There is deliberately no terminating newline. Existing 4087 bytes and valid header
are unchanged, as is archive 12. The file was flushed with os.fsync, closed and
read back to verify exact equality with original + fragment. Result: 4143 bytes.
Original SHA-256: 819d2f01747f520ecc917ae6cd339a2975708785175e6a7ab451d4847dabf078.
Fixture SHA-256: 9d5fc29d263b5c148c3b918877f306a1a47d79d43487f6b53d797ea505e020d7.
Fixture metadata is in the backup folder's fixture.json.

Next JP safely ejects, inserts only with board fully off, boots with USB/hotspot,
and captures log status, then another after 65 seconds. Expect ready,
generation 13, newest archive 12, count 1, resumed file growth and no unexpected
errors/drops. No fault commands or rebuild. The boot should add a newline and
TAIL_RECOVERY action=added_newline before BOOT append. Exact record placement
and intact original bytes need verification in the next combined card inspection.
The deliberate malformed fragment is a controlled fixture, not an actual power
interruption and not a normal firmware record. Other incomplete-tail branches,
such as rotation at a near-full normal file, are not inferred from this fixture.
No firmware changes, build, flash, commit or push. Stage 1 remains pending.


## 2026-09-17 09:06-09:07 incomplete-tail fixture: runtime recovery passed

JP supplied boot-35 statuses after the 4143-byte incomplete-tail fixture.

| Measurement | First status | Second status |
|---|---:|---:|
| Uptime, ms | 20061 | 88343 |
| Current bytes | 5099 | 5939 |
| Writes | 7 | 10 |
| Clock | unknown | synced |
| Generation / newest / archives | 13 / 12 / 1 | 13 / 12 / 1 |
| Internal minimum / largest, bytes | 84788 / 31732 | 84788 / 31732 |
| Writer used / margin, bytes | 4604 / 3588 | 4604 / 3588 |

Both statuses show ready, no errors/drops/suppression/truncation, high-water=1,
valid active PSRAM placement and no slow writes. Current gained 840 bytes and
three writes over 68282 ms. The normal probe covered 60000 ms and 6000 samples,
largest_min=31732, gap_max_us=10914, scan_max_us=1029, IMU average=49.24 Hz,
minimum=36.99 Hz. The second pasted LOG MEM block ends mid-line, but the
complete primary and stack statuses already contain the required result;
no repeat is needed just for that truncation. No visual assessment was supplied.

Runtime tail recovery passes. Exact TAIL_RECOVERY action=added_newline,
separation of the deliberate fragment and preservation of the original prefix
remain pending one combined card inspection. Clock synchronization returned;
unknown at the first early status was temporary. No new runtime fault was shown.

JP asked how many tests remain. Consolidate the remaining requirements before
issuing more tests. Passed memory/performance, media/reconnect, NVS stress,
normal shutdown, forced/natural rotation, pruning outcomes, header salvage,
empty-file recovery, reset classes/breadcrumbs, DST and no-card cases need no
repeat absent new evidence. Do not claim full Stage 1 acceptance yet.

Core remaining work:
- Inspect the repaired tail once; group it with setup for unrelated-file and
  /images preservation during pruning rather than a separate reader visit.
- Exercise deep-sleep close and touch wake, including reset class, retained time
  and breadcrumb evidence. Normal shutdown is already verified.
- Check pruning with unrelated names/dummy images and rotation while clock quality
  is unknown (the plan explicitly includes no-clock rotation).
- Bad-card/unsupported-card cases need suitable disposable media or an explicit
  owner-approved deferral. No-card and injected full-write error are already tested.
  The content-budget reserve_exhausted failure is not literal card-full hardware.
Brownout retention is explicitly practical/conditional in the plan, not a reason
to perform an improvised destructive electrical test. Record whether deferred.
Near-cap incomplete-tail rotation is not covered by the short-tail fixture;
do not imply that every branch was tested.

Source review for a later deep-sleep procedure: inactivity timeout is 60000 ms,
not the stale 30-second comments. In normal power mode, USB present disables sleep.
After USB loss in a boot that saw USB, motion keeps the board awake and stationary
eventually triggers shutdown. To reach deep sleep with existing firmware, start
a fresh battery-only boot with USB never connected, maintain detected movement
without touching the screen until inactivity expires, then touch to wake.
Confirm exact bench instructions before running; no new firmware needed.
No new fault command, card mutation, build, flash, commit or push in this turn.


## 2026-09-17 accepted scope refinement and next deep-sleep test

JP explicitly chose to skip bad/unsupported-card tests to speed version 1 and
then agreed to proceed. Record those as deferred, not passed. No-card and injected
write-failure results remain valid but do not prove damaged-card handling.
Stage 1 acceptance is still separate and has not been granted.

Next run deep-sleep close/wake on the existing battery board without removing
the card or reflashing. Source confirms INACTIVITY_TIMEOUT=60000 and motion
hold=30000 ms. finalizeSetup resets lastActivityTime. In normal power mode the
deep-sleep route requires a fresh battery-only boot; unplugging USB in the current
boot instead selects the USB-loss shutdown/keep-awake route.

Procedure: with hotspot on and card installed, unplug USB and let the stationary
board shut down normally. Start it again using its power button on battery only.
Wait for normal connected operation, tap the dashboard once to set a clear
inactivity starting point, then hold the edges and gently tilt every 10-15 seconds
without touching the screen. Keep the movement icon active until about one
minute after that touch. Expect Sleeping... then display off. Stop moving,
wait five seconds, tap the screen once, and verify it wakes before reconnecting
USB. Once normal display returns, reconnect USB and use the browser to capture
log status, then another after 65 seconds. Report the observed sleep message,
touch-only wake, and any visible abnormality.

If no sleep occurs after about two minutes, or touch does not wake the board,
stop and report rather than silently converting the test to a power-button or
USB wake. USB connection is intentionally after the observed touch wake so its
power change cannot be mistaken for the successful wake trigger.

The SD log later must show SESSION_END reason=deep_sleep, pending=0, then
BOOT reset=deep_sleep, expected external-touch wake cause and retained breadcrumbs.
Approximate time followed by real sync is inspected on disk; serial may attach
too late to observe the intermediate quality. The runtime statuses must show
ready, continuing generation 13 with retained archive 12 (absent other changes),
growing file and no unexpected errors/drops. A physical observation alone does
not replace reset classification.

Group the later card inspection with the pending tail-content verification and
setup of unrelated-file/no-clock-rotation fixtures. Do not add repetitions of
passed tests. No firmware changes, build, flash, card edits, commit or push.


## 2026-09-17 09:25-09:27 sleep-test capture: logging healthy; wake observation pending

JP supplied the post-test browser capture, beginning with USB availability and
connection. It contains boot 37 statuses; the previous supplied test was boot 35.
Two intervening boot increments are consistent with the planned battery startup
and wake sequence, but boot counts alone cannot identify the reset types.

| Measurement | First status | Follow-up |
|---|---:|---:|
| Uptime, ms | 26327 | 91300 |
| Current bytes | 18705 | 19262 |
| Writes | 7 | 8 |
| Generation / newest / archive count | 13 / 12 / 1 | 13 / 12 / 1 |
| Internal minimum / largest, bytes | 84760 / 31732 | 84760 / 31732 |
| Writer used / margin, bytes | 4604 / 3588 | 4604 / 3588 |

Both statuses show ready and synced, hooks=1, valid active PSRAM placement,
zero errors/drops/suppression/truncation/slow writes, queue high-water=1.
File growth is 557 bytes with one additional write over 64973 ms.
Normal probe: 60000 ms, 6000 samples, largest_min=31732, gap_max_us=10970,
scan_max_us=1072, IMU average=49.19 Hz and minimum=25.62 Hz.
Motion and USB-power prints also show the application responding after connection.

Runtime logging after the test is healthy. The capture does not show entry into
deep sleep or the actual touch wake because USB connected afterward. JP was asked
whether Sleeping... appeared and touch alone woke the board before USB, without
the power button. That observation is pending; do not infer it from normal status.
SD SESSION_END reason=deep_sleep, BOOT reset/wake cause, retained time and prior
breadcrumbs also await inspection. Do not request a test repeat unless the
observation or SD evidence reveals an unmet condition.

After confirmation, group the card inspection of tail recovery and deep sleep
with setup of unrelated-file/no-clock-rotation fixtures. Bad/unsupported cards
remain deferred by JP for version 1. No firmware changes, build, flash, card
mutation, commit or push. Stage 1 acceptance remains pending.

### Raw browser capture

```text
2026-09-17 09:23:42.363 EVENT Console cleared.
2026-09-17 09:25:46.015 EVENT USB serial device available.
2026-09-17 09:25:50.389 EVENT Connect requested. After open: DTR=true, RTS=false.
2026-09-17 09:25:52.999 EVENT Explicit signals applied. After open: DTR=true, RTS=false.
2026-09-17 09:25:53.000 EVENT Connected. Send status; compare with the previous reading if available.
2026-09-17 09:25:58.687 TX log status [CRLF]
2026-09-17 09:25:58.690 RX  complete: CPU 240 MHz | heap 90320 | PSRAM 8336052 ---
2026-09-17 09:25:58.691 RX
2026-09-17 09:25:58.691 RX [TEST] Serial bench commands: off, on, status, log status. Send with CR or LF.
2026-09-17 09:25:58.691 RX Movement Detected! (Accel: 0.00, Gyro: 6.69)
2026-09-17 09:25:58.691 RX TX motion MQTT: Moving (immediate)
2026-09-17 09:25:58.691 RX USB Power Connected - Sleep disabled
2026-09-17 09:25:58.692 RX [LOG] state=ready boot=37 session=boot-37 up_ms=26327 clock=synced setup=1 hooks=1 file_bytes=18705 generation=13 newest=12 archives=1 card_bytes=15931539456 free_bytes=15923019776 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-17 09:25:58.693 RX [LOG] measured=1 stack_min=3588 internal_min=84760 internal_largest=31732 dma_min=77264 dma_largest=31732 writes=7 slow=0 write_max_us=1863 flush_max_us=5146 sd_max_us=83851 rotations=0 pruned=0 oversized=0
2026-09-17 09:25:58.694 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4604 stack_final_margin=-1
2026-09-17 09:25:58.694 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-17 09:25:58.695 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-17 09:25:58.695 RX [LOG MEM] phase=before_clock up_us=1796259 free=172680 largest=110580 heap_min_boot=172680
2026-09-17 09:25:58.695 RX [LOG MEM] phase=after_clock up_us=1796948 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 09:25:58.695 RX [LOG MEM] phase=before_writer up_us=1797004 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 09:25:58.695 RX [LOG MEM] phase=writer_entry up_us=1797212 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 09:25:58.695 RX [LOG MEM] phase=after_formatter up_us=1797297 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 09:25:58.696 RX [LOG MEM] phase=before_mount up_us=1797910 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 09:25:58.696 RX [LOG MEM] phase=after_mount up_us=1881599 free=129836 largest=65524 heap_min_boot=128624
2026-09-17 09:25:58.696 RX [LOG MEM] phase=before_current_open up_us=1890828 free=129836 largest=65524 heap_min_boot=128624
2026-09-17 09:25:58.696 RX [LOG MEM] phase=after_current_open up_us=1892238 free=129836 largest=65524 heap_min_boot=128624
2026-09-17 09:25:58.696 RX [LOG MEM] phase=storage_done up_us=1907930 free=129836 largest=65524 heap_min_boot=128624
2026-09-17 09:26:11.601 RX TX motion MQTT: Moving (periodic)
2026-09-17 09:26:17.218 RX Movement Stopped.
2026-09-17 09:26:40.905 RX [PROBE] window=normal run=1 ms=60000 heap_min_boot=84760 largest_min=31732 interval_ms=10 samples=6000 gap_max_us=10970 scan_max_us=1072 timer=on imu_n=2945 imu_min_hz=25.62 imu_avg_hz=49.19
2026-09-17 09:27:03.656 TX log status [CRLF]
2026-09-17 09:27:03.662 RX [LOG] state=ready boot=37 session=boot-37 up_ms=91300 clock=synced setup=1 hooks=1 file_bytes=19262 generation=13 newest=12 archives=1 card_bytes=15931539456 free_bytes=15923019776 queue=0/16 high=1 drops=0 suppressed=0 truncated=0 error=none errno=0
2026-09-17 09:27:03.662 RX [LOG] measured=1 stack_min=3588 internal_min=84760 internal_largest=31732 dma_min=77264 dma_largest=31732 writes=8 slow=0 write_max_us=1863 flush_max_us=5146 sd_max_us=83851 rotations=0 pruned=0 oversized=0
2026-09-17 09:27:03.663 RX [LOG STACK] stack_mode=psram stack_bytes=8192 placement_valid=1 stack_start=0x3c213008 stack_external=1 stack_local_external=1 tcb_internal=1 tcb_bytes=352 writer_lifecycle=active stack_used_max=4604 stack_final_margin=-1
2026-09-17 09:27:03.664 RX [LOG TEST] nvs_active=0 summary_pending=0 nvs_writes=0 nvs_errors=0 sd_records=0 key_removed=0 first_nvs_ms=0 last_nvs_ms=0 first_sd_ms=0 last_sd_ms=0
2026-09-17 09:27:03.665 RX [LOG MEM] retained=boot snapshot_bytes=240 values=bytes timestamps=us
2026-09-17 09:27:03.665 RX [LOG MEM] phase=before_clock up_us=1796259 free=172680 largest=110580 heap_min_boot=172680
2026-09-17 09:27:03.665 RX [LOG MEM] phase=after_clock up_us=1796948 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 09:27:03.665 RX [LOG MEM] phase=before_writer up_us=1797004 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 09:27:03.665 RX [LOG MEM] phase=writer_entry up_us=1797212 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 09:27:03.665 RX [LOG MEM] phase=after_formatter up_us=1797297 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 09:27:03.667 RX [LOG MEM] phase=before_mount up_us=1797910 free=167100 largest=102388 heap_min_boot=166992
2026-09-17 09:27:03.667 RX [LOG MEM] phase=after_mount up_us=1881599 free=129836 largest=65524 heap_min_boot=128624
2026-09-17 09:27:03.667 RX [LOG MEM] phase=before_current_open up_us=1890828 free=129836 largest=65524 heap_min_boot=128624
2026-09-17 09:27:03.667 RX [LOG MEM] phase=after_current_open up_us=1892238 free=129836 largest=65524 heap_min_boot=128624
2026-09-17 09:27:03.667 RX [LOG MEM] phase=storage_done up_us=1907930 free=129836 largest=65524 heap_min_boot=128624
```


### JP's wake observation confirmed

JP answered: "Yes—touch alone woke it before USB."
This confirms the requested Sleeping.../screen-off/touch-only-wake behavior,
alongside the healthy boot-37 runtime statuses. The visual/runtime portion passes.
The SD close reason, reset class, wake cause, breadcrumbs and retained-clock
records still require the grouped card inspection.

Next: normal shutdown with the card installed, then place it in the reader and
confirm E:\sdcard availability. In that one visit, verify both pending tail
repair and deep-sleep evidence, preserve byte-identical originals, and prepare
unrelated-file/no-clock rotation fixtures. No further sleep repetition requested.

JP additionally confirmed: "Board operated normally." No visible anomaly was reported after touch wake.
## 2026-09-17 — 09:31 card inspection and final storage fixture

JP confirmed the card available at E:\sdcard after the normal shutdown.
[Raw evidence and hashes](bench_data/sd_logs_2026-09-17_0931/README.md) preserve
archive 12 (8136 bytes) and current (21633 bytes) before any fixture changes.

| Check | Result |
|---|---|
| Archive preservation | Archive 12 is byte-identical to the 08:52 backup. |
| Short-tail repair | Original 4087-byte prefix preserved. First 4143 bytes match the prepared fragment hash. Exactly one newline precedes boot-35 TAIL_RECOVERY action=added_newline. |
| Record continuity | Across both files: boot 34 seq 1–31, boot 35 1–27, boot 36 1–9, boot 37 1–13, with no gaps or duplicates. Only the deliberate fragment is not a normal record. |
| Deep-sleep close | Boot 36, 09:25:22.503-04:00, SESSION_END reason=deep_sleep pending=0. |
| Touch wake | Boot 37 reset=deep_sleep reset_code=8 wake_code=2; JP confirmed touch alone before USB and normal operation. |
| RTC breadcrumbs | Both valid from boot 36: main phase=sleep and writer phase=sd_close. |
| Clock after wake | Starts time=approx; sync at 09:25:51.843-04:00, up_ms=19515, prior_known=1, correction_ms=-118. |
| Final shutdown | Boot 37, 09:31:10.475-04:00, SESSION_END reason=shutdown pending=0. |

The short-tail and deep-sleep checks now pass both runtime and card inspection.
This does not add near-cap tail recovery or power-loss durability coverage.

### Preservation and unknown-clock rotation fixture

After verifying the backups, current was renamed exclusively to archive 13 and a
zero-byte current created. Archive 12 remains unchanged. No existing log was deleted.
The next cold boot should create generation 14 with two managed archives,
newest=13. Their combined content is 29769 bytes.

[fixture.json](bench_data/sd_logs_2026-09-17_0931/fixture.json) records hashes and
exact contents of six small unrelated files:
- images/diag-preserve-test.txt (dummy data in the future image folder).
- logs/diag-keep.txt.
- logs/archive-0000001.log (seven digits).
- logs/archive-00000014.log.bak (extra suffix).
- logs/Archive-00000098.log (case variant).
- logs/archive-00000099.log/keep.txt (archive-like directory).

Source inventory accepts only exact eight-digit archive basenames and regular
files. All these sentinels must survive pruning unchanged. The test is pending.

JP's next step is safely eject, insert while fully off, cold boot on USB with the
hotspot OFF, then send log status and paste it. Keep USB connected and hotspot off.
First require ready, clock=unknown, generation=14, newest=13, archives=2 and
file_bytes below 7000. Do not enable small mode until this guard is reviewed.
Later small mode should exercise natural rotation and actual pruning with no
clock sync; restore normal limits promptly and inspect all sentinel hashes.

No firmware changes, build, flash, commit or push. Stage 1 acceptance is pending.
Bad/unsupported-card tests remain explicitly deferred by JP.

## 2026-09-17 — 09:40 offline fixture startup still in progress

JP connected the web console with DTR=true, RTS=false at 09:40:14.766.
Output reported neither Wi-Fi profile connected and
"USB power present. Waiting 45 seconds before retry..."
JP sent log status at 09:40:16.971; this capture contains no response.

Source check: attemptWiFiConnection() has five attempts and four 45-second
waits plus scans. After that, USB-powered startup continues in local-only mode.
netBenchLoop() parses serial only in loop(), after setup completes; the startup
runBackgroundTick() does not parse commands. This capture is consistent with
that startup delay and does not establish a logger failure.

The earlier instructions omitted this several-minute offline startup wait.
Keep hotspot off and USB connected. Wait for local-only mode and Setup complete,
then send log status if the buffered command has not already returned a reply.
Review the same fixture guard before small mode; no test result is accepted yet.
No firmware change, build, flash, commit or push.

## 2026-09-17 — boot 38 offline fixture guard passed

[Raw serial capture](bench_data/sd_offline_startup_2026-09-17_0944.txt).

USB-powered startup exhausted Wi-Fi retries and completed in local-only mode at
09:43:31, uptime 211805 ms. The earlier buffered log status was then answered.
The second status at 09:44:12 confirms the intended starting state:

| Field | Result |
|---|---|
| Logger and clock | ready, clock=unknown, setup=1, hooks=1 |
| Managed files | generation=14, newest=13, archives=2 |
| Current growth | 2480 to 3115 bytes; writes 8 to 10 |
| Errors and drops | error=none, errno=0, drops=0, suppressed=0, truncated=0 |
| Writer | PSRAM 8192 bytes, active, placement valid, stack margin 3588 bytes |
| Memory | internal_min=128640, internal_largest=65524; no TLS workload in this offline capture |
| Rotation and pruning | Both zero before small limits |

The startup/fixture guard passes. This is not yet a preservation or offline
rotation pass. The Wi-Fi "sta is connecting, return error" line accompanies a
background retry; the following logger status remains healthy. No network fix
is proposed from this capture.

Next: keep hotspot OFF and USB connected. Check a fresh log status before changing
limits, since current continues growing. If still ready/unknown and below 7000
bytes, send log test small, wait 10 seconds, then log status and paste the output.
If the fresh size has reached 7000 bytes or another guard differs, send only
status for review. Do not restart or remove the card at this point.
The natural-growth phase follows review of that immediate post-hook status.
No firmware changes, build, flash, commit or push.

## 2026-09-17 — boot 38 small limits enabled offline

[Raw capture](bench_data/sd_offline_small_2026-09-17_0946.txt).

Fresh guard at 09:45:58 passed: current=3652 bytes, ready, clock=unknown,
generation=14, newest=13, archives=2. JP sent log test small at 09:46:37;
the hook was queued. At 09:46:51 current=4300 and writes=13 (previously 11).
Logger remains ready/unknown with no errors, drops, truncation or slow writes.
Rotation and pruning are still zero; this is not yet a completed rotation test.
Writer margin remains 3588 bytes; internal minimum 128640 and largest 65524.

One normal-operation probe reports IMU average 48.90 Hz and minimum 7.52 Hz,
with 2904 readings in 60001 ms; heap sampler maximum gap 11377 us.
Background Wi-Fi retry messages continue. The IMU minimum is retained as an
observation; this capture does not establish its cause or a logging regression.

Next: keep hotspot off and USB connected, allow eight minutes after the
09:46:51 status (approximately 09:54:51), then capture log status.
Immediately restore log test normal, wait ten seconds and capture log status
again, even if rotation has not occurred. Do not force rotation or leave small
limits enabled unattended. Expected evidence is at least one natural rotation
and pruning with clock still unknown; actual results and subsequent sentinel
hash inspection decide the pass. No firmware changes or commits.

## 2026-09-17 — boot 38 unknown-clock rotation runtime passed

[Raw capture](bench_data/sd_offline_rotation_2026-09-17_1023.txt).

The capture resumes at 10:23, rather than the suggested 09:54 check.
The longer run produced three rotations and two prunes while clock stayed unknown.

| Measurement | Before normal restore | After normal restore |
|---|---|---|
| Time | 10:23:25 | 10:23:48 |
| Logger | ready, clock=unknown | ready, clock=unknown |
| Files | generation=17, newest=16, archives=3 | unchanged |
| Current bytes and writes | 1487 bytes, 56 writes | 1600 bytes, 57 writes |
| Rotation and pruning | rotations=3, pruned=2 | unchanged |
| Errors and queue | error=none, errno=0, queue=0/16, high=1, drops=0 | unchanged |
| Other faults | slow=0, suppressed=0, truncated=0, oversized=0 | unchanged |
| Internal memory | minimum=128640, largest=65524 | unchanged |
| PSRAM writer | active, placement valid, margin=3380, used=4812 of 8192 | unchanged |

JP sent log test normal at 10:23:31.551; the hook was queued. The next status
shows the additional hook write and continued healthy operation, consistent
with processing the restore. Status does not expose the numeric active limits.
The single probe window shows IMU average 49.02 Hz, minimum 35.27 Hz,
2934 readings, 6000 heap samples and maximum sampler gap 11482 us.
JP did not include a separate visual-operation report in this attachment.

The runtime portion of unknown-clock rotation/pruning passes. From initial
generation 14 with archives 12 and 13, three rotations and two prunes predict
archives 14–16 with current generation 17. Confirm names and size-triggered
FILE_OPEN records on the card. No claim of sentinel preservation from status:
all six hashes and the archive-like directory still require inspection.

Next: unplug USB, leave the board stationary and wait for normal shutdown.
Once fully off, put the card in the reader and confirm E:\sdcard availability.
Back up retained logs, verify clock=unknown on the size rotations, sequences,
final close, and all six sentinel names/contents against the 09:31 manifest.
This closes the remaining storage evidence before presenting Stage 1 for JP's
acceptance. Bad/unsupported-card tests remain deferred; existing limitations
remain recorded. No repeat test, firmware changes, build, flash, commit or push.

## 2026-09-17 — final offline-rotation card inspection passed

JP confirmed the card available. Read-only inspection backed up all ten files
(27614 bytes) in [final evidence](bench_data/sd_logs_2026-09-17_offline_rotation/README.md).
Every backup was compared byte-for-byte and recorded in a SHA-256 manifest.
The card was not modified.

Managed archives are 14 (8059 bytes), 15 (7936) and 16 (7942);
current is generation 17, 3325 bytes. Boot-38 sequences are exactly 1–61,
without gaps or duplicates. Every record has time=unknown.
Generation 14 began with empty_recovery. Generations 15–17 opened with reason=size,
confirming three natural rotations. The two original managed archives, 12 and 13,
were pruned; both are already preserved in the 09:31 backup.
Final record is SESSION_END reason=shutdown pending=0 at uptime 2774316 ms.

All six fixture names, sizes and hashes match the original manifest.
The archive-like directory and its child remain intact. No unrelated file was lost.
This completes the pending offline-rotation/pruning and preservation checks.

The [Stage 1 acceptance checkpoint](sd_diagnostics_stage1_checkpoint.md) summarizes
passed evidence and explicit limits. Recommendation: accept the tested scope,
then implement Stage 1B USB retrieval. JP has not yet accepted this checkpoint.
No firmware change, build, flash, commit, push or card cleanup was performed.

## 2026-09-17 - JP accepts Stage 1

JP: "Please commit and push. Yes I accept Stage 1 let's continue."
Stage 1 is accepted with the checkpoint's recorded limitations. Stage 1B USB
retrieval is authorized. JP continues to compile and flash from VS Code.

## 2026-09-17 - Stage 1B implementation prepared; hardware gate pending

Stage 1 evidence and acceptance pushed as 3d0b125 and 3f5b309.
Prepared the single-writer USB command module and Web Serial file browser.
The new code remains uncommitted pending JP's bench feedback. No firmware was
compiled or flashed. Fault hooks are off; logging and the PSRAM writer stay on.
The generated build/build_amoled-1-8/sketch/companion.ino.cpp was removed.

Eleven Node checks pass against the actual page code, including saved archive bytes,
CRC, fragmented reads, strict validation, damaged-line cancellation and retry,
missing END and preservation of partial protocol input when clearing the console.
Firmware source review used installed core 3.1.3 HWCDC behavior; firmware compilation,
writer stack/heap impact, throughput, timing and board behavior require JP's tests.

JP requested a shorter status section in the main plan. Replaced its long
chronology with the current acceptance/implementation state and evidence links;
the detailed bench history and Waveshare SDMMC reference remain available.

First test: [Stage 1B handoff](../src/diagnostics/STAGE1B.md).
Download archive-00000014.log (8059 bytes, CRC32 99C24CB1), capture full status
before/after, and compare the saved file with the existing card-reader reference.
No further gate tests requested until that result is reviewed.


## 2026-09-17 11:11 - First Stage 1B archive download rejected

Boot 41, hooks off, logger ready, three archives, generation 17.
JP requested archive 14. The page sent log abort within about 71 ms and
received @@ERR reason=aborted. It reported an oversized or non-ASCII protocol
line; the original page did not retain that line, so the cause is not yet known.
No verified file was saved. This does not establish corruption on the SD card.

Pre-download status: internal minimum 33852 bytes, largest block 25588,
writer stack margin 3588, queue drops zero, logger error none.
There is no post-download status yet. Stage 1B acceptance remains pending.

Added browser-only rejection evidence: bounded escaped line preview, length,
truncation flag and first unexpected character position/code point.
Validation and firmware remain unchanged. Twelve browser tests pass,
including non-ASCII rejection and abort confirmation.
Next: reload the page, keep all transfer test switches off, retry archive 14,
then capture log status and the complete console.


## 2026-09-17 11:17 - Archive retrieval stalls; cleanup remains healthy

Same boot 41. Reconnection listed all four managed files successfully.
Archive 14 was requested at 11:17:06.452. The board replied
@@ERR reason=stalled at 11:17:11.510 (about five seconds later).
JP subsequently reported the page progress as 1152/8059 bytes.
The fresh 11:19:59 status retained bytes=1296 result=stalled active=0 paused=0.
The sender count measures bytes accepted by HWCDC, not receipt by the browser.
These observations do not yet identify why output stopped.

Logger remains ready, current.log grows, drops=0, error=none.
Largest internal block remains 25588 bytes. Writer stack margin reached
2980 bytes (5212 of 8192 used); no allocation failure is reported.

Added diagnostic fields to the firmware's existing stalled reply:
phase, bytes, line_bytes, tx_free and write_bytes. Values describe the last
transfer send attempt; -1 means that stage of the attempt was not reached.
No timeout, chunk size, USB setting or scheduling change.
Browser-only tests now total 13, including partial transfer followed by a
diagnostic stalled reply without saving a file. Firmware was not compiled or flashed.
Next: JP rebuilds/flashes, retries only archive 14 with test switches off,
then sends status and the full console. The cause and Stage 1B gate remain open.


## 2026-09-17 11:25 - Corrupted file list; same-core writer trial

After JP flashed the stall diagnostics, boot 43 remained ready with zero drops.
The initial list exchange was corrupted. The received STATUS line ended with
free_mib=15185 filesent.log size=21646, joining a status prefix to the suffix
of a FILE record. The page rejected the incomplete list and closed the port.
The capture shows a page-initiated disconnect, not evidence of a board reset.

Source evidence:
- setup() calls USBSerial.begin(), then diagnosticsStart(), on Arduino core 1.
- Installed SDK esp_intr_alloc.h says allocation occurs on the calling core.
- Core 3.1.3 HWCDC.cpp:350 allocates the USB interrupt. Its ISR at lines 95-102
  fills and commits a packet; write() at 442 and 463 also commits the FIFO.
  The task TX mutex is not shared with the ISR.
- SDK usb_serial_jtag_ll.h:135-143 writes until space runs out and returns the
  actual count. HWCDC's ISR ignores that count and returns the whole ring item.
  An opposite-core task committing the FIFO during this loop could lose bytes.
- Stage 1's writer was pinned to core 0. Stage 1B added repeated USB sends there.

This is a source-supported race hypothesis, not a confirmed hardware diagnosis.
Prepared a focused trial: pin the same writer to its setup caller's core
(core 1 in this build), for both stack modes. Report actual writer_core in status
and the startup banner. Keep the PSRAM stack, priority, queue, SD ownership,
timeouts, chunk size and pacing unchanged. No extra task or core-library patch.
The earlier stalled-reply diagnostics remain enabled.

No firmware build, flash or commit by the assistant. Bench confirmation is pending.
First repeat connection, full status, and archive 14 download, then full status.
Expect writer_core=1. If transport recovers, repeat same-session Live/memory
measurements because changing affinity can affect scheduling and allocation timing.
Stage 1 acceptance remains historical; Stage 1B has not passed.


## 2026-09-17 11:32 - Same-core trial lists files successfully

JP flashed the affinity trial. Boot 45 status confirms writer_core=1,
PSRAM stack 8192 bytes, placement_valid=1, internal TCB.
Initial automatic STATUS and USB replies arrived intact; all four managed
files were listed, without the previous incomplete-list disconnect.
Full status also arrived. Logger ready, hooks off, queue drops zero, error none.
Pre-download largest internal block 31732 bytes, internal minimum 84652,
writer stack margin 3588 bytes (4604 used).

This is an initial list/status success only. The capture contains no archive
download, so transfer integrity, throughput and the race hypothesis remain
unconfirmed. Next: download archive 14 in the same session, send status, and
provide the console and saved file path for byte comparison.


## 2026-09-17 11:33 - Core-1 transfer still stalls; upgrade investigation

Boot 45, writer_core=1. Archive 14 reached 1008/8059 bytes in the page.
Request was at 11:33:51.692. At 11:34:07.730 the page sent log abort after
its missing-response watchdog expired; the board acknowledged aborted.
The fresh 11:34:53 status retained bytes=1152 result=stalled active=0 paused=0.
Thus the device had already stalled; its original error did not arrive in
the capture. The earlier 11:32:55 STATUS companion USB line arrived only at
11:33:51, alongside the next normal probe. Output delivery is delayed as well
as previously corrupted. Pinning to core 1 alone did not resolve retrieval.

Logger remained ready, drops=0, error=none. Largest internal block 31732,
internal minimum 84652, writer stack margin 2980 bytes. This run contains
no TLS or Live window and cannot replace those memory/performance gates.

Checked installed HWCDC.cpp and SDK usb_serial_jtag_ll.h. The old ISR ignores
partial FIFO write counts, lacks a terminating empty-packet flush in its
empty-ring branch, and has unprotected interrupt enable changes.
These are credible causes; the exact hardware sequence remains unobserved.

JP suggested a newer core and offered to check the maker's product page.
Paused the proposed project-local transmit-service workaround; it was not implemented.
Read-only upstream investigation found:
- [Espressif PR 12606](https://github.com/espressif/arduino-esp32/pull/12606)
  fixes HWCDC sustained-write data loss and hangs, including interrupt races,
  partial FIFO writes and missing terminating zero-length packets.
- [Core 3.3.11 HWCDC.cpp](https://github.com/espressif/arduino-esp32/blob/3.3.11/cores/esp32/HWCDC.cpp)
  contains those fixes.
- [Release 3.3.11](https://github.com/espressif/arduino-esp32/releases/tag/3.3.11)
  uses ESP-IDF 5.5.5; this is a candidate, not validated for this application.

Local README.md records reset-loop I2C conflicts with 3.2.0 and 3.3.0.
The current link map identifies ESP32_IO_Expander 0.0.3 ESP_IOExpander.cpp
as the caller pulling in legacy i2c_driver_install. The library compatibility
and PSRAM-stack/SDK audit must precede an upgrade trial. Board support alone
does not establish compatibility of this pinned application.

No core, library, build profile or firmware change in this investigation.
No compile, flash, commit or push. Proposed next direction: review the maker's
exact core/library recommendation, then prepare an isolated upgrade build profile
with 3.1.3 preserved for rollback. Stage 1B remains pending.


### 2026-09-17: core 3.3.11 trial prepared; no new bench result

JP authorized the smaller upgrade trial after installing core 3.3.11 and Adafruit XCA9554.
The new Maker profile was still pinned to 3.1.3; it is now corrected, with both Adafruit libraries.
The sketch chooses the expander by core version, preserving the original profile paths.
Graphics and the Stage 1B USB implementation are unchanged for this experiment.
Installed SDK and tagged-source checks found no blocker for attempting compilation.
See [core trial preparation](core_3_3_11_trial.md). JP compiles next; no new firmware has been built or flashed by Codex.


### 2026-09-17: first core 3.3.11 compilation failed in graphics 1.4.9

The intended core and Adafruit libraries were selected. The old graphics
library fails to compile three spiFrequencyToClockDiv calls in its ordinary
SPI and SPI DMA backends; the core now requires a bus pointer argument.
The active display uses QSPI, but these other source files are compiled too.
The downloaded original-board Waveshare graphics 1.6.4 includes compatibility
helpers for this exact signature change. See [trial details](core_3_3_11_trial.md).
No new hardware result or flash. Graphics migration choice pending JP.


JP subsequently accepted the separate Waveshare graphics 1.6.4 copy.
The new profile now selects it, and constructor/brightness API changes are gated
for the new core. The old graphics folder and both 3.1.3 profiles are preserved.
Copied library files were hash-verified. No second build or hardware test yet;
JP recompiles next. The 20 MHz display clock and memory gate remain unchanged.


The second 3.3.11 compile selected graphics 1.6.4 correctly, but reported the
removed BLACK alias in initDisplay(). Replaced it with RGB565_BLACK, present
in both library versions with the same value. Trial remains compile-pending;
no flash or new bench result.


### 2026-09-17: core 3.3.11 compilation passed

JP's build completed successfully with the intended core 3.3.11, separate
Waveshare graphics 1.6.4, XCA9554 1.0.0 and BusIO 1.17.4. The other pinned
application libraries remain selected; the old ESP32_IO_Expander is absent.
First bench flash/startup/status check is next. USB download, memory and
runtime compatibility remain unverified on this build; Stage 1B is pending.


### 2026-09-17 14:21: core 3.3.11 first hardware/status checkpoint

JP reports normal Latest, Back, Live, inclinometer, G-meter, motion and IMU.
Boot 47 lists all four files cleanly. Logger ready, clock synced, hooks=0,
no errors or drops; final USB state idle and appends not paused.
Internal minimum=92760, sampled largest minimum=51188 bytes; writer PSRAM
placement valid, internal TCB=352 bytes, core 1, remaining stack margin=3768.
The short 18.267-second normal probe averages 42.31 Hz IMU (min 26.31 Hz),
to revisit with a full window rather than treating visual success as a
performance result. TLS/Live quantitative gates remain pending.
See [detailed trial measurements](core_3_3_11_trial.md).
Next: the original archive-14 retrieval, expected 8059 bytes and CRC 99C24CB1.
No code change, compile, flash or commit by Codex.


### 2026-09-17 14:22: first successful USB archive download on core 3.3.11

Boot 47: archive 14 downloaded in 0.12 s, 8059 bytes, CRC OK;
post-status reports bytes=8059, result=ok, active=0 and paused=0.
Logger ready with zero drops/errors, writer stack margin 3320 bytes,
internal minimum 92760 and sampled largest minimum 51188 bytes.
JP confirms the file was saved. Its path and independent byte comparison
remain pending. Strong evidence favors the newer HWCDC fixes, but the core,
graphics and expander changed together, so a single causal fix is not proven.
The full 60-second normal IMU window averages 42.45 Hz; this warrants follow-up
and cannot now be attributed solely to the short initial window.
See [trial details](core_3_3_11_trial.md). Stage 1B is not yet fully accepted.


### Archive-14 independent file comparison passed

Located `C:/Users/photo/Downloads/47-archive-00000014.log` and compared its raw
bytes with `docs/bench_data/sd_logs_2026-09-17_offline_rotation/logs/archive-00000014.log.txt`.
They are identical: 8059 bytes, CRC32 99C24CB1,
SHA-256 5ec652cb9e9268d92c44511b452746bbc43f82d3dc246eb9eea69a7e23c12eb0.
This completes the small-archive independent integrity check, not the entire USB gate.
Next: one normal current.log download, immediate status, then another status
70 seconds later to check append growth. All page fault switches stay off.
Current snapshot prefix comparison and later gate cases remain pending.


### 2026-09-17 14:28: current download and append resume passed

Boot 47 current.log: 128183 bytes in 1.28 s, CRC OK, saved locally with
CRC32 11E25C6B and complete UTF-8 records. USB returns to idle/unpaused.
Current grows from 128345 to 128908 bytes over the next 63.298 seconds;
writes 18 -> 19, no drops or errors, writer stack margin 3320 bytes.
Current-prefix integrity comparison and remaining stress gates stay pending.

The saved file additionally reveals an earlier task-watchdog reset into boot 47
at approximately 14:20:38, before both successful downloads. JP was asked for
context, especially VS Code monitor closure. Cause is not established.
Full-minute IMU averages remain about 42.5 Hz. See [trial details](core_3_3_11_trial.md).


### JP clarification: earlier reset associated with VS Code monitor closure

JP confirms a freeze/restart when closing the VS Code monitor, followed by
stable operation. The successful boot-47 USB downloads remain valid results;
monitor-close behavior remains an open issue on the core 3.3.11 trial.
This observation does not establish the underlying watchdog mechanism.
Next: current plus newest three with page fault switches off, then status.
No firmware change or rebuild. The separate 2 MiB throughput gate is pending.


### 2026-09-17 14:37: four-file USB bundle passed

JP confirms current plus archives 16, 15 and 14 downloaded successfully.
All browser CRC checks pass: 157514 bytes total in 1.742 seconds from first
command to final completion. Boot 47 remains ready with no drops or errors;
USB ends idle and unpaused. Writer stack margin remains 3320 bytes.
Internal minimum=44540 and sampled largest minimum=31732 bytes. The free-heap
low already appears in the pre-transfer normal probe; its cause is not isolated.
Next is a full Live baseline with no retrieval, followed by a same-session
Live/download comparison. The 2 MiB throughput gate remains pending.
See [trial details](core_3_3_11_trial.md). Documentation only; no commit.


### 2026-09-17 14:39: full Live baseline, no retrieval

JP reports normal video for the entire cycle. Boot 47: 181 frames in 60.426 s,
reported 3.0 fps, average frame 334 ms, first frame 1190 ms, max gap 1106 ms.
Decode=61 ms, blit=80 ms, HTTP=325 ms. Live probe largest minimum=26612 bytes;
TLS windows each have largest minimum=31732. Exact boot free-internal low=37628.
Writer stack margin=3320; no errors, drops or reset. Logger remains ready.
The 10 ms probe has 6042 samples and max gap 11226 us.
Next: repeat a full Live cycle in the same sitting, downloading current plus
newest three once after about ten seconds, then capture status.
See [trial details](core_3_3_11_trial.md) for the baseline and migration follow-ups.


### Live with downloads: visual report received, capture pending

JP reports continuous video with the in-frame seconds advancing during the
downloads, and all files downloaded successfully. The supplied attachment
(d765b4c4-885d-48ed-a5cc-f51d1ae11696) repeats the 14:39-14:40 baseline capture;
it contains no new get commands or DOWNLOAD lines. Record this as a positive
visual report only. Paired timing, memory and CRC verification await the
actual concurrent-transfer capture; no repeat test requested yet.


### 2026-09-17 14:45: measured Live plus bundle passed

JP repeated the test and supplied the concurrent-transfer capture. Both
baseline and download runs produce 181 frames, in 60.426 and 60.256 seconds
respectively (about +0.28 percent fps, within the approximately 5 percent gate).
Maximum frame gap is 901 ms versus baseline 1106 ms. Four downloads all have
CRC OK, totaling 165287 bytes in 1.971 seconds during Live.
Live and TLS sampled largest minima=31732 bytes, above 20480. Writer stack
margin stays 3320. Same boot 47; no errors/drops/reset, USB idle/unpaused.
The earlier visual report and this replacement capture now support the
ordinary-size concurrent-download case. See [trial details](core_3_3_11_trial.md).
Next: page-side damaged-line rejection followed by a normal current.log retry.
Remaining Stage 1B gates are still pending; no firmware changes or commit.


### 2026-09-17 14:49: damaged-line rejection and retry passed

The page rejects the deliberately malformed current.log data line, sends
log abort, and receives reason=aborted with abort confirmation. Normal retry
saves 145280 bytes in 1.44 s with CRC OK. JP confirms success.
Boot remains 47, logger ready, USB idle/unpaused, no errors or drops.
Writer margin=3320; internal minimum=37552, retained largest minimum=26612.
Next: stopped-browser-reader case; host buffering may make this file too
small to exercise the five-second firmware stall guard. Capture status
before retry to retain the first attempt's result. Documentation only.


### 2026-09-17 14:52: stopped reader triggers retained stalled result

Final USB status: bytes=9072 result=stalled active=0 paused=0. Logger ready,
no errors or drops; boot 47 and memory/stack minima unchanged. The page did
not receive the original error, then timed out and obtained an abort reply.
Firmware cleanup precedes a bounded five-second terminal-reply window, which
may expire while browser reads remain paused. The capture confirms cleanup
but does not directly time the five-second stall or prove error delivery.
Next: normal current download and status with all switches off, no restart.
See [trial analysis](core_3_3_11_trial.md). Recovery retry remains pending.


### 2026-09-17 14:54: normal retry after stopped-reader test failed

JP observes roughly 68 percent progress then a stop. Firmware retains
bytes=101808 result=stalled; page later aborts for missing response/END.
Logger cleanup succeeds (ready, USB idle/unpaused, zero drops/errors), same
boot 47, unchanged memory minima and stack margin. USB recovery gate fails;
Stage 1B remains pending. A causal link to the previous paused read is not
yet proven. Detailed sender evidence was absent because its terminal reply
was not received. Next: reopen only the web serial connection, verify boot
and uptime, retry current.log, then status. No firmware changes.
See [trial analysis](core_3_3_11_trial.md).


### 2026-09-17 14:58: connection-only recovery and new failure timing evidence

Reopening web serial restores a 150788-byte current download in 1.53 s, CRC OK.
Boot remains 47 with continuing uptime; logger/USB clean, memory and writer
margin unchanged. This is one successful recovery without board reset.
The saved log supplies earlier USB_GET_END durations: deliberate stopped
reader=5133 ms at 9072 bytes; ordinary failed retry=1054 ms at 101808 bytes.
The latter cannot be a five-second no-progress expiry. The sender also maps
negative send results to stalled; exact failure branch remains unrecorded
in the surviving status. The later browser timeout is a separate event.
Recommend retaining exact stop/send observations in status before repeating
fault tests. No firmware change implemented; Stage 1B remains pending.
See [trial analysis](core_3_3_11_trial.md).


### 2026-09-17 15:11: diagnostic build flashed, ordinary retrieval passed

JP again needed USB cable unplug/replug after closing the VS Code monitor;
board restarted. Boot 49 then completes a 160243-byte current download
in 1.61 s with CRC OK. New LOG USB FAIL/SEND lines are present and valid=0
throughout, as expected before a stall. Logger ready, zero drops/errors,
USB idle/unpaused. Stack margin=3320, internal minimum=94856 and sampled
largest minimum=51188. No reset during this captured web transfer.
Next: paused-reader failure with status, then ordinary retry with status,
without reconnecting, to capture the exact failure checks if it recurs.
Monitor-close freeze remains open; Stage 1B not yet accepted.


### 2026-09-17 15:13: five-second stall identified, normal retry passed

Boot 49 snapshot: path=stop_guard, idle_ms=5000, elapsed_ms=5138,
check=space, tx_free=119 versus line_bytes=201, write_bytes=-1, bytes=9216.
Browser receives the original stalled error. This confirms the intentional
no-progress timeout and that no write was attempted with insufficient space.
Normal retry succeeds without reconnecting: 162580 bytes in 1.62 s, CRC OK.
Final USB idle/unpaused/result=ok; retained snapshot remains unchanged, as
designed. No logger errors/drops or new memory/stack low.
This case passes; the previous early normal-transfer failure remains open.
Next: three normal current downloads, status after each, stop on failure.
See [trial details](core_3_3_11_trial.md). Documentation only.


### 2026-09-17 15:16: three consecutive ordinary downloads passed

Boot 49: 164029, 164352 and 165239 bytes in 1.64, 1.69 and 1.75 s;
all CRC OK. Each status is ready, no errors/drops, USB idle/unpaused/result=ok.
Memory minima and writer margin remain unchanged (94856/51188 bytes and
3320 bytes respectively). Historical stall snapshot stays at up_ms=248461.
The intermittent early failure remains unresolved. Next is a longer paused
read (at least twelve seconds), status, normal retry and status without
reconnecting, to cover late resumption after the terminal-reply window.
See [trial details](core_3_3_11_trial.md). Documentation only.


### 2026-09-17 15:19: longer reader pause and recovery passed

Stalled reply arrives 15.685 s after request. Retained diagnostics identify
the five-second guard: elapsed_ms=5124 idle_ms=5000, check=space,
tx_free=104 for a 201-byte line, no write attempted, bytes=9072.
Normal retry saves 167580 bytes in 1.74 s with CRC OK, same boot 49.
Logger clean, USB idle/unpaused, memory/stack minima unchanged.
No need to repeat for omitted intermediate status: the failure snapshot
survived the successful retry. Error delivery may have been buffered.
Move to retrieval while MQTT is unavailable; earlier unexpected early
failure and remaining Stage 1B gates stay open. Documentation only.


### 2026-09-17 15:22: USB bundle with MQTT off passed

All four CRC checks pass: 169031/7942/7936/8059 bytes; current takes 1.66 s.
Two ghost-broker retries take about five seconds each. Downloads occur
between those attempts, so blocking-connect overlap is not exercised.
Real broker reconnects in 508 ms after on. Same boot49; logger ready,
USB idle/unpaused, no drops/errors. Writer margin=3320, largest minimum=51188.
Free-internal low=94596 occurs in real MQTT connect; retained stall unchanged.
Next planned gate needs a 2 MiB archive. Proposed test-only writer fixture
generator avoids card removal; no such change implemented yet.
See [trial details](core_3_3_11_trial.md). Documentation only.


### 2026-09-17 15:39: 2 MiB fixture created; download failed

Boot51 creates archive18 (2097152 bytes) in 52.523 s, result=ok.
Download ends with retained result=disconnected, bytes=904320 (43.125%),
then the page times out and obtains abort confirmation. Same boot responds;
logger ready, zero drops/errors, internal minimum=92524, largest=49140,
writer margin=3336. USB inactive/unpaused. The archive has no 120-second limit.
Failure snapshot valid=0 because it currently captures stalled only.
Installed HWCDC source permits transient false connection readings; firmware
currently aborts on the first false. Brief connection-loss tolerance plus
retained disconnected diagnostics is the proposed next correction, pending
implementation. Keep archive18 for retry; no card removal or repeat generation.
See [source analysis and details](core_3_3_11_trial.md). Stage 1B remains open.
Documentation only; no build, flash, commit or push.


### Boot 51 follow-up: connection-loss tolerance ready for JP

Implemented JP-approved 1000 ms sustained-loss guard. No sending on a false
connection reading; brief recovery resumes the pending line. Five-second
no-progress protection and current's 120-second limit remain unchanged.
Retained diagnostics now include disconnected failures and connection-loss
counts/durations. Ten guard source simulations and fourteen browser checks
pass; no firmware build or flash. Hardware validation is pending.
Next: retry existing archive18 once, then status before refresh or retry.
See [handoff](../src/diagnostics/STAGE1B.md). No commit or push.


### 2026-09-17 15:52: 2 MiB gate passed; 3 ms link loss recovered

Boot53 downloads archive18, 2097152 bytes in 20.39 s, CRC OK.
Saved file independently matches every expected byte (CRC32 8D218D21).
LOG USB LINK losses=1 max_loss_ms=3 pending=0 confirms a brief connection
indication recovered without terminating the transfer. No retained failure.
USB idle/unpaused/result=ok; logger ready, no drops/errors. Internal minimum
95088, largest minimum51188, writer margin3320. The 120 s current deadline
exceeds 3 x 20.39 s = 61.17 s, so needs no increase.
Next: same archive with MQTT off, starting immediately to overlap its first
blocking retry, then on and status. No build or card removal needed.
See [trial details](core_3_3_11_trial.md). Stage1B acceptance remains pending.


### 2026-09-17 15:55: 2 MiB download during blocking MQTT retry passed

Boot53: archive18 downloads with CRC OK in 22.67 s; every saved byte independently
verified. Command was sent during retry 1 and finishes during retry 2, confirming
actual overlap with a blocking MQTT attempt. Browser time includes command wait.
One 4 ms USB connection indication recovers. No logger errors/drops, USB idle,
writer margin=3320 and largest minimum=51188 unchanged. MQTT on reconnects in 618 ms.
Free-internal low becomes 92472 in the real-broker connect probe. Same boot remains.
JP requests commit/push checkpoint; Stage1B remains open, fixture flag=1/hooks=0.
Keep archive18 for remaining tests, then verify serial deletion and disable flag.
See [timing and evidence](core_3_3_11_trial.md).


### 2026-09-17 16:02: fresh Live baseline recorded

Boot 53, no concurrent download; JP reports normal video. 182 frames/60.242 s
=3.021 fps; frame 331 ms, first 1323 ms, max gap 941 ms. Live sampled largest
minimum 28660; TLS 31732; internal low 39960; writer margin 3320. No drops/errors.
Next paired cycle downloads archive 18 about 10 s into Live, same session/build.
Compare against this baseline, with about 2.870 fps as the 5% lower boundary.
See [full baseline](core_3_3_11_trial.md). Documentation only.


### 2026-09-17 16:05: large Live overlap works, FPS comparison outside target

JP reports normal video. Archive18 downloads in 18.66 s with CRC OK and
independent exact-byte match. Live 167 frames/60.378 s = 2.766 fps versus
baseline 3.021 fps: about 8.45% lower, outside the 5% target. Maximum gap
941 -> 1002 ms; HTTP 322 -> 352 ms, decode 61 -> 65 ms, blit 80 -> 87 ms.
Memory remains above the gate: largest 28660, TLS largest 31732, writer
margin 3320. No errors/drops, same boot 53. One 4 ms USB loss recovered.
Integrity and memory pass; performance remains unresolved. Next: another
full Live-only cycle in the same sitting to check network/timing variation
before any pacing change. See [comparison table](core_3_3_11_trial.md).
Documentation only; no firmware change, build, flash, commit or push.


### 2026-09-17 16:10: Live-only control returns to baseline

JP reports normal video. 181 frames/60.302 s =3.002 fps, within 0.65% of the
first 3.021 fps control. The intervening loaded run 2.766 fps is 7.85% lower
than this control. HTTP/decode/blit return to 324/61/80 ms. Memory minima
and stack margin remain 39728/28660/3320 bytes; no drops/errors, same boot 53.
This strengthens download-interference evidence; the 5% FPS gate remains
unmet despite normal visual operation. Recommend a small Live-only USB pacing
adjustment, then fresh paired measurements. No firmware changes yet.
See [A/B/A results](core_3_3_11_trial.md). Documentation only.


### Live-only pacing adjustment prepared for JP

JP approved a small pacing trial. During Live, USB sends one protocol line
per writer turn with a 3 ms tick-based yield; outside Live it retains four
lines and one tick. Both archive and current append-pause paths are covered.
Live flag reads are atomic. No changes to probes, timeouts, tasks or priorities.
Ten guard simulations and fourteen browser checks pass; performance remains
unverified until a fresh same-session Live-only and concurrent-download pair.
See [implementation handoff](../src/diagnostics/STAGE1B.md).
No firmware build, flash, commit or push.


### 2026-09-17 16:20: pacing-build Live baseline ready

Boot 55, JP reports normal video. 180 frames/60.170 s =2.992 fps; frame 334 ms,
first 1109 ms, maximum gap 1025 ms. TLS largest minimum 31732; full Live 28660;
internal low 41476. Writer margin 3752 before any download this boot.
No errors/drops, queue 0/16 high 1, USB resultnone. Use this baseline for the
same-build concurrent 2 MiB test, with 5% lower FPS boundary 2.842.
Next: full Live, archive 18 about 10 seconds in, status after both finish.
See [full baseline](core_3_3_11_trial.md). Documentation only.


### 2026-09-17 16:22: paced Live download valid, tradeoff not compelling

Boot 55, JP reports normal video. 2 MiB download 54.73 s; CRC and independent
exact-byte verification pass. Concurrent Live 2.833 fps versus fresh 2.992:
5.29% lower, still outside 5%. Max frame gap 1037 vs 1025 ms. Largest 28660,
TLS 31732, stack margin 3320; no errors/drops. Two 4 ms-or-shorter USB losses recover.
Pacing extends overlap to almost 50 s and transfer to about 2.9x the prior
concurrent run. 3 x 54.73 =164.19 s also exceeds 120 s for a mixed-workload
margin comparison. No archive timeout occurred. Recommend dropping this
optional pacing experiment and seeking JP's explicit acceptance of the rare
unpaced performance cost, not further tuning. Neither reversal nor gate
exception is applied yet. See [full trial](core_3_3_11_trial.md).


## End-of-day decision: pacing reverted, checkpoint requested

JP agreed to revert the optional Live pacing and stop tuning this rare overlap.
All four pacing-only source files were restored to a1dae68 after reviewing their
diffs. This also removes the atomic Live flag added solely for the writer read.
The USB connection-loss correction, retained diagnostics and fixture commands
remain. The board still runs the paced build until JP rebuilds/flashes tomorrow.
The narrow concurrent-Live FPS exception and remaining Stage 1B gates, including
log test del 18 validation, are captured in the linked end-of-day checkpoint.
JP requested commit and push. No build/flash or card deletion by Codex.


### 2026-09-18 08:26: restored pacing policy flashed; Latest passes

JP compiled/flashed the current reverted source in VS Code. Boot 58 connects
Wi-Fi and MQTT; web status/list succeeds with five managed files and archive 18
still present. Latest retrieves 33871 bytes, HTTPS response 914 ms, total 2173 ms
from button press, JPEG decode and screen return successful. Same-day USB
recovery/download testing can proceed; this capture contains no file download.
No comparison with yesterday's network timing is used as a performance gate.

HTTPS probe:913 ms,92 samples at 10 ms, sampled largest minimum 31732,
free-internal low 44272. Logger ready/synced, hooks 0, fixture enabled 1,
queue 0/16 high 1, no errors/drops/suppression/truncation. Writer PSRAM placement
valid; margin 3752 before any transfer this boot, core 1. USB idle/unpaused,
resultnone, no retained failure/loss. Current 225839 bytes, generation 17,
newest 18, four archives. Boot stays 58 throughout the capture.

Normal IMU averages 42.77/42.87 Hz in short windows, then 43.19 Hz over a minute.
That final window includes minimum 8.62 Hz; cause is not identified by this
capture. Record it for the existing IMU/core follow-up, without attributing
it to SD or USB or claiming the minimum is normal. No reset/error is shown.

Next single bench case: archive 18 manual Cancel download after visible
progress, await abort confirmation, status, then retry the same archive to
completion and status again without resetting/reconnecting. This exercises
explicit cancel cleanup and gives a normal download on the restored build.
Keep Live stopped, MQTT connected, browser fault switches off. Do not delete
archive 18 yet. Documentation only; no firmware change/build/commit/push.


### 2026-09-18 08:31: manual cancel and same-connection retry passed

JP reports normal test operation. Boot 58, archive 18 request 08:31:01.713;
log abort 08:31:04.291; device confirms aborted 08:31:04.305 (14 ms host interval).
Status reports USB idle/unpaused, bytes 264240 resultaborted. No logger error,
drop or retained transport failure. This is an expected cancellation, not a fault.

Without reconnect/reset, retry at 08:31:19.386 completes 08:31:39.647:
2097152 bytes in 20.26 s, decoded 103514 B/s, wire 146104 B/s, CRC OK.
Independent verification of Downloads/58-archive-00000018.log matches every
reference byte, CRC 32 8D 218D 21. One 2 ms connection indication recovered.
Final same boot 58, logger ready/synced, queue 0/16 high 1, no errors/drops/
suppression/truncation; USB idle/unpaused/resultok. Internal low 44272 and
largest minimum 31732 unchanged; writer margin 3320 after exercising download,
consistent with earlier transfer use. Current grows 228992 ->229335, writes 14 ->16.
Three-times download time 60.78 s remains below the 120-second current limit.

Manual cancel cleanup and immediate successful reuse pass on the restored
firmware. Next single case: close only the web tab while archive 18 is actively
downloading, leave USB attached, wait 10 seconds, reopen/connect and status;
then retry archive 18 and status without resetting. A closed page loses its
console, so preserve pre-close output if needed and paste the new console.
Record any board freeze/reset; stop if unresponsive. This is page-close recovery,
not the separate physical USB-unplug case. Documentation only; no firmware
change, build, flash, commit or push.


### 2026-09-18 08:35: page-close recovery and retry passed

JP reports normal operation; reopening the HTML page took longer than the
suggested ten seconds. This does not invalidate the cleanup/recovery case.
The new capture begins after reopening. Boot 58 remains unchanged; initial
status uptime 547121 ms versus the prior test 338559 ms, so no intervening reset
is indicated. Exact tab-close time is absent; do not infer a disconnect latency.

Retained failure identifies the expected no-reader stall: archive 18, bytes 183744,
phase=data, path=stop_guard, elapsed 6783 ms, idle 5001 ms, loss 0 ms. Space 117
was below the 203-byte pending line; write_bytes=-1 means no write attempted.
Cleanup left USB idle/unpaused and logger ready without errors/drops. Reopening
runs automatic listing, which resets the generic byte count to 0; the retained
failure snapshot preserves the aborted archive's actual submitted byte count.
The five-second guard handled the closed page even without a disconnected result.

Retry completes 2097152 bytes in 20.18 s, decoded 103911 B/s, wire 146665 B/s,
CRC OK. Downloads/58-archive-00000018 (1).log independently matches every expected
fixture byte (CRC 32 8D 218D 21). One 1 ms link indication recovers during retry.
Final boot 58 uptime 578971 ms; USB idle/unpaused/resultok. Retained stalled
snapshot remains historical by design, not an ongoing failure. Memory lows
44272/31732 and writer margin 3320 unchanged; queue 0/16 high 1, no errors/drops/
suppression/truncation. Current grows 231936 ->232279, writes 22 ->24.

Next single case: physical USB unplug during archive 18 download on JP's
battery-equipped board. Leave browser tab open; unplug only USB after progress,
wait about 10 seconds observing board, reconnect cable and web port, status,
then ordinary retry and status. Stop/report if board powers down, freezes or
restarts; do not treat it as a clean transport-only result. Keep archive 18.
Documentation only; no firmware change, build, flash, commit or push.


### 2026-09-18 08:39: battery-powered USB unplug and recovery passed

JP reports normal operation. Capture begins after reconnection; boot 58 and
uptime 779810 ms continue the earlier boot, with no reset indicated. Retained
archive 18 failure: reason=disconnected, loss_ms=1000, elapsed 2955 ms,
idle 1004 ms, bytes 200880. Last check connected_before_space, no write attempted.
This exercises the one-second sustained-loss guard. Automatic reopening/listing
resets generic USB bytes/link counters; retained failure preserves the evidence.

Retry 2097152 bytes in 20.41 s, CRC OK, decoded 102753 B/s, wire 145030 B/s.
Downloads/58-archive-00000018 (2).log independently matches every reference byte.
Final boot 58 uptime 808406, logger ready/synced, no errors/drops/suppression/
truncation, queue 0/16 high 1. USB idle/unpaused/resultok, no link losses in retry;
retained disconnect is historical. Memory lows 44272/31732 and writer margin 3320
unchanged. Current 234322 ->235229, writes 29 ->32, confirming continued logging.

Manual cancel, page-close and physical unplug recovery cases now pass.
Next: use the existing Read at 3-second intervals switch on current.log to
exercise the 120-second overall limit while reads continue; other switches off,
Live stopped, MQTT connected. Disable slow reads after terminal result (or about
150 seconds to drain buffered output), capture status, then ordinary retry and
status if cleanly finished. Browser buffering can delay the displayed terminal
reply; host receive time alone is not exact device timeout time. If the file
completes early, record that and do not count the timeout gate as exercised.
No firmware changes, build, flash, commit or push. Keep archive 18 for its later
long-progress test and eventual serial deletion.


### 2026-09-18 08:42: slow-read test hit stall, not overall timeout

Current requested at 08:42:43.643; stalled reply displayed at 08:45:20.096,
156.453 seconds later. Retained device evidence is definitive: elapsed 5121 ms,
idle 5000 ms, phase=data, path=stop_guard, reasonstalled, bytes 9072, loss 0 ms.
The pending 201-byte line had only 113 bytes of transmit space; no write attempted.
The host timestamp includes delayed draining of buffered data and must not be
interpreted as a late device deadline. Normal probe summaries arriving together
with the terminal reply also reflect buffering.

The page sleeps 3000 ms before each reader.read(), whose returned chunk size is
not fixed by this test. Such reading does not guarantee that enough driver TX
space becomes available for a complete protocol line within five seconds.
This attempt exercised current-file stall cleanup, not the intended 120-second
progressing-transfer timeout. The test method needs revision; do not repeat
it unchanged or increase production timeouts to accommodate the test.

JP unchecked slow reads after the failure. No automatic resume is expected:
finishError has closed the transfer and resumed appends; a new download request
is required. Status confirms USB inactive/unpaused/resultstalled, logger ready,
current 238946 bytes, writes 40, same boot 58. Queue 0/16 high 1, no errors/drops/
suppression/truncation; internal lows 44272/31732 and stack margin 3320 unchanged.

Next single step: all browser switches off, download current.log once and then
status. Preserve the resulting file for recovery and SD event timing inspection.
Do not reset/reconnect or remove the card. Overall deadline gate remains open;
use a more controlled progress test after this ordinary recovery check.
Documentation only; no firmware/page change, build, flash, commit or push.


### 2026-09-18 08:48: ordinary current download recovered; SD timing verified

With the test switches off, JP downloaded current.log successfully: 240805 bytes
in 2.33 s, decoded 103461 B/s, protocol wire 145428 B/s, CRC OK. The saved
Downloads/58-current.log is 240805 bytes with independently computed CRC32
980232E6. Boot 58 continued without a reset. USB finished inactive and unpaused,
result=ok. Current grew to 240968 bytes, and writes increased to 45.

Logger ready/synced, queue 0/16 high 1, zero errors, drops, suppression and
truncation. Internal heap minimum 44272 bytes, lowest largest block 31732 bytes,
and writer stack margin 3320 bytes are unchanged. No USB link losses were recorded.
The retained stalled failure at up_ms=1002873 is the earlier failed test;
it remains visible after successful downloads by design.

The downloaded SD records establish the earlier timeout timing directly:
USB_GET_BEGIN at 08:42:43.646, up_ms=997781; USB_GET_END at 08:42:48.745,
up_ms=1002880, bytes=9072, duration_ms=5128, result=stalled. The retained
5121 ms value was captured before final cleanup. Minute HEALTH records continued
from 08:43:08 onward, well before the browser displayed the buffered failure at
08:45:20. The board resumed logging after about five seconds, not 156 seconds.
The successful download includes its own BEGIN; its END occurs after the snapshot.

Current-file stall recovery passes. The 120-second progressing-transfer gate
remains untested. Next, correct the slow-read test method before asking JP to
repeat it; retain the production five-second stall and 120-second current limits.
Keep fixture archive 18 for the remaining long-transfer test and later serial
deletion. Documentation only; no firmware or page edits, build, flash, commit
or push.


### 2026-09-18: corrected page-only slow-progress test, awaiting JP

JP authorized correcting the test page. The old three-second sleep before an
arbitrarily sized read is replaced by a bounded test connection. Select Slow
reads before connecting: the page opens with bufferSize=256 and requests a BYOB
reader. Each read accepts at most 256 bytes. During a download, the subsequent
wait is actual bytes / 1024 seconds, at most 250 ms. There is no accumulated
credit after idle time. Console and file-list reads outside downloads are not paced.

Ordinary connections keep their default reader and original open options.
Firmware, baud rate, DTR and RTS values, and device timeout limits are unchanged.
Turning the switch off or cancelling removes pacing, including on the bounded
connection. Selecting it on an ordinary connection is refused with reconnect
instructions. Unsupported BYOB setup fails instead of silently using an unbounded
reader. The test tab must stay visible; browser scheduling and Windows buffers
remain hardware-test variables. A successful simulation is not a deadline gate pass.

API reference: [Chrome Web Serial documentation](https://developer.chrome.com/docs/capabilities/serial)
describes BYOB support since Chrome 106 and the bufferSize open option. JP uses
Chrome 152. No assumption is made that this also fixes the VS Code close issue.

Validation: all 19 checks in tools/tests/sd_log_browser.test.cjs pass. New cases
cover refusal when unarmed, bounded full and short reads with byte-based delays,
split-line CRC integrity, unchecking to drain, unchanged ordinary reads, and
partial-file rejection plus retry after a device timeout. No board accessed.

Next single bench case (no rebuild or flash):
1. Disconnect using the web page, leave the USB cable connected, then reload it.
2. Select only Slow reads before Connect. Use DTR=true and RTS=false. Confirm
   the console says the slow-read connection is armed, then wait for the file list.
3. With hotspot and MQTT connected and Live stopped, download current.log.
   Keep the tab visible. The desired terminal result is Device: timeout.
4. If no terminal result is visible after about 150 seconds, uncheck Slow reads
   to drain buffered output. Do not cancel first: preserve the device's reason.
5. With Slow reads off, send status and paste the whole test console. If it
   reports stalled or completes normally, the overall-deadline gate remains open.

Keep fixture 18 for the later archive-over-120-seconds case and serial deletion.
No firmware edit, build, flash, commit or push.


### 2026-09-18 09:01: bounded slow reads ended on connection indication

JP used the updated page: slow-read connection armed with 256-byte buffer and
BYOB reads, capped at 1024 wire B/s. Current requested at 09:01:09.903;
the browser sent log abort at 09:02:46.711 (96.808 s later) and received aborted
confirmation at 09:02:46.726. The page reported Missing response or END.

The retained firmware evidence explains which guard fired first:
- reason=disconnected, at_ms=2173235, elapsed_ms=69194, idle_ms=1554.
- loss_ms=1000, phase=data, path=stop_guard, bytes=56304.
- Last check connected_before_space, tx_free=-1, write_bytes=-1: the writer
  did not attempt that line's write after observing the disconnected indication.
- Later USB status inactive/unpaused, result=disconnected, bytes=56304.

The firmware had already ended the transfer after 69.194 seconds and resumed
appends. For disconnected it deliberately omits terminal output. Buffered data
can continue reaching the page, followed by its 15-second no-response watchdog;
the later abort acknowledgment does not change the original failure cause.
This is not the 120-second limit and not the five-second no-progress guard.
It records a core USB connection indication, not proof of cable removal or a
Wi-Fi disconnection. The cause of the sustained false indication is unverified.
The installed HWCDC implementation bases connection state on its SOF watchdog
and TX interrupt activity; it is not a direct application-readiness signal.

Board stayed on boot 58, uptime 2216954 ms at final status. Logger ready/synced,
queue 0/16 high 1, no errors, drops, suppression or truncation. Current increased
248084 -> 249551 bytes. Writer stack margin 3320 bytes; internal minimum 41836,
lowest largest block 28660, above the unchanged 20480 floor. The heap minimum
41836 was already present before this transfer; do not attribute its decrease
from the earlier session reading to this test. No reset or logger failure shown.

The revised page simulation passed, but the hardware deadline gate remains open.
Do not ask JP to repeat browser throttling unchanged. Recommend a temporary,
explicit serial test control under the existing USB fixture build flag that
paces successful data lines from the writer while the browser reads normally.
It should be off by default, remain responsive between lines without a blocking
sleep, and retain all connection, stall, queue and overall limits. This would
exercise the real current deadline without host receive-buffer pressure; it
would not claim to solve arbitrary slow-host behavior. A separate archive run
could continue beyond 120 seconds before restoring normal speed or cancelling.
This firmware test change is proposed only, not implemented or approved yet.
JP would build and flash once if accepted. Keep fixture archive 18.

Source capture: attachment 551c6eb9-9c8e-4d2f-b707-b7b3737eff9d/pasted-text.txt.
Documentation only this turn; no firmware/page edits, build, flash, commit or push.


### 2026-09-18: sender-paced deadline control prepared with JP's approval

JP approved implementing the firmware test and asked what the 120 seconds tests.
It tests the firmware's maximum current.log append pause, even when transfer
progress continues; it is not a browser deadline. The browser must read normally.

Added fixture-build-only `log test slow on` and `log test slow off`. On arms one
file request while idle. Data lines are then separated by at least 100 ms without
sleeping in the sender. Off works during transfer; ending the transfer clears the
active setting automatically. Listing preserves an unused arm. A queued file
request cancelled before start consumes the arm as well. Shutdown clears both.
No task, buffer or heap allocation added. Fault hooks stay off; normal transfers
retain their existing four-lines-per-turn policy. This is not the reverted Live
performance optimization. All device safety limits and checks remain in force.

Status gains the fixture-only LOG USB TEST line. The retained LOG USB FAIL
snapshot also captures reason=timeout so its elapsed time can be checked directly.
Browser test switches must all be off for the next run. Current at about 250 kB
exceeds the roughly 172800-byte maximum at 100 ms per 144-byte line over 120 s.
A progressing archive remains exempt from the overall limit, tested separately.

Validation: 16 source-level connection/pacing simulations pass, covering serial
arming/disarming, one-shot lifecycle, exact data spacing, current deadline,
archive exemption and existing connection/stall/queue/abort/shutdown guards.
All 19 browser checks pass. No firmware build, flash, commit or push performed.
Next single test is documented in src/diagnostics/STAGE1B.md: JP flashes, arms
slow sending, downloads current, checks timeout cleanup/status, then retries
normally without rearming. Gate remains pending hardware evidence.


### 2026-09-18 09:16: current-file overall deadline and recovery passed

JP flashed the sender-control build and reports a successful test, boot 60.
At 09:16:42, log test slow on returned slow_armed=1, slow_active=0 and a 100 ms
interval. Current requested at 09:16:50.340; Device: timeout received at
09:18:50.349. The retained device snapshot establishes the actual guard:
reason=timeout, elapsed_ms=120000, idle_ms=38, at_ms=162416, bytes=170352.
The last 203-byte wire line completed with tx_free=256 and write_bytes=203.
Data was still progressing, so this is the overall limit, not a stall.
Two transient link losses reached only 2 ms and did not abort the test.

After timeout, USB inactive/unpaused, result=timeout, both slow-test flags zero;
logger ready/synced, boot 60 unchanged. Queue 0/16 high 1, no errors, drops,
suppression or truncation. Internal minimum 95012 bytes, lowest largest block
53236, writer stack margin 3320. Normal retry without rearming downloaded
260449 bytes in 2.57 s, decoded 101275 B/s, wire 142385 B/s, CRC OK.
Final USB result=ok, inactive/unpaused, no link losses, slow flags still zero.
Memory readings unchanged, writes 10 -> 12, current 260288 -> 260611 bytes.
The retained timeout after retry is historical, as designed.

Independently inspected Downloads/60-current.log: 260449 bytes, CRC32 5806D2A2.
Its boot-60 USB_GET_END records result=timeout, bytes=170352, duration_ms=120006
including cleanup, followed by HEALTH at up_ms=162438. This confirms logging
resumed immediately after the timeout. The successful retry's own BEGIN is in
its snapshot; its END is written afterward. Use duration_ms and retained elapsed
for timing, not differences between queued record timestamps.

The current-file 120-second progressing-transfer and normal-retry gate passes.
Stage 1B as a whole is still pending. Next single case: prove archive downloads
can exceed 120 seconds without pausing appends. Keep browser switches off and
Live stopped. Send log test slow on, download archive 18, wait about 2 min 15 s,
then send log test slow off while it is still downloading. Let it finish at
normal speed and send status. Expect full 2097152 bytes with CRC OK, total time
above 120 s, result=ok, logger ready with no drops, and both slow flags zero.
Keep the downloaded fixture for independent byte verification. No new flash is
needed, and archive 18 stays on the card until subsequent serial deletion test.

Source capture: attachment d161b4c7-684e-4431-ae31-913140aadfab/pasted-text.txt.
Documentation only this turn; no firmware/page edits, build, flash, commit or push.


### 2026-09-18 09:23: archive beyond 120 seconds passed

JP reports normal operation, boot 60. Slow test armed at 09:22:56.779; archive 18
requested at 09:23:06.623. JP sent log test slow off at 09:25:22.318, after
135.695 seconds. At that point USB was active, appends were not paused, bytes
192672, and both slow flags became zero. Current had grown 262864 -> 264162 bytes
while the archive was being read. The previous result=ok in an active status is
historical; final DOWNLOAD and idle status establish completion.

Full archive completed at 09:25:41.450: 2097152 bytes in 154.83 s, CRC OK.
Reported average decoded rate 13545 B/s and wire rate 19118 B/s include the
intentional 100 ms pacing phase; they are not normal-speed throughput measures.
Independently checked Downloads/60-archive-00000018.log with
tools/verify_usb_fixture.py: every expected byte matches, CRC32 8D218D21,
SHA256 b79a649116ba358243b2c9388b68ac718b9f65cef94f241236ad8550394f65be.

Final uptime 579801 ms, same boot 60; USB inactive/unpaused, result=ok,
bytes=2097152, slow flags zero. Logger ready/synced, current 264336, writes 20,
queue 0/16 high 1, zero errors, drops, suppression and truncation. Internal
minimum 95012, lowest largest block 53236, writer margin 3320 unchanged.
Five transient link indications reached at most 5 ms; pending=0. The retained
current.log timeout remains the earlier successful timeout test, not a new fault.

Archive exemption from the 120-second overall limit passes, alongside the
current-file deadline and recovery case. Large-file duration/integrity tests
using archive 18 are complete. Next single test is the already implemented
serial deletion: log test del 18, wait for result=deleted (roughly a minute
for full-pattern verification), Refresh files, then status. Confirm only archive
18 is removed and current plus archives 14, 15 and 16 remain. JP keeps the
verified downloaded copy. No card removal or new flash is needed.

Queue-pressure and selected-archive pruning gates remain separate outstanding
work; any sacrificial file needed for those should be prepared explicitly, not
by risking retained diagnostic archives. Stage 1B is not fully accepted yet.
Source capture: attachment 6782d3d8-c279-4511-ad92-db38a3acbe65/pasted-text.txt.
Documentation only; no firmware/page changes, build, flash, commit or push.


### 2026-09-18 09:30: fixture deletion over USB passed; normal default restored

JP agreed to retain test source but exclude it from normal builds. JP sent
log test del 18 at 09:29:39.843. Full-pattern verification and deletion completed
at 09:30:32.354 (52.511 s host interval): result=deleted, archive=00000018,
bytes=2097152, errno=0. Refresh listed four managed files instead of five;
newest returned to 16 and archives to three. Current.log remained and grew to
267861 bytes. JP reports the file was deleted successfully. The pasted summary
does not enumerate the individual remaining names, but its counts and newest
number agree with removal of the synthetic archive only.

Final boot 60 uptime 920256 ms: logger ready/synced, queue 0/16 high 1, no errors,
drops, suppression or truncation. Internal minimum 95012, lowest largest block
53236, stack margin 3320 unchanged. USB inactive/unpaused; slow flags zero.
Available space increased from 15920332800 to 15922429952 bytes: exactly
2097152 bytes recovered. The brief active=1 during Refresh was listing activity;
subsequent status is inactive. Retained timeout still refers to the intentional
current-file test at up_ms=162416, not a deletion failure.

Set the source default DIAG_USB_TEST_FIXTURE to 0 as agreed. This excludes the
fixture generator, safe deletion and sender-speed commands from normal builds;
source remains available for future bench testing. DIAG_TEST_HOOKS stays 0,
PSRAM writer stays enabled. Real USB retrieval and all production safety limits
remain enabled. JP's boot 60 still has fixture=1 until he flashes a newer build.
No need to flash only for this flag yet; prepare the remaining focused gate tests
before the next bench build. Queue pressure and selected-archive pruning remain
open, along with final evidence review and owner acceptance. Preserve real
archives when preparing any future sacrificial test file.

No build, flash, commit or push. This turn changed only the fixture default and
documentation; deletion was performed by JP through the existing serial command.


### 2026-09-18: normal-build and commit checkpoint

JP requested committing and pushing this checkpoint and asked to compile/flash
with the temporary tests disabled. Confirmed source defaults: USB fixture=0,
fault hooks=0, PSRAM writer=1; selected VS Code profile is
amoled-1-8-core-3-3-11. No matching test-flag overrides were found in sketch.yaml
or .vscode. Normal logging, USB retrieval and safety deadlines remain enabled.
JP may build/flash this normal configuration now, with all browser test switches
off, then send status and download current.log once to verify the configuration.
The normal status should no longer include LOG FIXTURE or LOG USB TEST output.
companion.ino has not changed; no generated-sketch deletion is required here.

All 35 local checks passed again (16 firmware-source simulations, 19 browser
checks); git diff --check clean. This is not a firmware compilation. No build
or flash by Codex. Stage 1B remains pending its queue/pruning checks and final
review. The separate owner document docs/sd_iphone_log_download_plan.md is outside
this commit's scope and remains untouched.


### 2026-09-18 09:43: normal build with USB fixture disabled passed

JP compiled/flashed the normal configuration from checkpoint ad01fb8 and supplied
status, current download and status, boot 62. hooks=0, PSRAM stack placement valid,
writer active on core 1. LOG FIXTURE and LOG USB TEST lines are absent from both
complete status replies, consistent with DIAG_USB_TEST_FIXTURE=0. The ordinary
MQTT [TEST] ON status remains intentional and is not the disabled USB fixture.

Current downloaded 276169 bytes in 2.81 s, decoded 98386 B/s, wire 138343 B/s,
CRC OK. Saved Downloads/62-current.log independently measured 276169 bytes,
CRC32 0685E3ED. Earlier Downloads/60-current.log was no longer at that location,
so no independent prefix comparison was performed in this turn.

Final USB inactive/unpaused, result=ok, bytes=276169. Logger ready/synced,
queue 0/16 high 1, zero drops, suppression, truncation and errors; no retained
USB failure or link losses. Same boot and advancing uptime, current grew
276010 -> 276329 bytes and writes 7 -> 9. Three archives remain, newest 16;
fixture 18 remains absent by the reported inventory. Internal minimum 92572
and lowest largest block 49140 stayed unchanged across this download; the
largest block is above the unchanged 20480 floor. Writer margin after first
transfer is 3400 bytes (4792 used of 8192), versus 3752 before it. This is the
expected deeper exercised path, not evidence of a leak. A short normal IMU
window reports 42.97 Hz average, 31.67 minimum; no fresh performance A/B claim.

Normal-build status/download smoke check passes. Stage 1B still needs focused
queue-pressure and selected-archive pruning coverage plus final evidence review
and JP acceptance. Next recommendation is to prepare those focused tests for a
separate bench build, preserving this normal configuration as the checkpoint.
No further test is required from JP on this normal build right now.

Source capture: attachment aca39999-450d-4a7d-8234-2bd648c9e59c/pasted-text.txt.
Documentation only; no firmware/page edit, build, flash, commit or push this turn.


## 2026-09-18: queue-pressure and selected-archive gate controls prepared

JP approved preparing the remaining focused Stage 1B controls after boot 62's
normal-build smoke passed. No new hardware result is claimed here.

- `log test queue` arms one current-file transfer. After at least 1440 bytes,
  the writer adds bounded USB_QUEUE_TEST events to the real PSRAM ring until
  half its slots are occupied. Existing entries and reserved capacity remain.
  The production logger_busy guard must close the download and resume appends.
- `log test prune N` accepts only a complete 2 MiB synthetic fixture created
  this boot. After transfer progress it rechecks ownership/type/size and invokes
  the same close-reader-before-unlink helper used by normal pruning. It leaves
  retention limits and real archives alone. This tests reader removal, not
  retention threshold selection; Stage 1 supplies the separate pruning evidence.
- Both controls are one-shot, report `[LOG USB GATE]` in status, and compile out
  with DIAG_USB_TEST_FIXTURE=0, which remains the source default. Fault hooks
  remain off; PSRAM writer and all guards/probes are unchanged.
- 12 logger-gate source simulations, 16 USB guard/pacing simulations and
  19 browser checks pass. These do not establish hardware behavior or compilation.

Next: JP builds the fixture-enabled bench configuration and runs only the queue
procedure in [STAGE1B.md](../src/diagnostics/STAGE1B.md). Review console and
retrieved records before the disposable-archive test. No firmware build, flash,
commit or push performed by the assistant. Stage 1B acceptance remains open.


## 2026-09-19: USB-powered startup delay, before queue-pressure test

JP reports a blank screen after plugging in USB, then Connecting with a frozen
spinner after about 30 seconds. Opening the web console allows normal startup
and MQTT connection. Queue-pressure testing is paused; this report is not a
queue-test result. The current local fixture flag is 1; fault hooks remain 0.

Source inspection found a plausible cause: the installed core 3.3.11 HWCDC.cpp
uses a 100 ms TX wait, with 20 consecutive failed ring-buffer sends before
returning a short write. USB can remain connected while no application reads.
Each blocked debug write can therefore wait about two seconds. The sketch
prints repeatedly before initializing the display and during Wi-Fi startup.
This matches the symptoms; hardware confirmation is still pending.

A minimal candidate fix sets USBSerial.setTxTimeoutMs(1) after begin and before
first sketch output, for core 3.3.11 and later. The installed SDK has a 1000 Hz
FreeRTOS tick. Its consecutive no-progress waits now total about 20 ms per
write, rather than two seconds. This is not a nonblocking serial redesign or a
promise about other SDK versions. The 3.1.3 rollback settings are unchanged.
Debug text can be dropped or shortened when the host does not drain output.
The USB file sender still checks space and returned byte counts; its five-second
stall, one-second disconnect grace and current-only 120-second limit are intact.
No core files, USB signals, SD settings, task placement or probe logic changed.

Next bench check: hotspot on, VS Code monitor and web connection closed; boot
from USB and verify the display, connection and touch work without opening a
serial console. Then connect the web page with DTR=true and RTS=false, send
status, download current.log normally and send status again. Compare uptime and
report startup delay, any freezes, console output and CRC result. Do not arm
queue or pruning controls yet. The older VS Code monitor-close freeze remains
an unresolved issue; this patch does not establish its cause or resolution.

JP builds and flashes. No assistant build, flash, commit or push.

Validation: git diff --check is clean. The 16 USB guard/pacing source simulations
and 19 browser protocol checks pass. They do not simulate the hardware TX driver
or prove this startup fix. Removed only the active profile's generated
build/build_amoled-1-8-core-3-3-11/sketch/companion.ino.cpp for JP's next rebuild.
Preserved JP's existing DIAG_USB_TEST_FIXTURE=1 edit and the untracked iPhone plan.


## 2026-09-19 09:11: USB startup fix and ordinary retrieval pass (boot 67)

JP compiled and flashed the TX-wait change, then reports that the board woke
normally on USB insertion without first opening the web console. The requested
startup/download check passed visually. No startup duration was measured.

- Same boot 67 before and after retrieval; uptime advances from 27273 to
  41120 ms. Wi-Fi and MQTT connected, clock synced, logger ready.
- Downloaded current.log: 292363 bytes in 2.85 s, browser CRC OK, decoded
  102656 B/s. This is browser integrity evidence, not a card-reader prefix check.
- File grows from 292204 to 292523 bytes; writes rise from 7 to 9.
  Final USB active=0, paused=0, bytes=292363, result=ok. No USB failure or
  link loss reported. Queue empty, high-water 1, drops=0 and error=none.
- Reported internal low-water 95164 bytes and largest-block minimum 51188
  remain unchanged. Writer PSRAM stack margin changes from 3720 to 3368
  bytes after download; placement valid, core 1. No TLS memory gate was run.
- Normal probe covers 15322 ms, IMU average 42.18 Hz, minimum 30.02 Hz.
  This short window does not replace the pending controlled IMU comparison.
- Fixture enabled, fault hooks off, queue/prune controls unarmed and unfired.
  Sender slow mode off. Transfer limits remain 120000 ms current-only,
  5000 ms stalled and 1000 ms disconnect grace.

This passes the targeted no-console startup and normal retrieval smoke check.
It supports the serial-backpressure explanation but does not isolate every USB
signal behavior or resolve the separate VS Code monitor-close freeze. Resume
only the queue-pressure test next; Stage 1B acceptance remains pending.

Source: attachment 81b7e1f4-aa97-4624-9cb0-c1d9dd61d90d/Pasted text.txt and JP's
visual report. JP requested commit and push. Keep the temporary fixture=1 edit
local for the next bench test; committed normal-build default remains 0.


## 2026-09-19 09:14: Stage 1B queue-pressure gate passes (boot 67)

JP reports normal operation. Profile remains amoled-1-8-core-3-3-11 with the
startup TX-wait fix, fixture=1, fault hooks=0 and the PSRAM writer on core 1.

- At 09:14:37 log test queue arms the next current download. The hook adds
  eight real events and records queued_at_test=8/16, fired=1, result=injected.
  Production protection returns logger_busy after 1584 bytes. Final gate
  outcome is logger_busy; both armed and active clear.
- Logger resumes ready with queue=0/16, high=9, drops=0 and error=none.
  The ninth queued entry is consistent with USB_GET_END after the eight test
  records. File grows from 294212 to 295628 bytes; writes rise from 12 to 22.
- Retry without rearming succeeds: current.log 296353 bytes in 2.85 s,
  browser CRC OK, decoded 104141 B/s. Final active=0, paused=0, result=ok;
  file_bytes=296515 and writes=25. Same boot, no USB link losses or failure.
- Reported internal minimum 41156, largest-block minimum 28660 and writer
  stack margin 3368 bytes stay unchanged across this test. These are retained
  minima, not a current-free-memory leak test. They were already lower before
  arming than in the earlier startup smoke capture.
- The opening 60-second normal probe averages 42.90 Hz with a 6.41 Hz minimum,
  before queue arming. The cause of that isolated slow interval is unverified;
  retain it for the planned controlled IMU comparison, not a queue regression.

Read the saved C:/Users/photo/Downloads/67-current (1).log directly: 296353
bytes, computed CRC32 F4B06442. Boot 67 sequences 14-21 contain exactly eight
USB_QUEUE_TEST source=bench synthetic=true records. Sequence 22 is USB_GET_END
bytes=1584 duration_ms=69 result=logger_busy. Sequence 23 is a later HEALTH
record with drops=0; sequence 24 starts the successful retry. The retry's own
end record falls after its snapshot and is instead confirmed by serial status.
This is saved-file content verification, not the still-pending card-reader
current-file prefix comparison. The file also records SETUP_COMPLETE at
11592 ms for boot 67, adding a device startup measurement to JP's visual report.

Queue-pressure gate passes. Next: selected synthetic-archive pruning, on the
same running build. No reflash or card removal needed. Stage 1B acceptance,
the agreed documentation reconciliation and bounded SDK regression remain open.
Source console: attachment 08a9bcd9-dc3a-459e-9c46-3b84d1599b4f/Pasted text.txt.
Documentation updated only; no firmware edit, build, flash, commit or push.


## 2026-09-19 09:17: selected-archive pruning gate passes (boot 67)

JP reports normal operation. A new synthetic archive 00000018 was created in
this boot: 2097152 bytes, result=ok at 09:18:39. Creation took about 52.6 s.
It is distinct from the old archive 18 deleted on September 18.

log test prune 18 armed the matching transfer. At 09:19:45 the production
close-reader-before-unlink path returned reason=pruned; gate fired=1,
result=pruned and outcome=pruned. Managed count returned from five to four,
newest from 18 to 16, archive count to three, and pruned rose from zero to one.
The fixture reports pruned_by_test. Its roughly 2 MiB of space was reclaimed.
The original current file and archive count were preserved; the full console
list does not print every surviving filename.

Ordinary current download then succeeded: 300115 bytes in 3.03 s, browser CRC
OK, decoded 99005 B/s. Read Downloads/67-current (2).log directly: 300115 bytes,
CRC32 BCC256A0. Boot 67 seq 29 records fixture creation, seq 31 begins its
retrieval, seq 32 ends it with bytes=1728 duration_ms=61 result=pruned,
and seq 33 records USB_PRUNE_TEST. The following HEALTH shows archives=3,
pruned=1 and drops=0; seq 35 begins the successful current retry. It also
contains the prior queue retry's successful END (seq 25).

Final status: same boot, logger ready, queue=0/16, high=9 from the prior queue
test, zero drops/errors, USB inactive/unpaused/result=ok. File grew to 300277.
Retained internal minimum 41156, largest minimum 28660, writer stack margin
3368 bytes unchanged. One 2 ms connection indication recovered within the
1000 ms grace; no retained USB failure. This is not an observed cable outage.

Selected-reader pruning gate passes. Fixture is already removed; no delete
command or card removal is required. Source: attachment
c93603a9-33dd-4f67-abd5-9c973e20b56b/Pasted text.txt and saved download above.

## 2026-09-19: evidence reconciliation before the bounded SDK regression

Historical correction: JP's September 17 boot-45 capture, before the 3.3.11
upgrade, already reported writer_core=1. At 11:33:51.328 the normal probe was
ms=60006, samples=6000, imu_n=2910, imu_min_hz=27.03, imu_avg_hz=48.63.
It used core 3.1.3. This is not a same-session comparison with the later 42-43 Hz
on 3.3.11, but moving the writer to core 1 alone does not explain the drop.
Source is JP's inline 11:33/11:34 capture in this conversation. Driver, graphics,
scheduling and logging costs remain unisolated. Run the agreed three-way test;
compare writer cores only if the logging-on leg explains the slowdown.

### Stage 1B evidence by gate (2026-09-19)

| Gate | Evidence and remaining work |
|---|---|
| 1 Integrity | Archive 14 matches its card-reader copy (8059 bytes, CRC 99C24CB1); 2 MiB synthetic content independently verified. Current integrity now also passes: September 19 boot-74 USB download (130783 bytes, CRC 9E5F4441) exactly matches the physical card file prefix. Later records include clean shutdown; see the 10:02 entry. |
| 2 Throughput | September 17 2 MiB download: 20.39 s; 3x is 61.17 s, below unchanged 120 s. Current plus three archives completed at 14:37. |
| 3 Non-interference | Same-session Live and MQTT overlap measured. JP accepts only the roughly 8% large-download Live FPS exception; pacing reverted. Frame gaps and visual reports available; no separate cause-tagged main-loop gap measurement yet. Bounded storage/close/NVS and resource regression now pass. Post-checkpoint review accepts existing 3.3.11 TLS/memory evidence against 20480 bytes. Same-sitting logging-off/on comparison passes September 19 at 21:25 (3.4679 -> 3.6414 FPS). Small-bundle overlap after a675a76 passes at 21:29, same boot 82: 3.5220 FPS, 3.28% loss, four CRC successes, zero drops/errors and memory above the gate. Ready for explicit JP acceptance; see the current checkpoint. JP accepts the measured IMU profile difference. |
| 4 Debug and rejection | September 17 damaged-line rejection and clean retry passed. Ordinary MQTT attempt BEGIN at 15:55:50.881 appears during the download ending 15:55:54.270. This proves visibility in that run, not lossless debug output under backpressure. |
| 5 Abort/liveness | Explicit abort, page close, battery unplug, 5000 ms stall and recovery passed. September 18 sender-paced current reached elapsed_ms=120000, then normal retry passed; progressing archive completed in 154.83 s. Browser slow reads hit other guards, so JP approved sender pacing for the deadline case. |
| 6 Current pause | September 19 boot 67: eight queued events triggered logger_busy at 8/16; all eight saved, zero drops, appends resumed and retry passed. |
| 7 Pruning | September 19 boot 67: new synthetic archive 18 reader closed and archive removed, pruned=1; current retry passed. Stage 1 separately covered retention selection on 3.1.3. |
| 8 Resources | Passed September 19, boot 74: five deliberate aborts and five CRC-checked retries. Matching idle HEALTH readings stayed at 105488 bytes free internal memory and 57332 bytes largest block; PSRAM and writer stack margin unchanged. No handle counter; repeated successful opens exercise the three-handle limit. See the 09:56-09:58 entry. |
| 9 Round trip | Web DTR=true/RTS=false reconnects work. VS Code monitor close can freeze/reset the board and predates SD logging (Step 0). It remains an explicit limitation. September 19 no-console startup fix passed; monitor-close resolution was not tested. |

JP's agreed order now proceeds to a bounded 3.3.11 regression: storage recovery,
shutdown and deep-sleep close, NVS/SD overlap; resource series; current-file
card-copy prefix; three-way same-sitting IMU comparison, three normal 60-second
windows each (3.3.11 logging off, 3.3.11 on, 3.1.3 off). Reuse existing passed
checks. Keep the 20480 gate, transfer limits and narrow Live exception. Restore
fixture=0 and hooks=0 for normal use. JP accepts Stage 1B before Stage 2.
Documentation and a config comment only; no behavior, flag, build, flash,
commit or push changes in this turn.


## 2026-09-19 09:29: interrupted rename recovery, archive verification pending

JP reports the test passed visually. Boot 69 ran hooks=1, fixture=0, PSRAM
writer=1 on core 1. log test rename was queued at 09:29:59.271, and the writer
confirmed its requested boundary pause at 09:29:59.326. USB disappeared during
the requested power cycle; boot 70 subsequently recovered to ready.

Generation advanced 17 -> 18, newest archive 16 -> 17, archives 3 -> 4.
The new current file had 1259 bytes at uptime 43184 ms. Download of current
succeeded: 1418 bytes, 0.06 s, browser CRC OK. Inspected Downloads/70-current.log:
FILE_OPEN format=1 generation=18 reason=new, BOOT reset=power_on, LOGGER_START
mount=ok and SETUP_COMPLETE result=ready. CLOCK_SYNC and Montreal -0400 offset
follow. Computed CRC32: F85E81B5.

After about 70 seconds, current grew to 2135 bytes, writes 8 -> 11. Same boot
70, ready/synced, queue empty, high=1, zero drops/errors, USB inactive/unpaused,
result=ok, no USB losses. Retained internal minimum 92476 and largest minimum
51188 bytes; PSRAM writer stack margin 3304 after retrieval. rotations=0 is a
new-boot counter and does not contradict recovery of the prior boot's rename.

The supplied console and Downloads contain the current download but no new
archive-00000017.log download. Recovery and continued append pass; preserved
archive bytes remain to be checked before closing this case. A pre-test backup
exists as Downloads/67-current (3).log (303257 bytes), alongside archives 14-16.
Next action only: download archive 17 with log get 17, then send status. Compare
its prefix to the pre-test backup and inspect its final pre-rotation records.
No reflash, restart, card removal or new fault trigger needed.

Source: attachment fc6c08c2-89bb-43d5-a4b1-863c6fc46676/Pasted text.txt and
Downloads/70-current.log. Documentation only; no build, flash, commit or push.

## 2026-09-19 09:34: renamed archive verified; recovery case passes

Archive 17 downloaded successfully: 308745 bytes in 2.99 s, browser CRC OK.
Read Downloads/70-archive-00000017.log: computed CRC32 541A8F0C. Its first
303257 bytes exactly match Downloads/67-current (3).log, the pre-test backup.
Later records include boot 69's setup and the deliberate TEST_HOOK at uptime
56456 ms, immediately before the archived file's end. Thus the known pre-test
contents survived the rename; the newly created generation-18 current file was
already verified in the preceding entry. This completes this recovery case.
This USB-to-USB comparison does not replace the separately agreed current-file
prefix comparison against a physical card-reader copy.

Boot 70 remains ready/synced with current growing to 3600 bytes, writes=15,
queue=0/16 high=1, drops=0 and error=none. USB inactive/unpaused, result=ok,
bytes=308745, no link losses or retained failure. Historical internal minimum
92476, largest minimum 51188 and writer stack margin 3304 bytes unchanged.
Source: JP's inline 09:34 console and the two saved files above.

Next single recovery case: a deliberately partial current-file header. Keep
this build (fixture=0, hooks=1, PSRAM writer=1), normal retention, hotspot on,
Live stopped and browser test switches off. Download current first as backup;
send log test partial and wait for the boundary pause, then power fully off
and restart. Refresh/status, download both newly added archives and current,
then wait 70 seconds and status. Expected: one archive preserves the old current,
one preserves the deliberately incomplete header, a valid new current grows,
and logger remains ready without drops/errors. The power-down close path may
append shutdown records after the deliberate fragment; preserve that evidence.
No rebuild or card removal required. No firmware edits, commit or push.

## 2026-09-19 09:37: partial-header salvage passes (boot 71)

JP reports normal operation. The console begins after restart, but the saved
files directly establish the injected partial header and recovery:

- Downloads/71-archive-00000018.log: 5177 bytes, CRC32 8E2CF931. Its first
  4323 bytes exactly match Downloads/70-current (1).log, the pre-test backup.
  Later records end with the deliberate TEST_HOOK in boot 70 at up_ms=314608.
- Downloads/71-archive-00000019.log: 153 bytes, CRC32 A7DA7C08. Starts with
  the exact incomplete fragment local=unknown time=unk, immediately followed
  by SESSION_END reason=shutdown pending=0 from boot 70 at up_ms=357326.
  This malformed concatenation is the expected injected artifact; it was
  preserved rather than appended to as a valid current header after restart.
- Downloads/71-current.log: 2131 bytes, CRC32 D528B520. Starts with FILE_OPEN
  generation=20 reason=corrupt_header, then BOOT context=boot_after_salvage
  reset=power_on. Mount and setup succeed; Montreal time synchronizes.
  Browser CRC checks pass for all three downloads.

Final boot-71 status: ready/synced, generation=20, newest=19, six archives;
current grows 1303 -> 3410 bytes, writes 8 -> 16. Queue empty, high=1,
drops=0 and error=none; USB inactive/unpaused/result=ok and no link losses.
Retained internal minimum 92476 and largest minimum 51188 bytes; PSRAM
writer core 1, stack margin 3304 after downloads. Full normal 60-second probe
averages 43.24 Hz, minimum 25.64 Hz; not a controlled IMU comparison.

Interrupted-rename and partial-header recovery now pass on the 3.3.11 trial.
No extra recovery repetition requested. The fragment's shutdown record proves
the paused-writer close path ran, but an ordinary active-writer close remains
the next bounded regression case. The physical card-reader prefix check is
still separate from these USB-download comparisons.

Next test, same build, no new hook: with hotspot on and Live stopped, send
status and download current as backup. Unplug USB on the battery-equipped
board, leave it stationary without touching it, and wait up to two minutes
for Shutdown and screen off. Do not use the power button to force this step.
After it switches off, reconnect USB, connect the web console, send status,
download current, wait 70 seconds and send status again. Check the saved log
for the prior boot's SESSION_END reason=shutdown pending=0, then new boot
append and continued health records. Report any failure to shut down rather
than treating a forced power-off as a pass. Deep-sleep close is a later case.

Source: attachment a556c851-1cc3-4492-b82f-444ccda5715c/Pasted text.txt and
Downloads files above. Documentation only; no build, flash, commit or push.

## 2026-09-19 09:41: ordinary shutdown close and restart pass (boots 71-72)

JP reports the requested automatic-shutdown test passed. Browser loses USB at
09:41:37.743 after unplug. Downloads/72-current.log contains boot 71 seq 21:
SESSION_END reason=shutdown pending=0 at 09:42:08.588 (up_ms=301001).
This is about 31 seconds after cable removal and consistent with the existing
30-second USB-loss grace once other inactivity conditions are satisfied.
The next BOOT is boot 72, reset=power_on, context=append, not a watchdog reset.

The downloaded 6293-byte file (0.09 s, browser CRC OK) has computed CRC32
29CD556E. Its first 4694 bytes exactly match Downloads/71-current (1).log,
the pre-shutdown download. The prior successful USB_GET_END, shutdown record,
new BOOT and successful mount/setup/sync follow in the same file. No extra
FILE_OPEN is inserted: generation 20 and six archives/newest 19 are retained.
This comparison verifies append preservation, not the pending card-reader check.

Boot 72 final current grows 6134 -> 7008 bytes and writes 7 -> 10. Logger
ready/synced, queue=0/16 high=1, zero drops/errors, USB inactive/unpaused/result=ok,
no link losses or retained failure. Internal minimum 95036, largest minimum
51188, writer core 1 with valid PSRAM placement and stack margin 3304 bytes.
Normal 60-second probe averages 42.94 Hz, minimum 29.40 Hz. Hooks=1, fixture=0.
No normal shutdown close deadline is measured by this capture; the persisted
SESSION_END and successful append establish the tested close/restart behavior.

Next single case: deep-sleep close and touch wake on this same build. Hotspot
on, unplug USB and let the stationary board shut down; start with the power
button on battery only. After connection tap the dashboard once, then gently
tilt every 10-15 seconds without touching the screen to keep motion active.
After roughly 60 seconds without touch expect Sleeping and screen off. Stop
moving, wait five seconds, touch once to wake BEFORE reconnecting USB. After
normal operation returns, reconnect USB/web, status, download current, wait
70 seconds and status. Report observed sleep and touch-only wake. If no sleep
within two minutes or touch fails, stop and report rather than substituting a
power-button/USB wake. Fresh battery-only startup is needed because a boot
that has seen USB uses the different shutdown/grace branch. No rebuild needed.

Source: attachment 393ed93f-7fd5-49bf-bf4e-a811d12005f4/Pasted text.txt and
saved Downloads/71-current (1).log and 72-current.log. Documentation only;
no firmware edit, build, flash, commit or push.

## 2026-09-19 09:47: deep-sleep close and wake pass (boots 73-74)

JP reports the requested test passed. Downloads/74-current.log directly records:

- Boot 72 SESSION_END reason=shutdown pending=0 at 09:45:58.992, followed by
  a fresh battery-only power-on boot 73.
- Boot 73 SESSION_END reason=deep_sleep pending=0 at 09:47:21.717,
  up_ms=76563. Boot 74 follows with reset=deep_sleep, reset_code=8, wake_code=2.
  The firmware enables EXT0 on TP_INT for touch wake. Boot 74's timing precedes
  the browser's USB-available event at 09:47:53.675, consistent with waking
  before USB was reconnected, rather than rebooting on cable insertion.
- Valid retained breadcrumbs identify prior boot 73: main phase=sleep,
  writer phase=sd_close. Initial time quality is approx, later synced with
  prior_known=1 and correction_ms=-99. No crash reset in this sleep/wake cycle.

Downloaded current: 11491 bytes in 0.17 s, browser CRC OK, computed CRC32
C940757D. Its first 6293 bytes exactly match Downloads/72-current.log. This
verifies preserved append history across shutdown, battery boot and sleep wake.
Generation remains 20, newest=19, six archives. No card-reader comparison yet.

Final boot 74 status: ready/synced, current 11332 -> 12207 bytes, writes 7 -> 10,
queue empty, high=1, drops=0 and error=none. USB inactive/unpaused/result=ok,
no link losses or retained failure. PSRAM writer placement valid, core 1,
stack margin 3304 bytes; internal minimum 92500, largest minimum 53236.
Normal 60-second probe averages 43.24 Hz with minimum 22.87 Hz. These values
are evidence for this run, not a paired performance comparison.

Both ordinary shutdown and deep-sleep close/restart cases now pass on 3.3.11.
Next single case: the existing 30-second NVS/SD overlap test on this same
hooks=1, fixture=0, PSRAM=1 build. Keep USB, hotspot and real MQTT connected,
Live stopped, dashboard selected and browser test switches off. Send status,
then log test nvs once. Leave the unit alone until NVS stress ended; wait two
seconds, then status. Require inactive, summary_pending=0, key_removed=1,
nvs_errors=0, positive overlapping NVS/SD counts, zero logger drops/errors and
adequate stack. Download current after completion, then status, for saved
TEST_NVS_START/SD/END verification. Stop on a reset, error or pending summary.
No rebuild or other fault trigger needed. The writer does not call NVS; this
checks interleaved main-task flash commits and PSRAM-stack writer SD activity.

Source: attachment 8f43a693-4d08-4a71-9ac2-deb4bf783eff/Pasted text.txt,
Downloads/74-current.log, Downloads/72-current.log and JP's pass report.
Documentation only; no build, flash, commit or push.

## 2026-09-19 09:51: NVS/SD overlap passes on 3.3.11 (boot 74)

JP reports normal operation. The early status was taken during the active test;
the later status and saved file establish completion, so no repeat is needed.
The two early downloads returned unavailable by design: usbStatus excludes
active NVS stress and a pending final summary from retrieval readiness.

- Stress ran for 30 seconds: 278 successful NVS commits, zero NVS errors,
  736 SD test records, key_removed=1. The 300 commits are a maximum, not a
  required count. Final nvs_active=0 and summary_pending=0.
- NVS completion times span 225275-255229 ms; SD completion times span
  225308-255280 ms. The saved file contains one TEST_NVS_START, exactly 736
  TEST_NVS_SD records and one TEST_NVS_END with the matching final counts.
- Downloads/74-current (1).log is 122914 bytes, browser CRC OK, independently
  computed CRC32 D5321915. Its prefix exactly matches all 11491 bytes of
  Downloads/74-current.log. This is not the pending physical-card comparison.
- Post-test logger ready/synced, queue=0/16 high=1, drops=0, error=none.
  Internal minimum 92500 and largest minimum 53236 bytes; writer stack margin
  3304 bytes, valid PSRAM placement, core 1. No reset; boot remains 74.
  Writes=753, write_max_us=52978, flush_max_us=11960; slow=0.
  USB inactive/unpaused, result=ok, no retained failure or link losses.
- The stress-containing normal probe has gap_max_us=62770 and imu_min_hz=11.18.
  Deliberate flash stress affects this window; do not use it for the ordinary
  three-way IMU comparison or claim unchanged sampling under flash writes.

This verifies the bounded main-task NVS/PSRAM-stack writer overlap case.
Storage recovery and both close paths also have passing new-SDK evidence.
No firmware changes, build, flash, commit or push in this review.
Source: attachment 72b75716-06be-45d5-a386-20ccf399ebef/Pasted text.txt,
Downloads/74-current (1).log, prior Downloads/74-current.log and JP's report.

Next single test: resource stability, same boot/build, hooks=1 but no active
fault hook, fixture=0. Keep USB/hotspot/MQTT connected, Live stopped and the
same dashboard screen. Leave ordinary browser switches off. Wait 70 seconds,
send status and download current as a baseline. Run five pairs: enable Damage
one data line, download current and wait for abort confirmed; then download
current normally and require CRC OK. The damage checkbox clears itself for
the retry. After the fifth pair wait another 70 seconds, send status and
save one final current download. Send the full console and keep all saved
files. These are five deliberate aborts and five successful retries, plus
baseline/final evidence downloads. Stop on an unexpected error or reset.
Compare current internal_free/internal_largest in idle HEALTH records before
and after, not only the retained minima in serial status. Also compare writer
stack, drops, errors and successful repeated file opens. There is no open-handle
counter; repeated opens exercise the configured three-handle limit. No reflash.

## 2026-09-19 09:56-09:58: resource series passes (boot 74)

JP reports normal operation. The console contains five damaged-line aborts,
each confirmed by the device, and five successful ordinary retries with CRC OK.
The saved log records all five USB_GET_END result=aborted entries, with 288-1152
bytes sent and 38-50 ms durations, followed by successful retries. Baseline and
final evidence downloads also succeeded. No restart; all results are boot 74.

Matching idle HEALTH records before and after the series (09:55:35.296 and
09:58:35.373) have screen=1, operation=idle, moving=0, Wi-Fi/MQTT connected:

| Measurement | Before | After |
|---|---:|---:|
| Current free internal memory | 105488 | 105488 |
| Current largest internal block | 57332 | 57332 |
| Current free PSRAM | 8339860 | 8339860 |
| Writer minimum stack margin | 3304 | 3304 |
| Boot internal minimum | 92500 | 92500 |
| Queue high water / drops | 1 / 0 | 1 / 0 |

Intermediate idle HEALTH records agree. No accumulating memory loss observed
in this bounded series. Status remains ready/synced, error=none, zero drops,
USB inactive/unpaused/result=ok, and no retained failure or link loss. File
writes increase 757 -> 782; current grows 125344 -> 130621 before the final
download. The repeated successful opens exercise the configured three-handle
limit; there is no direct open-handle count.

All seven saved downloads preserve the preceding file as an exact byte prefix.
Baseline Downloads/74-current (2).log: 125506 bytes, CRC32 AF0DEA59.
Final Downloads/74-current (8).log: 130783 bytes, CRC32 9E5F4441.
Five successful retries: 126723, 127373, 128023, 128674 and 129891 bytes;
1.22-1.33 seconds each. Test build remains hooks=1 with stress inactive,
fixture=0, PSRAM writer=1, core 3.3.11 and writer core 1.

Next single case: physical-card prefix comparison. Reuse the final verified
USB download above; no need to download again. Disconnect USB and leave the
battery board stationary until its normal shutdown finishes. With power off,
remove the card and make it available to the computer as before (E:/sdcard,
or report the actual path). Read-only compare the first 130783 bytes of the
card's logs/current.log to Downloads/74-current (8).log; later appended health
and shutdown records are expected. Preserve both files. This is the one
remaining test before the agreed three-way IMU comparison, in one sitting.
No firmware change, build, flash, commit or push.

Source: attachment 55da2776-988a-4352-a093-be2a85445a5a/Pasted text.txt,
Downloads/74-current (2).log through 74-current (8).log, and JP's pass report.

## 2026-09-19 10:02: physical-card current-file prefix passes

Read E:/sdcard/logs/current.log without modifying the card. Its first 130783
bytes exactly match Downloads/74-current (8).log, CRC32 9E5F4441. The full card
file is 133346 bytes, CRC32 40B6D40C. The additional 2563 bytes contain the
successful USB_GET_END, four subsequent HEALTH records and SESSION_END
reason=shutdown pending=0 at 10:02:39.947 (boot 74, up_ms=906857).
This closes the previously pending physical-card current-download integrity
check. The USB snapshot is preserved byte for byte; later appends are expected.

The resource-series matching idle comparison remains unchanged. Later HEALTH
records after the comparison include motion and USB unplug, with free internal
memory 105284 then 105168 bytes; largest block remains 57332. Those are different
operating conditions and do not replace the matching-idle series readings.

Next: agreed three-way IMU comparison in one sitting. First arm only:
amoled-1-8-core-3-3-11, DIAG_ENABLED=0, DIAG_TEST_HOOKS=0,
DIAG_USB_TEST_FIXTURE=0, DIAG_WRITER_STACK_PSRAM=1. These are instructions to JP;
source flags have not been changed in this review. Stage 0 probes remain active
when DIAG_ENABLED=0; the logger clock/queue/writer initialization is excluded.
Return the card to the powered-off board and keep it installed for all arms.
JP deletes the selected profile's companion.ino.cpp intermediate, then builds
and flashes from VS Code. Leave hotspot/USB connected and use the web console
with DTR=true, RTS=false. Automatic log list may report unavailable with logging
disabled; normal probe lines should still arrive. Send status once, then stay
on dashboard screen 1, stationary, no Live/images/downloads or test commands.
Wait for stable Wi-Fi/MQTT and collect three consecutive complete normal
windows of about 60000 ms. Exclude startup/partial or interrupted windows;
send full console including status and all three probe lines. Next arm is
3.3.11 logging on, then 3.1.3 logging off, in the same sitting with identical
conditions. Writer-core A/B is conditional on logging explaining the drop.
No build, flash, firmware edit, commit or push in this review.

## 2026-09-19 10:09-10:13: IMU arm A, core 3.3.11 logging off

JP supplied four consecutive complete normal windows on the requested
amoled-1-8-core-3-3-11 profile. Status confirms Wi-Fi/MQTT connected,
logger=off, hooks=0, measured=0 and writer_lifecycle=off. Boot=0 and absent
logger memory/placement snapshots are expected with DIAG_ENABLED=0; the Stage 0
probe timer remains on. Local config also reads enabled=0, hooks=0, fixture=0,
PSRAM selection=1. No writer task is running in this arm.

| Run | Window ms | IMU samples | IMU minimum Hz | IMU average Hz | Largest block minimum bytes |
|---|---:|---:|---:|---:|---:|
| 1 | 60004 | 2513 | 27.02 | 42.30 | 59380 |
| 2 | 60006 | 2508 | 27.03 | 42.22 | 59380 |
| 3 | 60000 | 2511 | 22.73 | 42.28 | 59380 |
| 4, extra confirmation | 60006 | 2510 | 30.30 | 42.25 | 57332 |

The first three reported averages have a mean of 42.27 Hz; all four range
42.22-42.30 Hz. These are averages of existing sampling_frequency readings,
not sample-count divided by wall time. Boot internal minimum remains 95812;
10 ms timer has 6000-6001 samples per window, maximum gap 11070 us and maximum
scan 1034 us. No connection transition or active media transfer is reported.

The approximately 42 Hz behavior persists with the SD logger/writer absent.
It therefore cannot be attributed solely to that task or its core placement.
Do not yet assign the cause to the SDK: the logging-on arm and old-profile
logging-off arm are still required. Old versus new profiles also differ in
graphics and expander dependencies. No writer-core comparison is warranted
from this first arm alone.

Next single arm, same sitting: retain core 3.3.11 profile and all physical/UI
conditions, change only DIAG_ENABLED to 1. Keep hooks=0, fixture=0, PSRAM=1.
JP rebuilds/flashes (delete selected profile companion.ino.cpp intermediate),
then uses the same web console, dashboard, stationary unit, connected hotspot
and MQTT. Send status after connection, then collect three complete normal
60-second windows without Live, images, downloads or test commands. Automatic
file listing on initial connection may split the first window; use full ones.
Source: JP inline 10:09-10:13 capture. Documentation only; flags not changed by
Codex, no build, flash, commit or push.

## 2026-09-19 10:17-10:20: IMU arm B, core 3.3.11 logging on

JP supplied three complete normal windows in the same sitting as arm A.
Boot 77 status: ready/synced, hooks=0, PSRAM writer active on core 1 with valid
placement, stack margin 3752 bytes, queue empty/high=1, zero drops/errors.
Wi-Fi and MQTT connected, USB inactive with no failure or link loss. Local
config confirms enabled=1, hooks=0, fixture=0, PSRAM=1.

| Run | Window ms | IMU samples | IMU minimum Hz | IMU average Hz | Largest block minimum bytes |
|---|---:|---:|---:|---:|---:|
| 2 | 60005 | 2508 | 25.64 | 42.31 | 57332 |
| 3 | 60007 | 2534 | 24.39 | 42.71 | 57332 |
| 4 | 60004 | 2548 | 28.57 | 42.98 | 57332 |

Mean of the three reported averages: 42.67 Hz, versus 42.27 Hz logging off
(first three full arm-A windows). Difference is +0.40 Hz, about +0.95%; this
small sequential-run difference is not evidence that logging improves IMU
performance. The measurements show no logger-induced average-rate reduction
in this comparison. Both arms remain near 42-43 Hz. Do not run the conditional
writer-core 0/1 experiment: logging on does not explain the observed drop.
Boot heap minimum remains 95144 throughout; timer interval=10 ms, 6000-6001
samples, maximum gap 10967 us and maximum scan 1118 us.

Next single arm: switch Arduino Maker profile to amoled-1-8 (verified pinned
core 3.1.3), set DIAG_ENABLED=0 and retain hooks=0, fixture=0, PSRAM=1. Delete
build/build_amoled-1-8/sketch/companion.ino.cpp before JP compiles/flashes.
Keep card installed, USB and hotspot connected, dashboard selected and unit
stationary. Connect the same web console, status, then three consecutive full
normal 60-second windows without media/downloads/test commands. Logger off
and unavailable file listing are expected. Send build library/platform versions
and the console so the old profile is identifiable even if startup text is lost.
This compares complete profiles: old graphics and expander libraries also
change, so a difference would not isolate the ESP32 core alone. Continue in
this sitting. Source: JP inline 10:17-10:20 capture and local sketch.yaml.
Documentation only; no firmware edits, build, flash, commit or push.

## 2026-09-19 10:31-10:33: IMU arm C completes the three-way comparison

JP supplied three complete normal windows with logging off. Status confirms
Wi-Fi/MQTT connected, logger=off, hooks=0 and writer_lifecycle=off. Local
Arduino Maker selection is amoled-1-8; build.options.json identifies core
3.1.3 and CPUFreq=240, with an ELF timestamp of 10:29:28. libraries.cache
identifies the old graphics folder and ESP32_IO_Expander 0.0.3. These local
artifacts support the requested old-profile run, although status itself has
no core-version field. Source flags enabled=0, hooks=0, fixture=0, PSRAM=1.

| Run | Window ms | IMU samples | IMU minimum Hz | IMU average Hz |
|---|---:|---:|---:|---:|
| 1 | 60004 | 2947 | 31.26 | 49.21 |
| 2 | 60002 | 2948 | 33.22 | 49.23 |
| 3 | 60004 | 2952 | 30.27 | 49.30 |

Boot internal minimum 86972 and largest-block minimum 31732 stay unchanged.
Probe interval 10 ms, 6000-6001 samples/window, maximum gap 10928 us and
maximum scan 1003 us. No connection transitions reported during these windows.

Same-sitting means of three reported imu_avg_hz values:

| Profile | Logging | Mean Hz |
|---|---|---:|
| 3.3.11 | Off | 42.27 |
| 3.3.11 | On | 42.67 |
| 3.1.3 | Off | 49.25 |

The new-profile logging-off average is about 14.2% lower than the old-profile
logging-off average. Logging on did not lower the new-profile average in this
bounded comparison. The writer is not the sole cause of the rate change; the
conditional writer-core A/B is not warranted. These are software motion-update
frequency observations, not a direct measurement of the sensor hardware ODR.
The configured updateMotionState minimum interval remains 20 ms; actual calls
are serviced by the application loop. SDK, graphics and expander dependencies
change together between profiles, so the test does not identify the responsible
component. The timer sampler itself stayed at its expected cadence.

Next recommended action: bounded read-only timing/source review of the changed
profile's IMU/I2C and main-loop/UI paths before choosing a targeted probe or
fix. Do not alter the IMU update interval or move writer cores on this evidence.
No additional broad bench series is requested now. Board/source currently
remain on the 3.1.3 logging-off comparison build; restoring 3.3.11 logging on
is still required before ordinary SD use and final acceptance. Stage 1B remains
pending JP acceptance and the remaining new-profile TLS/performance gate review.
Source: JP inline 10:31-10:33 capture, local build.options.json, libraries.cache
and Arduino Maker profile selection. Documentation only; no firmware edits,
build, flash, commit or push.

## 2026-09-19 end of day: IMU rate accepted; normal trial restored

JP accepts the approximately 42-43 Hz new-profile motion-update rate and requests
no further investigation. Logging does not explain the difference in the
same-sitting three-way test; the exact core/library cause remains unverified.
No writer-core A/B or new timing instrumentation is planned. Revisit only if
practical symptoms arise. The 20480-byte memory gate is unchanged.

JP reports restoring the new core and logging, compiling/flashing, and seeing
approximately 42 Hz again. Local Arduino Maker selects amoled-1-8-core-3-3-11;
config is enabled=1, hooks=0, fixture=0, PSRAM writer=1. No raw post-restore
capture or boot number supplied. Preserve this as JP's confirmation, not a
new full gate measurement. JP is stopping for the day and requests commit/push.

Resume from sd_diagnostics_checkpoint_2026-09-19.md. Final TLS/performance
evidence review and explicit JP Stage 1B acceptance remain before Stage 2.
Do not repeat completed transport/storage/resource/card tests or the accepted
IMU investigation. No runtime behavior changes, build or flash by Codex.

## Post-19845b8 review: remaining performance checks; case A prepared

JP supplied the agreed review outcome: existing 3.3.11 TLS/memory evidence is
sufficient; retain the 20480-byte gate. For example, September 17 boot 47 Live
has sampled largest minimum 26612 and TLS 31732; its small-bundle overlap has
Live/TLS minima 31732. September 18 Latest also records HTTPS minimum 31732.
These are existing measurements, not a new bench run.

Two focused comparisons remain in one sitting: logging-off versus logging-on
Live, then logging-on Live alone versus current-plus-newest-three retrieval
starting about 10 seconds into Live. Use frames/duration and about 5% maximum
loss for each comparison, considering network variation. Require CRC success,
zero logging-on queue drops, no new stalls/resets and memory above the gate.
Keep the roughly 8% exception confined to large downloads. Reuse all other
passed evidence; no further IMU, writer-core or older-core Live testing.

Live does not print every frame: geometry is guarded by dimension changes and
summaries print at completion. Installed 3.3.11 HWCDC TX-lock waits use the
configured TX timeout, supporting the focused regression check after a675a76.

Prepared only case A: DIAG_ENABLED=0, hooks=0, fixture=0, PSRAM selection=1;
profile remains amoled-1-8-core-3-3-11. Removed the selected generated sketch
before JP's VS Code rebuild. JP will send status, run Latest once, complete
one full Live cycle without downloads, then send status and retain the console.
No hardware result yet and no Stage 1B acceptance. No build/flash/commit/push
by Codex; untracked docs/sd_iphone_log_download_plan.md preserved.

## 2026-09-19 21:16-21:18: Live case A logging-off baseline passes

JP reports video looked fine and the test was OK. Local profile remains
amoled-1-8-core-3-3-11; pre-edit flags were enabled=0, hooks=0, fixture=0,
PSRAM selection=1. Capture confirms logger=off and writer_lifecycle=off,
Wi-Fi/MQTT connected before and after, and increasing uptime 28521 -> 139962 ms.
Logger boot=0 and zero logger memory snapshots are expected with logging off;
use the independent Stage 0 probes for memory. No reset/media failure observed.

| Measurement | Case A, logging off |
|---|---:|
| Live frames | 209 |
| Live probe duration | 60267 ms |
| Calculated FPS using probe duration | 3.4679 |
| Video summary duration / FPS (rounded) | 60.3 s / 3.5 |
| Mean frame / first frame / max gap | 288 / 1105 / 951 ms |
| HTTP / decode / blit | 278 / 70 / 78 ms |
| HTTP TTFB / transfer | 160 / 117 ms |
| Mean JPEG / transfer rate | 12.9 KB / 109 KB/s |
| Latest bytes / response / total from button | 19665 / 780 ms / 1207 ms |
| HTTPS sampled largest minimum | 31732 bytes |
| Live TLS sampled largest minima, three windows | 31732 / 31732 / 31732 bytes |
| Full Live sampled largest minimum | 31732 bytes |
| Since-boot internal heap minimum after Live | 45220 bytes |

FPS uses the full-Live probe window as the millisecond duration proxy, rather
than the one-decimal printed FPS. The video summary and probe end a few
milliseconds apart; 209 / rounded 60.3 s also gives 3.466 FPS. This tiny timing
precision difference does not affect the gate. Use the same method for B/C.
Approximate 5% lower boundary for B: 3.2945 FPS.

Memory passes the unchanged 20480-byte gate with 11252 bytes margin. Full Live
has 6026 samples at 10 ms, maximum sample gap 11215 us and scan 880 us. HTTPS
has 78 samples over 779 ms; three Live TLS windows have 65/69/66 samples over
645/686/658 ms. No USB download occurred. No active-writer resource or queue
performance claim is made from logging-off zero counters. No IMU re-test.

Next case B prepared: restored only DIAG_ENABLED=1, retained hooks=0,
fixture=0, PSRAM=1 and the same profile, and removed its generated
companion.ino.cpp before JP's rebuild. In this same sitting JP builds/flashes
in VS Code, connects the web console DTR=true/RTS=false with test switches off,
waits for Wi-Fi/MQTT, sends status, runs Latest once and one full Live cycle
without downloads, then status. Retain full console and visual observations.
Stage 1B remains pending B/C and JP's explicit acceptance. No build, flash,
commit or push by Codex. Untracked iPhone plan untouched.

Source: [case A raw console](bench_data/sd_live_2026-09-19_case_a.txt), copied
unchanged from attachment eb611736-78d1-4161-a56b-5534e3b464c7/Pasted text.txt,
and JP's visual pass report.

## 2026-09-19 21:23-21:25: case B passes paired logging-on comparison

JP reports video was fine and the test was OK. Same sitting as case A,
core 3.3.11 profile; normal logging enabled, hooks=0, fixture=0, PSRAM writer=1.
Boot 82 remains ready/synced with Wi-Fi/MQTT connected before and after.
No USB download, reset, media failure or new stall observed in the capture.

| Measurement | A, logging off | B, logging on |
|---|---:|---:|
| Frames / full-Live probe duration | 209 / 60.267 s | 219 / 60.142 s |
| Calculated FPS | 3.4679 | 3.6414 |
| Mean frame / first frame / max gap ms | 288 / 1105 / 951 | 275 / 1121 / 918 |
| HTTP / decode / blit ms | 278 / 70 / 78 | 265 / 57 / 80 |
| TTFB / transfer ms | 160 / 117 | 150 / 115 |
| Mean JPEG KB / transfer KB/s | 12.9 / 109 | 12.9 / 112 |
| Latest bytes / total ms | 19665 / 1207 | 19665 / 1191 |
| HTTPS / Live TLS largest minima bytes | 31732 / 31732 | 31732 / 31732 |
| Full Live largest minimum bytes | 31732 | 28660 |
| Since-boot internal heap minimum bytes | 45220 | 40912 |

FPS increased 5.0025%; the approximately 5% maximum-loss criterion passes.
Use the same probe-duration proxy as A, not the rounded video FPS. Network
transfer and HTTP timings improved, and decode time changed; this bounded
sequential pair does not establish that logging improves performance.
Latest is 16 ms faster (about 1.33%) on equal-length images.

Memory passes: full-Live minimum is 8180 bytes above the unchanged 20480 gate;
HTTPS and all three TLS windows each have 31732 bytes minimum. Full Live:
6014 samples at 10 ms, max gap 11434 us, scan 1535 us. HTTPS: 75 samples/750 ms;
Live TLS: 63/62/66 samples over 631/619/657 ms. Writer stack margin is unchanged
at 3752 bytes, valid PSRAM placement and internal TCB, core 1. Queue 0/16,
high=1, drops/suppressed/truncated=0, error=none, slow=0. Writes 7 -> 8 and
current size 157043 -> 157602. USB inactive/unpaused, no retained failure/loss.
This is not a repeated-download resource measurement; prior passed evidence
continues to cover that gate. JP's IMU acceptance remains unchanged.

Only case C remains before presenting Stage 1B for explicit acceptance:
keep the same logging-on build and boot 82, same sitting, Wi-Fi/MQTT connected,
web DTR=true/RTS=false and all browser fault switches off. Refresh the file
list before Live only if needed and let listing finish. Run one full Live;
about 10 seconds in click Download current + newest 3 once. Let all four
transfers and Live finish, then send status before any refresh/retry. Require
all CRC checks OK, no new stalls/resets, zero drops and memory >=20480 bytes.
Compare C to B's 3.6414 FPS; approximate 5% lower boundary is 3.4593 FPS.
Review network variability if marginal. No additional Latest or rebuild needed.

Saved [case B raw console](bench_data/sd_live_2026-09-19_case_b.txt) unchanged
from attachment 10a7eee3-5dde-4a03-82eb-addd245e92d2/Pasted text.txt.
Documentation/evidence only; no firmware edits, build, flash, commit or push.

## 2026-09-19 21:28-21:29: case C passes; Stage 1B ready for owner acceptance

JP reports video was fine and the test was OK. Same build and boot 82 as B,
logging on, post-a675a76 USB timeout policy, no optional Live pacing.
Current plus newest three starts 10.993 seconds after Live begins. All four
browser/device CRC checks pass, entirely during Live:

| File | Bytes | Browser duration | Result |
|---|---:|---:|---|
| current.log | 159448 | 1.60 s | CRC OK |
| archive-00000019.log | 153 | 0.03 s | CRC OK |
| archive-00000018.log | 5177 | 0.08 s | CRC OK |
| archive-00000017.log | 308745 | 3.02 s | CRC OK |

Total 473523 bytes; command-to-last-completion span 4.727 s. Archive 19 remains
preserved partial-header evidence. Archive 18 here is 5177 bytes, not the old
2 MiB synthetic fixture. No new independent saved-file byte comparison is
claimed from this capture; the prior passed integrity/prefix evidence is reused.

| Measurement | B, Live alone | C, Live plus bundle |
|---|---:|---:|
| Frames / full-Live probe duration | 219 / 60.142 s | 212 / 60.193 s |
| Calculated FPS | 3.6414 | 3.5220 |
| Mean frame / first frame / max gap ms | 275 / 1121 / 918 | 284 / 1136 / 983 |
| HTTP / decode / blit ms | 265 / 57 / 80 | 273 / 58 / 81 |
| TTFB / transfer ms | 150 / 115 | 153 / 120 |
| Mean JPEG KB / transfer KB/s | 12.9 / 112 | 12.9 / 107 |
| Live TLS largest minimum bytes | 31732 | 31732 |
| Full Live largest minimum bytes | 28660 | 28660 |
| Since-boot internal heap minimum bytes | 40912 | 40912 |
| Writer stack margin bytes | 3752 | 3304 |

Calculated FPS loss 3.2784%, within approximately 5%; use the same probe-window
duration proxy as A/B. Network throughput also fell, so do not assign the
entire small difference to USB. Max frame gap remains below one second, JP
reports normal video, and the capture shows no new stall, failure or reset.

Memory passes: 28660 is 8180 above the unchanged 20480-byte gate. All three
Live TLS windows have minimum 31732 with 64/73/66 samples over 645/727/660 ms.
Full Live has 6019 samples at 10 ms, max sample gap 11530 us and scan 1160 us.
Writer margin falls after exercising retrieval to 3304 bytes (4888 used of
8192), matching the September 19 passed resource series; no overflow observed.
This is a path high-water observation, not evidence of a leak or unchanged
stack usage. Prior repeated-download matching-idle evidence remains applicable.

Final boot 82 ready/synced, Wi-Fi/MQTT connected, hooks=0, valid PSRAM writer
placement/core 1; queue 0/16 high=1; drops/suppressed/truncated=0, error=none,
slow=0. Current grows to 161185 bytes after the snapshot. USB idle/unpaused,
result=ok, retained failure valid=0; final transfer link losses=0. Link counters
are per transfer and do not by themselves establish every bundle member's
link-loss history. All four members independently reported CRC OK.

The two focused remaining performance checks are complete. Recommend Stage 1B
acceptance with the existing narrow approximately 8% large-download exception,
accepted 42-43 Hz profile rate, unresolved pre-logger VS Code monitor-close
freeze/reset and deferred unsupported/bad-card testing. No separate cause-tagged
loop-gap probe was added; use the existing summaries and visual observations.
Stage 1B is NOT marked accepted: request JP's explicit acceptance before Stage 2.
Stages 2-4 have not started. Normal flags remain enabled=1, hooks=0, fixture=0,
PSRAM=1. No further bench case or rebuild requested.

Source: [case C raw console](bench_data/sd_live_2026-09-19_case_c.txt), copied
unchanged from attachment e8054e2f-8c72-4d89-ae4c-7361aa7476ef/Pasted text.txt,
and JP's visual pass. Documentation/evidence only; no build/flash/commit/push.

## Stage 1B accepted by JP - September 19, 2026

JP explicitly stated: "I accept Stage 1B. Please commit and push then proceed
to the next step." Stage 1B is accepted with the documented large-download-only
FPS exception, accepted IMU rate, unresolved pre-logger monitor-close issue
and deferred unsupported/bad-card testing. The 20480-byte memory gate and
transfer limits remain unchanged. Cases A/B/C pass; normal logging-on flags
remain enabled=1, hooks=0, fixture=0, PSRAM=1 on core 3.3.11 (last measured boot 82).

Commit and push this acceptance/evidence checkpoint first, then implement
Stage 2 network-event logging under the existing plan. JP builds and flashes;
bench instructions remain one case at a time. Earlier pending-acceptance
statements below are historical and superseded by this explicit decision.
Leave the untracked iPhone download plan untouched.

## Stage 2 preparation after explicit Stage 1B acceptance

Accepted Stage 1B checkpoint `f899e3c` was committed and pushed at JP's request.
Stage 2 observation hooks are now local: Wi-Fi driver/profile/retry evidence,
all MQTT attempts and first observed loss, error snapshots before application
cleanup, setup/TLS spans and breadcrumbs, bench controls, message aggregates,
notification decisions and publish/subscribe acceptance. Existing network
policy and Stage 1B transport limits are unchanged. See STAGE2.md for scope,
API-observation limits and the first normal-startup case.

61 host checks pass (14 new source-contract checks, 47 existing browser/USB
checks). No firmware compiled/flashed and no Stage 2 hardware result. Flags
remain enabled=1, hooks=0, fixture=0, PSRAM=1; tag=stage2-network. Removed the
selected generated sketch before JP's rebuild. Stage 2 changes are uncommitted.

## 2026-09-19 22:02-22:04: Stage 2 startup records verified; prior watchdog needs context

JP supplied the console and identified Chrome Downloads. Read
C:/Users/photo/Downloads/84-current.log: 190637 bytes, CRC32 5C52544D,
SHA256 be39465b7e3981031a682f8651fccfc1584bb0d671d4bccaf3bd0cbfbf785836.
Matches the browser-reported byte count and CRC OK outcome; the browser did
not expose a numeric reference CRC, so the computed hash identifies the saved
file rather than a second device/card byte comparison. Original preserved.

Sources: [console](bench_data/sd_stage2_2026-09-19_boot84/console.txt) and
[saved current log](bench_data/sd_stage2_2026-09-19_boot84/84-current.log.txt).

Boot 84 contains build=stage2-network, compiled Sep 19 2026 21:55:31. The local
ELF timestamp is 21:59:02 and Arduino Maker selects core-3-3-11. This establishes
JP's built/flashed Stage 2 runtime, not a build/flash by Codex. Flags remain
enabled=1, hooks=0, fixture=0, PSRAM=1. Stage 2 stays uncommitted/unaccepted.

All 33 boot-84 records parse, sequence 1-33 is contiguous, all five setup/scan/
association/MQTT spans pair correctly, and no NET_FORMAT_ERROR appears:

- Driver scan: status=0, nine networks; main scan duration 2884 ms.
- Primary association connection=1/channel=6 and GOT_IP changed=1 are captured
  separately from main-task association wait (1541 ms).
- Real MQTT connection=1, Wi-Fi connection=1, port=9735, TLS enabled: attempt
  id=5 succeeds in 540 ms, state=0. All three subscriptions and calibration
  publish report accepted=1, ack=unobserved. No mismatch. Setup completes.
- HEALTH and NET_HEALTH at uptime about 61.96 s: snapshot_age_ms=151,
  inbound=2 (power=1, energy=1), inbound age 1143 ms, RSSI -61 valid, suppression=0.
  Older boot 83 also shows counters advancing 2 -> 4 across its two health records.
- Final downloaded record is USB_GET_BEGIN, as expected for a current snapshot.
  Its own USB_GET_END is expected in later appends, not this downloaded snapshot.

Runtime: ready/synced and Wi-Fi/MQTT connected at both statuses; uptime advances
23082 -> 120931 ms. Queue high=7, drops/suppressed/truncated=0, error=none,
slow=0. The one queued status event drains to zero in the following USB snapshot.
Current grows from 189506 to 190933; writes 29 -> 34. Download 1.81 s, 105604
payload B/s; final USB inactive/unpaused/result=ok, no retained failure/link loss.
Writer stack margin 3640 -> 3144 after retrieval, valid PSRAM placement/core 1.
Internal boot minimum 92316 and retained largest minimum 51188; this idle case
stays above 20480 but is not a new TLS/Live performance-gate measurement.
Normal 60010 ms probe has 6001 samples, IMU average 42.78 Hz (no new IMU inquiry).

Important separate finding: boot 84 reports reset=task_watchdog (code 6) and
previous boot 83 breadcrumbs main=idle at 10197 ms, writer=idle at 122597 ms.
Boot 83 had completed Stage 2 setup and two healthy zero-drop health intervals.
The restart preceded this console capture, around 22:02:06. Idle breadcrumbs
do not identify a blocked call or prove a cause. No reset occurs within the
supplied boot-84 capture. Asked JP whether closing the VS Code monitor/switching
to Chrome triggered the restart, or whether it happened spontaneously. Do not
attribute it to the pre-existing monitor-close issue without that context.

The record-format/startup/health/download checks pass. Overall first-case
reset assessment awaits JP's answer before the deliberate MQTT outage case.
No firmware change, rebuild, flash, commit or push during this analysis.

### JP clarification: watchdog followed VS Code monitor closure

JP confirms: "Yes I closed VS code monitor. this is normal". The preceding
boot-83 -> 84 restart therefore occurred during the known monitor-close
transition, which predates SD logging. Record the owner-observed association;
it does not isolate the root cause or establish a fix. The boot-84 supplied
startup/health/download case passes; no spontaneous restart was reported.

Next single case: same normal Stage 2 build, hotspot on, dashboard, web console
DTR=true/RTS=false, test switches off. Send off once, wait for two completed
TEST MQTT attempt END lines, then on once. Wait for real broker connection,
send status, download current.log once, final status. Do not add media or a
hotspot outage to this case. Failed test attempts are expected; capture actual
duration/state/TLS observations and recovery, zero drops/errors and CRC success.
Commands can wait behind a blocking attempt. No rebuild, commit or push.

## 2026-09-19 22:10-22:11: Stage 2 controlled MQTT off/on passes, boot 84

JP reports the test ran fine. Same Stage 2 build/boot 84, Wi-Fi connected
throughout, no media. Saved Chrome download 84-current (1).log has 200198 bytes,
CRC32 14419DAE, SHA256 caa93f137d05e34c3579538839409383f10d3004b98590df1a25a05c56057def.
The previous 190637-byte 84-current.log is its exact byte prefix. New file is
UTF-8/newline terminated; boot-84 sequence 1-69 is contiguous and parses,
all eight spans pair, and no NET_FORMAT_ERROR is present. Browser CRC OK,
1.95 s, decoded 102729 B/s, wire 144313 B/s. This is saved-snapshot continuity,
not a new card-reader comparison.

Sources: [console](bench_data/sd_stage2_2026-09-19_boot84_off_on/console.txt)
and [saved current log](bench_data/sd_stage2_2026-09-19_boot84_off_on/84-current.log.txt).

| SD attempt ID | Target | Result/state | Recorded duration | TLS observation |
|---|---|---|---:|---|
| 6 | test, TLS port 9735 | failed / -2 | 5003 ms | queried=1, code=-1, Generic error, freshness unknown |
| 7 | test, TLS port 9735 | failed / -2 | 5003 ms | queried=1, code=-1, Generic error, freshness unknown |
| 8 | real, TLS port 9735 | ok / 0 | 606 ms | queried=0 |

Serial bench numbers 1/2/3 are separate from the per-boot diagnostic IDs.
Serial elapsed values are 5005/5004/607 ms and independent probe windows
5002/5002/605 ms; these instrument boundaries differ slightly, not inconsistent
attempts. Test attempt 7 starts 15102 ms after attempt 6's END timestamp,
consistent with the unchanged 15-second post-attempt retry delay plus cleanup.
The generic -1 secure-client error does not establish a TLS-handshake cause.

BENCH_REQUEST and BENCH_APPLIED identify off/on correctly; requested disconnect
records preserve state_before=0 and -2. On restores the real configuration;
MQTT_CONNECTED has recovery=1; all three subscriptions and calibration publish
report accepted=1/ack=unobserved. No fabricated MQTT_LOST for the intentional
off action. Driver Wi-Fi loss and spontaneous MQTT loss remain to be exercised.
An outage NET_HEALTH has inbound=14 (power=7, energy=7), age=62325 ms; inbound
age describes application callbacks, not a broker keepalive failure.

Both test MQTT probe largest minima are 63476 bytes (500/501 samples); real
recovery minimum 55284 (61 samples). All use the retained 10 ms probe and exceed
20480. Final internal boot minimum 92048, retained largest 51188, writer margin
3144 unchanged, valid PSRAM placement/core 1. Ready/synced, queue high=7,
zero drops/suppressed/truncated/errors/slow writes. Status queue 1 drains to 0;
current appends resume, final size 201336, USB idle/unpaused/result=ok, no retained
failure or link loss. Same boot throughout; no new reset observed or reported.

Next single case: hotspot loss/recovery followed by Latest once, same build.
Keep USB power and browser connected, dashboard, DTR=true/RTS=false, test
switches off. Send status; turn hotspot off for 30 seconds, send status once,
then restore hotspot and leave its settings page open for discovery. Wait for
Wi-Fi and real MQTT recovery, up to about 90 seconds. Do not issue serial off/on
or manually reset. If not recovered, stop and capture status/console. On recovery,
run Latest once, then status, download current once and final status. Expect
Wi-Fi driver disconnect/reassociation/GOT_IP, observed MQTT loss/recovery,
paired HTTPS evidence, CRC success, zero drops/errors and memory >=20480.
This single case adds actual Wi-Fi loss and post-recovery HTTPS; Live remains
for a later separate case. No rebuild or firmware change requested.
Stage 2 remains uncommitted and unaccepted. No build, flash, commit or push.

## 2026-09-19 22:14-22:16: Stage 2 hotspot recovery works; diagnostic corrections pending repeat

JP reports the test ran fine. Boot 84 throughout, automatic Wi-Fi/MQTT recovery
followed by Latest. Sources: [console](bench_data/sd_stage2_2026-09-19_boot84_hotspot/console.txt)
and [current log](bench_data/sd_stage2_2026-09-19_boot84_hotspot/84-current.log.txt).
Saved download: 215429 bytes, CRC32 F2B32C72, SHA256
57f4196db9798441c34f3e57312121f4704ca7eacfb00deeaada0acb93048275.
It preserves the previous 200198-byte snapshot exactly. Boot-84 records 1-131
are contiguous and parse; all 11 spans pair. CRC OK, download 2.05 seconds.

Initial WIFI_DISCONNECT reason=2/auth_expired at uptime 770864, RSSI -39.
MQTT_LOST follows with state=-3, TLS code 48 UNKNOWN ERROR CODE (0030),
freshness unknown: this does not prove a new TLS failure. Offline attempt id9
fails in 2 ms, state=-2, generic TLS -1. Its 1 ms probe has zero samples and
cannot establish a TLS memory gate. Alternating reasons 201/no_ap_found and
36 (then labeled other) recur about every 2.415 seconds, all suppressed=0;
raw -128 is incorrectly accepted and propagates into NET_HEALTH.

Association resumes at 807216, primary connection1/channel6; GOT_IP 808586,
changed=0. No LOST_IP event was observed. Real MQTT id10 succeeds in 776 ms,
state=0/recovery=1, subscriptions/calibration accepted. HTTPS id11 takes
857 ms to headers, HTTP200; Latest total 1289 ms, 19665 bytes, normal display.
Recovery probe: 77 samples, largest minimum 51188. HTTPS: 85 samples, largest
minimum 31732. Both exceed unchanged 20480 gate. Final ready/synced, queuehigh7,
zero drops/errors/slow writes/truncation, writer margin3144, USB idle/unpaused,
result=ok; final current size215727/writes132. No new reset observed/reported.

Functional outage/recovery/Latest passes. Diagnostic fidelity needs correction:
reject the observed int8 lower bound -128 (not a claimed universal SDK sentinel),
retain and label historical valid RSSI, expose raw validity; suppress repeated
reason/profile combinations even when interleaved. Local fix uses four fixed
slots with five-second windows, resets on association, and reports suppressed_any
plus cumulative health suppression. Reason36 now labeled sta_leaving per installed
SDK enum. No retry, timeout, transport or writer-policy changes. Sixteen network
host checks pass, including translated policy replay; not a firmware compile.
Next: JP rebuild/flash and repeat only this hotspot/recovery/Latest case.
Stage 2 remains uncommitted/unaccepted. No build/flash/commit/push by Codex.

## 2026-09-19 22:27-22:29: corrected hotspot case passes, boot 86

JP reports normal operation. Both diagnostic corrections pass on hardware:
raw_rssi=-128/raw_valid=0 retains rssi=-31/rssi_source=last_valid. Initial
beacon_timeout reason200 has raw_valid=1. Alternating reasons201 and36 are
reported about every7.245 seconds per reason, with four suppressed events
between pairs. NET_HEALTH wifi_suppressed=16 confirms cumulative suppression;
logger queue suppression/drops remain zero. Association resets pending count;
reason36 is sta_leaving. Recovered valid health RSSI is -18.

MQTT_LOST state=-3 precedes recovery; TLS48 freshness unknown is not a proven
new TLS failure. Reconnection id6 succeeds in486ms, state0/recovery1, all three
subscriptions/calibration accepted. Latest id7 headers714ms/HTTP200, total1135ms,
19665bytes; motion publish accepted. Sampled recovery minimum55284 (49 samples),
HTTPS28660 (71 samples), both above20480. Writer margin3640 before download,
3144 after; queuehigh7, zero drops/errors/truncation/slow writes. USB ends idle,
unpaused/resultok with no link losses. No reset during captured test. Boot86
itself reports a watchdog before capture with prior boot85 idle breadcrumbs;
this capture does not establish its cause. The earlier monitor-close limitation
remains unresolved and is not newly diagnosed here.

Download 241825 bytes, CRC32 9CC35ADA, SHA256 2f408372b9feafd5e48af8822702236aaa407b1629b0703294e4c0b337a97c8d.
Prior boot84 snapshot is an exact byte prefix; 62 contiguous boot86 records,
seven paired spans, no format errors. CRC OK;2.36s. Sources:
[console](bench_data/sd_stage2_2026-09-19_boot86_hotspot/console.txt) and
[current log](bench_data/sd_stage2_2026-09-19_boot86_hotspot/86-current.log.txt).

Next single case, same build: keep hotspot on, dashboard, browser DTR=true/
RTS=false and test switches off. Status; send off once. After first TEST MQTT
attempt END, promptly start one full Live cycle (before next15-second retry).
Keep MQTT off throughout Live; no downloads during it. After normal Live finish,
send on once, wait for real MQTT recovery, then status/download current/status.
Expect media retry deferral, paired Live connect records, no MQTT attempts while
Live is active, release after media ends, normal video and recovery, CRC OK,
zero drops/errors and memory>=20480. Capture full console, log and visual report.
No rebuild. Stage2 remains unaccepted; Stage3-4 unstarted. No firmware changes.

## 2026-09-20 08:06-08:08: Live during MQTT outage passes, boot 87

JP reports video and test ran fine. Same corrected Stage2 build; boot87 is
power_on, no reset during case. Intentional off precedes test attempt id6,
5003ms/state-2/TLS-1 generic, freshness unknown. Live starts after that attempt.
Paired Live connects id7/id8 succeed in693/625ms. MQTT_RETRY_POLICY deferred
at uptime50499 and released/media_clear at110161. No MQTT attempt within
Live; next test id9 starts110262, 101ms after release, fails in5003ms.
The on command waits behind this already-running call, then restores real
broker. Real id10 succeeds603ms/state0/recovery1; subscriptions/calibration
accepted. This is expected blocking-command behavior, not a new stall.

176 frames /60.326s = 2.9175 FPS. First frame1250ms, max gap939ms,
average18.2KB/frame, HTTP334ms (TTFB153+xfer181), decode61ms, blit80ms.
No direct regression percentage against last night's smaller-image runs:
scene/image size and network conditions differ. Next normal connected run
provides a same-session comparison; this is not a logging-off/on comparison.
Live sampled largest minimum34804 (6033 samples); both TLS windows42996
(69/63 samples). Test attempts63476 each (500 samples), real recovery49140
(61 samples). All above20480. Queuehigh6; drops/errors/suppressed/truncated/
slow writes0; writer margin3144 after download. USB idle/unpaused/resultok,
no link loss. NET_HEALTH correctly shows mqtt0 during Live and mqtt1 afterward;
inbound remains2, age advances: application counters, not keepalive evidence.

Download 258232 bytes, CRC32 C5BA48A1, SHA256 2aa3eafb32f37b070cc61f4fe801a7adf3d2724aef41b9923dc35fdf3746a9ef.
CRC OK,2.47s; prior boot86 snapshot exact prefix. Boot87 records1-59 contiguous,
all ten spans paired; no NET_FORMAT_ERROR. Sources:
[console](bench_data/sd_stage2_2026-09-20_boot87_live_outage/console.txt) and
[current log](bench_data/sd_stage2_2026-09-20_boot87_live_outage/87-current.log.txt).

Next single case: same build/session, hotspot on and normal MQTT connected,
DTR=true/RTS=false, test switches off. Send status, run one full Live cycle
with no off/on commands or downloads, then status, download current, status.
Send console/log and visual observations. Compare frame count/duration and
network/image metrics with this outage run; target no more than about5% loss,
interpret marginal differences with network variability. No rebuild. Stage2
remains unaccepted; Stages3-4 unstarted. Completed result commit/push authorized
by JP. iPhone plan untouched; no firmware changes or new tests required.
