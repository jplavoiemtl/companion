# SD diagnostics bench results

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
