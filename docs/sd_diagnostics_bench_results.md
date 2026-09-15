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
