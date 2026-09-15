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
