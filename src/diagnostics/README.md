# Stage 0 measurement probes

Project owner and bench tester: **JP**, who builds and flashes from VS Code.

The Step 0 USB console passed the tested browser cycles with explicit DTR=true,
RTS=false. See [bench results](../../docs/sd_diagnostics_bench_results.md).
The VS Code monitor disconnect freeze remains unresolved. Stage 0 initial
measurement coverage is complete: normal, Latest HTTPS, Live TLS, full Live,
and failed and successful MQTT connects. JP accepted this checkpoint.
The probe-only reference is commit f406063. Stage 1 now retains these probes
unchanged; see [its handoff](STAGE1.md). JP's initial Stage 1 readiness check passed;
Latest then failed the internal-block memory floor (14836 versus 20480 bytes).
No-card HTTPS recovered to 28660 bytes. The writer-start-order retest still
measured 14836 bytes with the card installed. Stage 1 acceptance remains on hold
pending allocation diagnosis. JP verified late retrieval of retained startup
snapshots with log status. Latest still measured 14836 bytes; see the Stage 1
handoff and bench results for allocation-phase evidence.
JP tested the writer-stack A/B experiment with hooks off. Latest measured 14836
bytes with the internal stack and 26612 with PSRAM, passing B's initial memory
check. Both used 4204 bytes of stack, with correct placement and no reported
errors or drops. B's overnight Live run on 2026-09-16 completed normally, but
both TLS windows and full Live measured 14324 bytes, failing the memory floor.
Fresh-boot B then passed full Live at 25588 bytes, with normal video and no errors.
Three more same-boot Live cycles passed at 25588, 25588 and 26612 bytes with
normal video and no logger errors or drops. The overnight cause is unresolved.
G-meter selection, saves to G-meter and dashboard, and following Live passed.
Normal-window minima stayed 31732; Live measured 26612 with unchanged stack use
and no logger errors or drops. Inclinometer navigation and both saves also passed;
following Live measured 25588 with unchanged memory history and stack usage.
Latest-to-Live then passed at 26612 bytes in both windows with unchanged stack
usage and no logger errors or drops. JP supplied current.log; the
[overnight analysis](../../docs/sd_diagnostics_overnight_analysis.md) found a transient
low between 07:07 and 07:08 while sampled offline, and reduced current contiguous
space at the 07:34 recorded reconnection. A short outage after Latest reproduced
14324 inside mqtt_connect and subsequent Live. Next is a fresh-boot hotspot
control with no prior media request. The allocation cause is unresolved;
hooks remain off. See STAGE1.md.
JP's local switch is 1 for B testing; the intended default remains 0.
No file protocol or firmware USB configuration change is included.

## Windows and output

- mqtt_connect: each PubSubClient connect call, including test attempts. Covers TCP,
  TLS when applicable, and CONNACK. An immediate rejection can have zero timer samples.
- image_https: HTTPS GET through response headers. HTTPClient connects inside GET,
  so this includes DNS, TCP, TLS and server response time, not just the handshake.
  Plain HTTP requests do not produce this window.
- live_tls: each actual shared secure-client connect; reuse does not produce another.
- live: startup allocations through stop and cleanup, including nested live_tls windows.
- normal: up to 60 seconds of ordinary loop operation. Ends early when media or a
  measured connect starts. Setup is excluded. Only this summary carries IMU rate.

Every finished window prints one [PROBE] line. There is no per-sample output.
The esp_timer task samples the largest free internal block every 10 ms. In this SDK
that task runs on core 0, independently of the core 1 application loop. No new sampler
task stack is allocated. Timer initialization failure is reported; measurements then
have boundary readings only and are not sufficient for the sampling gate.

| Field | Meaning |
|-------|---------|
| run | Per-window occurrence number since boot |
| ms | Measured window duration |
| heap_min_boot | Sum of heap regions' since-boot low watermarks, possibly from different times; not a window-local or simultaneous global minimum |
| largest_min | Lowest observed heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL), including two boundary readings |
| interval_ms | Requested periodic interval, 10 ms |
| samples | Periodic readings for this window, excluding boundary readings |
| gap_max_us | Longest interval between readings, including the leading and trailing boundary gaps |
| scan_max_us | Slowest observed largest-block query, useful for evaluating probe overhead |
| timer | on when the periodic timer started; off on initialization failure |
| imu_n | Number of eligible existing sampling_frequency observations |
| imu_min_hz | Minimum eligible sampling_frequency |
| imu_avg_hz | Arithmetic mean of eligible sampling_frequency values |

IMU values observe the existing calculation in updateMotionState; no sensor behavior
changes. Exclude Live, still-image work, measured connects and setup. Discard the first
frequency after each normal window begins because its interval can cross excluded work.
An imu_n of zero means no rate measurement. A fresh normal window is also started after
a summary so its printing delay does not contaminate the next IMU interval.

Stage 1 keeps these probes unchanged. Stage 1B must pass false to
diagnosticsProbeNormalUpdate while USB file transfers are active. The Step 0 console
is not a file transfer; monitor use is allowed during the normal baseline.

Largest-block minima are sampled, not exact. Inspect count, gap and scan cost together.
A zero count or long gap cannot establish coverage of a brief allocation. heap_min_boot
uses the allocator's low-water API and does not depend on sampling.

The existing Video summary still reports fps and average frame timing. It now also has
first_frame (startup to first displayed frame) and max_gap (largest interval between
successfully displayed frames), in milliseconds. With fewer than two frames, max_gap
is zero. The existing Latest total-time output is reused.

## Owner bench sequence

1. Step 0 passed on the tested board with Chrome 152 and explicit DTR=true,
   RTS=false. Use those settings in tools/sd_log_browser.html. For a different
   setup, repeat the connection check including Live. Compare status uptime in
   the page before and after reconnecting; keep USB plugged in and observe touch.
   The VS Code disconnect freeze remains unresolved. Close the browser port
   before building and flashing.
2. Before each firmware rebuild after changing companion.ino, delete
   build/build_amoled-1-8/sketch/companion.ino.cpp. Build and flash from VS Code.
3. With the hotspot and real MQTT connected, leave the normal screen untouched for
   at least 60 seconds to collect a normal IMU window. Save the complete summary.
4. Request Latest, then run one full Live cycle. Save the image_https, live_tls and live
   summaries, the Latest total time and the Video timing lines.
5. Between media operations, use off, allow one MQTT attempt to finish, then on. Save
   both mqtt_connect summaries. Commands retain their existing blocking behavior.

Record the firmware build, board and power source, display, hotspot, sampler status,
all probe lines and any visual slowdown or error. Check the proposed 20 KiB largest-block
threshold during TLS; if this probe-only baseline falls below it, discuss that result
before attributing a later change to logging.

Today's run establishes probe coverage and the baseline procedure. For Stage 1's actual
performance gate, remeasure this probe-only build immediately before the logging build
in the same sitting. Keep the probes, scene, hotspot and test sequence the same. Compare
normal IMU minimum/average, Live fps and gaps, Latest total time and internal-memory
figures. Another day's measurements are not a valid paired baseline.
