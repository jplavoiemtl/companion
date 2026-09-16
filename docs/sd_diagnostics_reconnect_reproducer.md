# SD diagnostics checkpoint: reconnect memory-headroom reproduction

Date: 2026-09-16. Owner and bench tester: JP.
Branch: sd-diagnostics. Base before this checkpoint: 54baf96.
This is a diagnostic checkpoint, not Stage 1 acceptance or a production fix.

## Reproduced condition

With the PSRAM-stack experiment, Latest succeeds with largest_min=26612 bytes.
After a short real Wi-Fi hotspot outage, the MQTT reconnect window reaches
14324 bytes, below the unchanged 20480-byte floor. The next Live cycle also
measures 14324. JP reports normal visible operation throughout.

This reproduces the low-headroom measurement, not the original car symptom of
unresponsive image buttons. No allocation/TLS failure or logger error was observed.
The first observed low in this controlled sequence is inside mqtt_connect.
That locates timing, not allocation ownership or a proven memory leak.

## Configuration and reproduction

Use the battery-equipped board and 16 GB card, pinned ESP32 core 3.1.3 and its
existing SDK/profile. JP builds and flashes from VS Code.

- DIAG_ENABLED=1.
- DIAG_WRITER_STACK_PSRAM=1 (experimental B, 8192-byte PSRAM stack).
- DIAG_TEST_HOOKS=0.
- Stage 0 probes unchanged, periodic interval 10 ms.
- Existing 20480-byte minimum-largest-internal-block gate unchanged.
- Web console: 115200, explicit DTR=true and RTS=false.

The committed switch defaults to 0 (A: 6144-byte internal stack). JP's local
working copy stays at 1 for ongoing B tests. Set 1 explicitly when reproducing
from a fresh checkout; do not mistake the default A configuration for this test.

1. Start from a normal shutdown and fresh B boot. Wait for green Wi-Fi/MQTT.
2. Send log status.
3. Request Latest once, return to the dashboard, send log status.
4. Turn the actual iPhone hotspot off for about 90 seconds. Confirm red offline
   and send log status while offline.
5. Turn the hotspot on; after green MQTT, send log status.
6. Inspect mqtt_connect before any further operation. Stop and send the capture
   if largest_min is below 20480 or a functional error occurs.

The original capture continued into a full Live cycle, providing additional
evidence. Continuing after the failed threshold is not necessary to reproduce.
Serial MQTT off/on commands do not substitute for the physical hotspot outage.

## Captured checkpoints (boot 14)

| Record | Result |
|--------|--------|
| 08:20:52.935 image_https | 798 ms, largest_min=26612, 79 samples |
| Latest complete | 35085 bytes, 1401 ms |
| 08:21:12.992 | Wi-Fi offline, MQTT disconnected |
| 08:22:32.794 preceding normal window | largest_min=32756, 3932 samples |
| 08:22:33.412 mqtt_connect | 618 ms, largest_min=14324, 62 samples, max gap 10273 us |
| 08:22:33.425 | Wi-Fi and remote MQTT connected |
| Following Live TLS windows | both largest_min=14324 |
| Full Live | largest_min=14324, 6034 samples, 170 frames in 60.3 s |
| Writer stack used / margin | 4204 / 3988 bytes, unchanged |
| Logger errors / queue drops | none / zero |

Logged offline-to-green interval: 80.433 seconds. Actual hotspot toggle times
were not recorded, so this is not an exact 90-second outage measurement.
Historical internal_min fell from 34944 to 22656 (12288 bytes). That arithmetic
does not establish one 12 KiB allocation. Video transferred more slowly than
earlier tests; its 2.8 fps does not prove a fragmentation-related slowdown.

## Controls already completed

- Internal-stack A reproduced the earlier 14836-byte Latest result.
- Fresh B Latest passed at 26612, with correct PSRAM stack/internal TCB placement.
- Seven full Live cycles passed in boot 13, including repeated cycles,
  G-meter and inclinometer navigation, ordinary NVS saves and Latest-to-Live.
- Those passing tests do not clear the later reconnection case.
- The overnight log contains a historical dip between 07:07 and 07:08 while
  surrounding snapshots are offline, followed by reduced current contiguous
  space at the 07:34 recorded reconnection. Short-outage reproduction does not
  identify the precise unobserved overnight event.

## Source clue and next control

In core 3.1.3, HTTPClient::end preserves its client when reuse is permitted.
The image completion path calls end; image-screen unload frees pixel/JPEG
buffers without explicitly stopping its secure client. New image preparation
and Live teardown do stop it. A retained image connection is a candidate
interaction, not a confirmed bug.

The fresh-boot no-media control completed in boot 15: mqtt_connect measured
31732 bytes over 753 ms with 75 samples, passing the floor. Historical memory
and stack minima were unchanged, with no logger errors or drops. No Live ran.
The notification interval was 46.719 seconds versus 80.433 in the failing test;
the control was not duration-matched and cannot prove prior media is required.

JP approved the experiment: explicitly stop the still-image secure client after
the complete body and httpClient.end, before decoding. The single stop call is
implemented, preserving the JPEG and normal Live keep-alive. JP built and tested
it in boot 17 with B and hooks off: Latest, return, hotspot off/on, green MQTT,
log status, then stop without Live. Latest passed at 25588 bytes and 1302 ms.
MQTT reconnect passed at 31732 bytes over 509 ms with 51 samples, versus 14324
before the patch. Historical internal_min stayed 36084 through reconnect.
Stack used/margin stayed 4204/3988; errors and drops were zero.

The observed offline-to-green interval was 67.234 seconds, not an exact measured
90-second outage. This first patched pass supports the cleanup; the overnight
allocation remains untraced. JP then completed full Live in the same boot:
TLS minima 26612 and 25588; full Live 25588; 196 frames in 60.3 s, first frame
1241 ms, maximum gap 905 ms. Video looked normal. Stack margin stayed 3988,
with no errors or drops. The largest-block minimum remained 25588 while
historical internal_min fell to 34688. Both Latest/reconnect and following Live
now pass the unchanged 20480-byte floor for this patched sequence.
A separate manual early-exit test in the same boot also passed, with no hotspot
interruption: 35 frames in 11.7 s, Live largest_min=26612, following Latest
largest_min=26612 and 1298 ms completion. Normal-window minimum recovered to
31732 between them; no errors, drops or stack-margin change. JP reported normal
operation.

JP next turned the hotspot off during Live in the same boot. Video stopped on
the expected fetch failure and returned to the dashboard. Live measured 26612,
successful MQTT reconnect 31732 (152 samples), then Latest 26612 and 1367 ms.
JP reported normal operation; no logger errors, drops or stack-margin change.
The historical largest-block minimum remained 25588. The separate immediate
MQTT window with zero samples is not evidence of handshake memory headroom.
This network-loss recovery check also passes. Back then passed in the same boot
at 26612 bytes, displaying 33949 bytes in 1427 ms. JP confirmed the image.
Historical memory minima and stack margin were unchanged, with no errors or drops.
The automatic camera still-to-Live test also passed: still and full Live measured
25588, both Live TLS windows 26612, with 195 frames in 60.3 s. The still displayed
for 1001 ms before Live started; first video frame took 1036 ms from Live start.
JP reported normal operation; no errors, drops or stack-margin change.
All targeted cleanup checks now pass in patched boot 17. Review the checkpoint
before the remaining same-session performance and hooks-phase gates.
See Stage 1 handoff for details; this does not accept all of Stage 1.

## Evidence and validation

- [Bench captures](sd_diagnostics_bench_results.md): dated results, serial evidence,
  passing controls, failed threshold and pending tests.
- [Overnight analysis](sd_diagnostics_overnight_analysis.md): field semantics,
  event timing, source checks and limits.
- [Original SD log](bench_data/sd_current_2026-09-16_0807.txt): byte-identical copy
  of JP's supplied file; hash recorded in the analysis.
- [Stage 1 handoff](../src/diagnostics/STAGE1.md): active test and later gates.
- [Accepted experiment review](sd_diagnostics_memory_experiment_review_claude.md).

JP compiled and flashed both A and B with hooks off. The assistant did not build
or flash. Hooks-enabled NVS stress is implemented but not yet hardware-tested.
Stage 0 probe files and network/media behavior remain unchanged by this
experiment. Checkpoint review used source inspection and git diff checks.
The memory floor remains 20480; Stage 1 is still not accepted.

Checkpoint 3aead88 includes the A/B switch, retained task placement/lifecycle/stack
status and hooks-only NVS stress, plus reviews and measurements through boot 14.
The later TLS-close experiment and boot 17 result are uncommitted follow-ups.
No push is requested.
