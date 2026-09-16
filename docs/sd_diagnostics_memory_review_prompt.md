# Claude review request: SD diagnostics internal-memory margin

Please independently review the Stage 1 memory problem on branch `sd-diagnostics`,
checkpoint `54baf96`. Read `CLAUDE.md` first. This is analysis only:
do not change firmware, build, flash, commit, or push. JP builds and flashes in VS Code.
Challenge Codex's interpretation where the evidence warrants it.

## Files to read

- [Bench results](sd_diagnostics_bench_results.md): especially the no-card comparison,
  failed writer-start-order experiment, and 2026-09-15 17:29 retained-snapshot test.
- [Accepted diagnostics plan](sd_diagnostics_plan.md): memory decisions, Stage 1 gate,
  external-stack constraints, and pinned SDK appendix.
- [Stage 1 implementation and handoff](../src/diagnostics/STAGE1.md).
- [Writer, storage, and retained measurements](../src/diagnostics/sd_diagnostics.cpp).
- [Clock and RTC code](../src/diagnostics/diagnostics_clock.cpp).
- [Probe implementation](../src/diagnostics/diagnostics_probes.cpp) and
  [probe definitions and limitations](../src/diagnostics/README.md).
- [Configuration](../src/diagnostics/diagnostics_config.h) and
  [internal structures](../src/diagnostics/diagnostics_internal.h).
- [Startup and shutdown](../companion.ino), [screen memory](../src/screen_memory/screen_memory.cpp),
  and [calibration NVS operations](../calibration.cpp).
- The locally installed core and SDK, local sketch.yaml, and
  build/build_amoled-1-8/sdkconfig. Trace image and Live TLS calls under src/image/
  and src/video/ if useful.

The build is Arduino ESP32 3.1.3, IDF release v5.3-489d7a2b-v1.
Installed sources are under %LOCALAPPDATA%/Arduino15/internal/:
- esp32_esp32_3.1.3_e149c3cd368ed269
- esp32_esp32-arduino-libs_idf-release_v5.3-489d7a2b-v1_80ffc9027a/esp32s3

Use those exact sources and configuration where available. If an implementation
is available only as a compiled library, distinguish verified source from inference.

## Evidence to assess

- Largest free internal block during Latest HTTPS repeatedly reaches 14836 bytes
  with logging ready. The agreed gate is 20480 bytes. Images still succeed.
- With no card, it reached 28660 bytes. In that case the failed writer releases SD
  resources and exits; this comparison does not isolate one allocation.
- Moving writer startup after hardware/UI initialization but before Wi-Fi did not help.
- Latest's logger write count and file size did not change during the failing runs.
- Latest retained snapshots: clock interval net internal reduction 5580 bytes;
  writer creation to entry 6528 bytes; explicit PSRAM formatter zero;
  mount interval 22304 bytes; post-mount preparation 14964 bytes;
  first writable current.log open zero.
- Wi-Fi startup overlaps writer mounting. These are sequential whole-system heap
  readings, not ownership traces. configTzTime also calls esp_netif_init.
- Writer stack is 6144 internal bytes on core 0, priority 1. Minimum reported unused
  stack is 1988 bytes. The 240-byte retained-snapshot array is additional internal storage.
- Stage 0 probes remain unchanged. Latest's last run had 74 samples at 10 ms,
  a maximum sample gap of 10217 microseconds, and total display time 1301 ms.
- Queue and formatter explicitly use PSRAM. The possible transient internal DMA
  buffer for SD transfers is still unverified.
- Recorded SDK settings include external-stack support, FATFS external allocation
  preference, per-file cache, stack LFN workspace, 4096-byte sector support,
  internal mbedTLS allocation, and internal preference for allocations <=4096 bytes.
  Verify them rather than treating configuration support as proof of safety.

## Questions

1. What do the results establish, and what remains unproven? Is the writer stack
   the best next target, or are there simpler and safer persistent allocations to address?
2. Audit whether this particular writer can safely use a PSRAM stack with the
   pinned SDK. Cover the creation API, internal task-control requirements, deletion
   and cleanup, ROM calls, cache-disabled flash/NVS operations on either core,
   SDMMC/DMA use of stack-local buffers, filesystem calls, formatting/time calls,
   error recovery, rotation/pruning, and future USB retrieval. A configuration
   option permitting external stacks is not sufficient evidence.
3. Assess keeping the internal stack and reducing stack use or other resident
   allocations instead. Check peak call paths and watermark limitations. Do not
   assume 4096 bytes is safe when measured use already exceeds it.
4. Check whether the largest-block metric and its capability mask represent the
   actual TLS allocation risk. Distinguish the agreed gate failure from an observed
   allocation failure. Do not silently lower the gate; explain any proposed change
   for JP's explicit decision.
5. Rank the smallest safe options by evidence, expected memory benefit, and risk.
   Returning free bytes does not necessarily enlarge the largest contiguous block.
6. Recommend ONE next correction or diagnostic experiment with a concrete bench
   pass/fail test. JP attaches Web Serial after startup, so any startup evidence
   must remain retrievable with log status. Preserve same-session comparisons.

Reply with a concise source-backed assessment, separate confirmed facts from
hypotheses, and finish with your preferred next action. No implementation yet.
