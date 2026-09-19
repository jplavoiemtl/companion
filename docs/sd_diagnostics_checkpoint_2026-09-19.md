# SD diagnostics checkpoint - 2026-09-19, end of day

## Resume here

JP has stopped for the day. Stage 1 is accepted; **Stage 1B acceptance remains
pending**. Stages 2-4 have not started. Do not resume the IMU investigation:
JP accepts the newer profile's approximately 42-43 Hz rate and reports normal
motion, G-meter and inclinometer operation.

JP restored core 3.3.11 with logging enabled, compiled and flashed from VS Code,
and reports approximately 42 Hz again. Local profile and flags agree. No raw
post-restore capture or new boot number was supplied; the restoration is JP's
confirmation, not a newly analyzed full gate run. Board power state is unknown.

## Current configuration

- Branch: `sd-diagnostics`; checkpoint includes the September 19 documented
  results and review reconciliation. Last runtime-code commit before this
  documentation checkpoint is `a675a76` (bounded USB debug TX waits).
- Profile: `amoled-1-8-core-3-3-11`, Waveshare graphics 1.6.4 and Adafruit expander.
- `DIAG_ENABLED=1`, `DIAG_TEST_HOOKS=0`, `DIAG_USB_TEST_FIXTURE=0`,
  `DIAG_WRITER_STACK_PSRAM=1`. Writer: 8192-byte PSRAM stack, internal TCB,
  priority 1, core 1, parks after cleanup. Stage 0 probes retained.
- Card returned to board. The synthetic fixture was deleted/pruned; archive 19
  is deliberate partial-header recovery evidence, not a disposable test fixture.
- JP builds/flashes. Remove the selected profile's
  `build/build_<profile>/sketch/companion.ino.cpp` before required rebuilds.
- Use the web console with DTR=true, RTS=false. Leave browser test switches off.
- Leave JP's untracked `docs/sd_iphone_log_download_plan.md` untouched.

## Evidence completed

See [bench results](sd_diagnostics_bench_results.md), especially the nine-gate
matrix and September 19 entries, for raw sources and limitations.

- Queue guard at 8/16: all eight injected records persisted; retry passed.
- Selected-reader pruning: synthetic archive removed, transfer ended cleanly,
  current-file retry passed. No synthetic archive from that case remains.
- Core 3.3.11 regression: interrupted rename, partial-header salvage, normal
  shutdown close, deep-sleep close and touch wake all pass.
- NVS/SD overlap: 278 commits, 736 SD records, zero errors, dummy key removed.
- Resource series: five confirmed aborts and five CRC-checked retries, same
  boot. Matching idle internal free memory 105488 bytes, largest block 57332,
  PSRAM 8339860 and writer stack margin 3304 all unchanged. Zero drops/errors.
- Physical-card check: USB current snapshot 130783 bytes, CRC32 9E5F4441,
  exactly matches the prefix of the 133346-byte card file. Additional records
  end with clean shutdown pending=0. No card writes performed by Codex.
- Earlier accepted transport evidence remains: small/current/bundle/2 MiB
  integrity, MQTT overlap, damaged-line recovery, abort/page close/battery
  unplug, 5-second stall, 120-second current deadline, progressing archive
  beyond 120 seconds, and fixture deletion. Reuse these results.

## IMU decision and accepted limits

Same-sitting three-window means: 3.3.11 logging off **42.27 Hz**, logging on
**42.67 Hz**, 3.1.3 logging off **49.25 Hz**. The approximately 14% drop persists
without logging. It is associated with the changed core/library profile;
no specific component is proven responsible. JP accepts it as-is. No writer
core A/B, added timing probes or optimization is planned unless symptoms arise.

Keep the **20480-byte** internal-largest-block gate and every transfer limit.
Keep JP's narrow roughly 8% Live FPS exception during simultaneous large USB
transfers; Live pacing stays reverted. Unsupported/bad-card tests remain
explicitly deferred. VS Code monitor-close freeze/reset predates the logger
and remains unresolved; the USB startup fix does not establish its resolution.

## Next session

1. Review remaining new-profile TLS/memory and performance acceptance evidence.
   Reuse passed tests. If a paired Latest/Live baseline is still missing, give
   JP only that focused case next, using unchanged Stage 0 probes. Any new
   performance comparison must be taken in the same sitting per CLAUDE.md.
   Do not use old-day numbers as a new paired baseline.
2. Present the remaining gate assessment and accepted limitations for JP's
   explicit Stage 1B acceptance. Do not silently mark the stage accepted.
3. After acceptance, implement Stage 2 network evidence: Wi-Fi events/profiles,
   every MQTT attempt, error snapshots before cleanup and TLS phase markers.
   Bench-test with existing off/on controls, one case at a time.
4. Later: Stage 3 operation context and optional log tail; Stage 4 car use
   and retention tuning. USB retrieval should avoid routine card removal.

This checkpoint changes documentation and the accepted-PSRAM config comment;
no runtime behavior change, build or flash by Codex. JP requested commit/push.
