# Stage 1 acceptance checkpoint — 2026-09-17

**JP accepted the tested Stage 1 scope on 2026-09-17 and authorized Stage 1B.** This checkpoint supersedes older "next test"
and "not yet passed" statements in the chronological bench history.
JP builds and flashes; no firmware, card contents or Git history changed during
this final inspection.

## Evidence reviewed

| Area | Recorded result |
|---|---|
| Internal memory and media | Internal writer stack failed at 14836 bytes. PSRAM writer plus still-HTTPS cleanup passed subsequent exercised media/TLS paths above the unchanged 20480-byte largest-block floor. |
| Same-session performance | September 16 logging-off/on pair: Live fps -3.4576%, within 5%; logging-on measured block minima 26612 bytes. No normal-use queue drops. |
| PSRAM writer validation | Internal TCB and external stack placement verified. NVS/SD overlap stress and terminal cleanup tested. Writer parks after cleanup; it does not free its static task storage. |
| Append and recovery | Repeated boots, corrupt/partial header salvage, missing/empty current, interrupted rename/create and short incomplete-tail repair checked. Original tail prefix preserved. |
| Storage limits | Natural rotation, pruning, no-card and injected full-write failure tested. Offline final run: 3 rotations, 2 prunes, 61 continuous records and no errors/drops. |
| Protected files | All six fixtures match original names, sizes and hashes after pruning, including images-folder data and an archive-like directory. |
| Clock and reset context | Unknown/approximate/synced states, corrections, both Montreal DST transitions, panic/watchdog reset classes and retained breadcrumbs checked. |
| Power paths | Normal shutdown and deep-sleep close records have pending=0. JP confirmed touch alone woke the board; matching reset, breadcrumbs and approximate-to-synced recovery were verified on card. |

Final evidence is in
[offline rotation backup](bench_data/sd_logs_2026-09-17_offline_rotation/README.md).
[Full bench history](sd_diagnostics_bench_results.md) includes earlier captures,
failed experiments and the same-session performance pair.

## Scope limits and known issues

- JP explicitly deferred bad/unsupported-card tests for version 1. They are untested.
- Brownout breadcrumb survival, arbitrary power-loss durability and near-cap
  incomplete-tail recovery remain unverified.
- Memory block minima are sampled at 10 ms; the heap minimum is since boot.
  The tests do not prove every allocation ordering or PSRAM/DMA path safe.
- VS Code serial-close freezing is an unresolved, separately recorded issue.
  The web console's tested DTR=true, RTS=false configuration is the bench route.
- Small-limit hooks must not be applied to an already large current file.
  An earlier deliberate-limit run pruned backed-up archives then disabled
  logging with reserve_exhausted. The subsequent guarded tests passed.
- Physical full-media behavior was not exhaustively tested; the write-failure
  hook and no-card path provide the recorded failure coverage.

## Authorized next step

Keep the validated PSRAM writer configuration. Turn DIAG_TEST_HOOKS off in the
next ordinary build; it is still 1 in the current test source/firmware.
Implement Stage 1B USB retrieval on the existing web console and writer owner,
following the accepted main plan: list/download/abort, framed lines with integrity
checks, paced output, pause current appends only when required, bounded abort
handling and updated status. Do not add Stage 2 network hooks yet.

JP then builds/flashes and runs the Stage 1B tests one at a time, starting with
listing and a small download. The 2 MiB throughput, current-file pause limits,
integrity/abort tests and normal operation checks follow the plan.
No additional Stage 1 runtime test is proposed before acceptance.
The card may be safely ejected and reinstalled with the board fully off;
leave the harmless preservation fixtures in place.
