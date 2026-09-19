# SD diagnostics checkpoint - 2026-09-18, end of day

## Superseded checkpoint

Resume from the [September 19 checkpoint](sd_diagnostics_checkpoint_2026-09-19.md).
JP restored core 3.3.11 with logging enabled and both test flags off, and accepts
its approximately 42-43 Hz IMU rate. Stage 1B acceptance remains pending.
The material below preserves the September 18 handoff and earlier plans;
do not use its board state or next-test instructions as current.

JP stopped for the day before testing the new queue and pruning controls.
Stage 1 is accepted. Stage 1B USB retrieval is substantially tested, but its
remaining safety checks and JP's acceptance are still pending. Stage 2 has not started.

Read CLAUDE.md, this checkpoint, and the first section of
[the Stage 1B handoff](../src/diagnostics/STAGE1B.md) before continuing.
Check Git status and the actual build flags; do not assume a new flash occurred.
Give JP one bench test at a time. JP compiles and flashes from VS Code.

## Code and board state

- Branch: `sd-diagnostics`. Last bench-validated normal-build checkpoint:
  `ad01fb8`. The newer commit containing this note includes the prepared
  queue/pruning controls; inspect Git log for its identifier.
- Last confirmed flashed firmware: normal build, boot 62, tested at 09:43.
  Its USB fixture and fault hooks were off, with the PSRAM writer enabled.
  The board's present power state is not reported.
- This checkpoint commit contains newer, **unflashed** queue/pruning controls,
  their source simulations, and documentation updates. These controls have not
  been compiled or bench-tested. The board still has the earlier normal build.
- Source defaults remain `DIAG_ENABLED=1`, `DIAG_USB_TEST_FIXTURE=0`,
  `DIAG_TEST_HOOKS=0`, `DIAG_WRITER_STACK_PSRAM=1`.
- Build profile: `amoled-1-8-core-3-3-11`, with the documented Waveshare library
  adaptations. Core 3.1.3 remains the rollback baseline.
- No companion.ino change was made for these controls, so this update does not
  require deleting its generated build intermediate. Follow CLAUDE.md's stale
  build rule if companion.ino is changed later.
- Leave the owner's untracked `docs/sd_iphone_log_download_plan.md` untouched.
  JP subsequently requested committing and pushing the prepared changes.
  Firmware compilation and flashing remain JP's responsibility.

## Today's confirmed results

- Explicit abort, page close, and battery-powered USB unplug all recovered with
  subsequent downloads. Keep USB-unplug testing on the battery-equipped board.
- Stopped-reader protection recorded exactly 5000 ms without progress, then
  resumed logging. A later ordinary current download succeeded.
- Sender-paced current download reached the firmware's 120000 ms deadline with
  recent progress, then cleaned up and allowed a normal retry.
- A progressing synthetic archive completed in 154.83 s, confirming it is exempt
  from the current-only overall limit. Its full 2097152 bytes were verified.
- `log test del 18` verified and deleted that fixture over USB. Archive 18 is gone;
  four managed files remained. No card removal was needed.
- Normal-build boot 62 downloaded current.log: 276169 bytes in 2.81 s, CRC OK.
  The saved file was independently checked: CRC32 0685E3ED. Logger ready,
  zero drops/errors, inactive and unpaused afterward, continued file growth.
  Internal heap minimum 92572 bytes; lowest largest block 49140 bytes;
  PSRAM writer stack margin 3400 bytes after retrieval. No performance A/B claim.

Detailed evidence and attachment references are in the
[bench results](sd_diagnostics_bench_results.md).

## Prepared controls, not yet bench-tested

`log test queue` arms one current download. After at least 1440 bytes, the writer
fills the real PSRAM event queue to half capacity with bounded synthetic events.
The existing logger_busy guard must stop retrieval, resume appending, and drain
those events without drops. This is not a fake queue count or a direct abort.

`log test prune N` is restricted to a complete synthetic archive successfully
created during the same boot. It shares the production close-reader-before-unlink
path. It keeps retention limits and real archives unchanged. Stage 1 supplies
retention-selection evidence; this test covers removal of an active reader's file.
A new disposable fixture will be required; do not assume its archive number.

Both controls compile out when the USB fixture flag is zero. Local validation
passed: 12 logger-gate source simulations, 16 USB guard/pacing simulations, and
19 browser checks (47 total). These are not firmware compilation or hardware tests.

## First test when JP returns

1. Temporarily set `DIAG_USB_TEST_FIXTURE=1` in diagnostics_config.h. Keep fault
   hooks at 0 and PSRAM writer at 1. JP builds and flashes the profile above.
2. Connect the web page with all browser test switches off. Keep Wi-Fi and MQTT
   connected and Live stopped. Send `status` for the starting state.
3. Send `log test queue`, then download current.log. Expect the intentional
   `Device: logger_busy` error; no partial file should be saved.
4. Send `status`. Expect the gate to report fired=1, result=injected,
   outcome=logger_busy, queued_at_test=8/16 and added greater than zero.
   Logging must return to ready, with no drops or errors and no append pause.
5. Download current.log again without rearming, then send `status` again.
   Expect CRC OK. Send the full console and downloaded file for checking the
   injected records and USB_GET_END evidence. Stop for review.

After that passes, give JP the disposable-archive pruning test from the handoff.
After both checks, review remaining current snapshot/prefix and resource-stability
coverage. Restore fixture=0, verify the resulting normal build as needed, and ask
JP to accept Stage 1B before implementing Stage 2. Reuse existing passed evidence.

## Preserved decisions and limitations

- Keep the 20480-byte internal-block gate, PSRAM writer, existing probes, 50%
  queue threshold, 5-second stall guard, 1-second sustained connection-loss guard,
  CRC validation, and current-only 120-second deadline unchanged.
- Live-specific USB pacing remains reverted. JP accepted the rare large-download
  overlap FPS cost as a narrow exception, not a general performance waiver.
  The [September 17 checkpoint](sd_diagnostics_checkpoint_2026-09-17.md) records
  the paired measurements and decision. Do not reintroduce the experiment.
- Closing the VS Code monitor can still freeze or restart the board; explicit
  DTR=true, RTS=false in the web page is the tested connection setting.
- Core 3.3.11 remains a trial. Its normal IMU rate is about 42-43 Hz versus
  historical roughly 49 Hz; these are not same-session controlled comparisons.
- Earlier unexplained stalls are retained as history; later successful transfers
  do not establish that every USB or core issue is resolved.

## Main steps remaining

1. Complete and accept Stage 1B as above.
2. Stage 2: Wi-Fi and MQTT event evidence, attempt/error snapshots and TLS markers.
3. Stage 3: still/Live lifecycle, buttons, power/motion and cause-tagged loop gaps;
   optional log tail afterward.
4. Stage 4: car use, incident-log review and retention tuning from measured growth.


## Agreed remaining regression order (2026-09-19)

Stage 1 acceptance on 3.1.3/core 0 is preserved. Reuse passed results but check
storage recovery, both close paths and NVS/SD overlap on 3.3.11/core 1.
Then run approximately five aborted and five successful downloads with matching
idle current-memory readings, and download current before power-off/card-copy
prefix comparison. No new broad fault campaign or unsupported-card tests.

In one sitting, record three normal 60-second IMU windows per configuration:
3.3.11 logging off, 3.3.11 logging on, 3.1.3 logging off. Delete the selected
profile's generated companion.ino.cpp when switching. Compare writer core 0
versus 1 only if the 3.3.11 logging-on leg explains the drop. Historical boot 45
already had core-1 writer and 48.63 Hz IMU, so core placement alone is unproven.
The controlled SDK comparison must also close the outstanding paired Latest/Live
TLS memory/performance coverage. Keep all Stage 0 probes and the 20480-byte gate.

Restore fixture=0 and hooks=0 before normal-build acceptance. Record the known
VS Code monitor-close limitation explicitly. JP accepts Stage 1B before Stage 2.
