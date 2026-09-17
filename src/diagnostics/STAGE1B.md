# Stage 1B USB retrieval - first bench handoff

JP accepted Stage 1 on 2026-09-17. Stage 1B source is prepared, **not built or
hardware-tested by the assistant**. Its gate remains pending.
Stage 1 evidence and acceptance were pushed in 3d0b125 and 3f5b309.
JP requested committing and pushing the pre-flash checkpoint. Core 3.1.3 download attempts failed; JP has now compiled the core 3.3.11 trial successfully. First hardware validation of that trial is pending. See [current trial instructions](../../docs/core_3_3_11_trial.md) before following the original handoff below.

## Prepared implementation

- diagnostics_usb.h/.cpp is serviced only by the existing writer for filesystem
  work. The main parser posts bounded requests. No second SD task or USB port.
- log list, log get current, log get <generation>, log abort, and bounded log status.
  Status gives @@STATUS plus @@USB. The ordinary status command retains the
  full Stage 1 memory/stack snapshot and also schedules these bounded lines.
- Strict managed filenames, bounded directory scan, PSRAM transfer/list buffers.
  A reader is closed before pruning its archive. Logging batches retain priority.
- Current retrieval writes USB_GET_BEGIN, flushes/closes append, snapshots size,
  reads exactly that size, closes the reader, reopens O_APPEND, then resumes.
  USB_GET_END is queued after cleanup with bytes, duration and result.
- At most four complete protocol lines per writer turn. Idle turns remain 20 ms;
  active retrieval yields one tick. Writer memory sampling stays at about 20 ms;
  the unchanged Stage 0 sampler remains 10 ms. Normal IMU probes exclude transfers.
- Limits: 144 data bytes per base64 chunk; 240-byte maximum wire line;
  queue occupancy 50%; five seconds without a complete transfer line;
  current-only 120-second overall deadline from request acceptance.
  Archives have no overall deadline. Connection, pruning, read failures and
  shutdown also abort. Cleanup restores appending before error output.
- No periodic acknowledgments or transfer IDs. After cancellation, the page waits
  for aborted confirmation; after eight seconds without it, it closes the port.
- Browser validates version, name, sequence, strict base64, byte count, line count
  and CRC-32/ISO-HDLC before saving. A 64 MiB browser allocation ceiling bounds
  exceptional oversized files. Console and partial-line storage remain bounded.
- Page adds file listing, single and current-plus-newest-three downloads, progress,
  cancellation and damage/pause/slow-read bench switches. The tested explicit
  DTR=true/RTS=false combination is preselected; driver-default comparison remains.
  Firmware USB mode, buffers and timeout were not changed.

Configured source: DIAG_ENABLED=1, DIAG_WRITER_STACK_PSRAM=1,
DIAG_TEST_HOOKS=0, build tag stage1b-usb.
No Stage 2 network hooks or Stage 3 tail command have been added.

## First test only: one known archive

1. Safely eject the card from Windows. Insert it with the companion fully off.
   Turn the iPhone hotspot on and connect USB.
2. JP compiles and flashes from VS Code. Because companion.ino changed, delete
   build/build_amoled-1-8/sketch/companion.ino.cpp before rebuilding.
   The assistant removes the existing intermediate once before handoff;
   repeat this precaution if it is regenerated before another changed-sketch build.
3. Open or reload tools/sd_log_browser.html. Leave every bench checkbox OFF.
   Connect with explicit DTR=true, RTS=false after normal startup. The page
   requests log status and log list. If attached too early, reconnect after setup.
4. Send status once for the full memory/stack snapshot.
5. Download archive-00000014.log from the table, once. Expected: **8059 bytes**,
   CRC **99C24CB1**, then a Verified message and a normal browser download.
   Filename uses the new boot number followed by -archive-00000014.log.
6. Send status again. Save the console and tell us the downloaded file's path,
   so its bytes can be compared against the already saved reference.
   Report whether the board stayed responsive.

Do not start current-file, bundle, slowdown, damage or disconnect tests yet.
No more card removal is needed for this first comparison: its reference is
[the saved archive](../../docs/bench_data/sd_logs_2026-09-17_offline_rotation/logs/archive-00000014.log.txt).
A verified CRC checks transport integrity; the reference comparison checks the
correct file bytes independently.

## Validation performed here

Node tests execute the actual inline page protocol and a mocked DOM/serial shell.
Thirteen checks pass: empty/known CRC/binary files; invalid BEGIN/path/version/size;
base64/sequence errors; END mismatch/truncation; list parsing; the real archive
through split reads and debug text; damaged-line abort barrier and retry;
missing END; console clear during a partial line; oversized protocol rejection; stray data and busy replies during a transfer.
Run: node tools/tests/sd_log_browser.test.cjs.
Firmware review checked the installed core 3.1.3 HWCDC mutex, available-space and
write behavior. No firmware compile, flash or claim of on-board validation.

## Later gate work, one test at a time

Follow the main plan's Stage 1B gate after the small archive comparison.
Current snapshot comparison, a 2 MiB throughput measurement, the three-times
current deadline margin, same-session Live/MQTT and memory/stack checks, validation,
abort/liveness, queue-pressure, prune-selected-archive and repeated retrieval
still need hardware results. Hooks remain off for the initial ordinary-use test;
later hook-dependent tests get separate instructions.
The slow-read switch requires a sufficiently large file; USB/OS buffering can
absorb a small download before its effect is observable. Firmware timing and
actual byte throughput determine the test result.
The previously observed VS Code monitor-close freeze remains unresolved.


## First-download follow-up: stall diagnostics

The initial on-board attempts rejected a malformed line, then stalled after
partial transfer. See the 11:11 and 11:17 entries in the bench results.
The cause is still under investigation; this gate has not passed.

The page retains bounded evidence for rejected protocol lines.
The firmware now adds phase, bytes, line_bytes, tx_free and write_bytes to
stalled replies. A value of -1 means the corresponding space check or write
was not reached on the last transfer send attempt. Limits remain unchanged.
JP rebuilds and flashes this diagnostic version, then repeats only archive 14
with all browser test switches off. Send status afterward and retain the console.
Thirteen browser checks pass. Firmware compilation and hardware validation
remain JP's next step.


## 11:25 follow-up: same-core USB writer trial

JP captured STATUS and FILE text merged together; the page disconnected after
rejecting the incomplete list. See the bench results for the pinned driver evidence.
The prepared trial pins the existing writer to the setup/USB interrupt core,
currently core 1, instead of core 0. Both stack modes use this affinity.
Status adds writer_core so the flashed trial can be identified. All other
writer settings and transfer limits remain unchanged.

JP rebuilds/flashes, reconnects, sends status, downloads only archive 14, and sends
status again. Send the complete console and any downloaded file path.
If the trial fixes corruption, Live performance and memory need a same-session
recheck before accepting Stage 1B. The cross-core race remains a hypothesis until tested.
