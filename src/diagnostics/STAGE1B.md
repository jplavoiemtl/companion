# Stage 1B USB retrieval - first bench handoff

Current handoff: [checkpoint](../../docs/sd_diagnostics_checkpoint_2026-09-19.md).
JP accepted Stage 1. Stage 1B is substantially tested; acceptance remains pending.
September 19 queue/pruning, storage/close/NVS, resource and card-prefix cases
pass. JP accepts 42-43 Hz IMU operation and restored 3.3.11 logging on with both
test flags off. Final TLS/performance evidence review remains. Earlier sections
preserve bench history and test instructions, not the current next action.
JP builds and flashes; the assistant has not built or flashed these changes.

## 2026-09-18: remaining queue and pruning gate controls

Queue gate passed on boot 67 on September 19: 8/16 triggered logger_busy,
eight saved test records verified, ordinary retry CRC OK, zero drops.
Selected-archive pruning also passed on boot 67: fresh fixture 18 removed,
pruned=1, normal current retry CRC OK. Continue with the bounded SDK regression. Normal source defaults
`DIAG_USB_TEST_FIXTURE=0`. Set it to 1 for this separate bench build; retain
`DIAG_TEST_HOOKS=0` and `DIAG_WRITER_STACK_PSRAM=1`. Use profile
`amoled-1-8-core-3-3-11`. These edits only touch src files, so there is no new
companion.ino intermediate to delete for this change.

Leave all browser test switches off, keep Wi-Fi and MQTT connected, and stop
Live. No new task, allocation, retention limit or transfer timeout is added.
Both controls fire once after at least 1440 file bytes have been sent.
Listing preserves the arm. Downloading the wrong file consumes it without
injection. Ending a transfer clears its active control. Reset clears both.
`log test usb off` disarms while idle; during transfer use `log abort` first.
`status` includes `[LOG USB GATE]` with armed, active, target, fired, added,
queued_at_test, result and outcome. Normal builds omit these controls and fields.

### First test: current-file queue pressure

1. Build and flash the bench configuration above. Connect the web page and send
   `status` to capture the starting state.
2. Send `log test queue`. Expect `armed=queue result=armed`.
3. Download current.log. Expect `Device: logger_busy`, with no partial file saved.
4. Send `status`. Expect `fired=1`, `result=injected`, `outcome=logger_busy`,
   `queued_at_test=8/16`, and added greater than zero. Existing events are preserved.
   The real queue is filled only to half capacity with USB_QUEUE_TEST records;
   the unchanged production guard must abort and reopen appends. Queue high-water
   may exceed eight when USB_GET_END is enqueued, but drops must stay zero.
5. Download current.log again without rearming, then send `status` again.
   Expect CRC OK, ready, active=0, paused=0, no logger error and no drops.
   Send the full console and the downloaded file for checking the injected
   records and USB_GET_END evidence. Stop here for review.

### Later test: selected synthetic archive pruning

After the queue result is reviewed, in the same bench build:

1. Send `log test file` and wait for result=ok. Use the archive number printed;
   do not reuse the old number 18 by assumption. Refresh the file list.
2. Send `log test prune N`, replacing N with that new number. Only a complete
   synthetic fixture successfully created during this boot is accepted.
3. Download that archive. Expect `Device: pruned` and no saved partial download.
4. Refresh files and send `status`. The fixture alone must disappear, pruned
   must rise by one, and the gate must report result=pruned and outcome=pruned.
   Logger readiness, zero drops and ordinary files must be preserved.
5. Download current.log normally, then send `status`. Send console and file.

The hook rechecks fixture ownership, regular-file type and exact 2 MiB size.
It uses the production removal helper, which notifies USB to close the reader
before unlink. It does not lower retention limits or choose a real archive.
This covers the active-reader removal path; it does not recreate the retention
threshold selection already exercised in Stage 1. The fixture is removed by
this test, so no card removal or separate deletion is needed after success.

Local checks: 12 new logger-gate source simulations, 16 USB guard/pacing
simulations and 19 browser checks pass. Firmware compilation and bench results
remain JP's next steps. No commit or push performed.

## 2026-09-18: temporary sender-paced deadline test

Bench update at 09:19: JP's boot 60 passes the current deadline at exactly
120000 ms, cleanup, automatic test reset and ordinary retry (260449 bytes,
2.57 s, CRC OK). The procedure below is retained as the reproduction recipe.
Archive 18 also passed at 09:25: 154.83 s total, all 2097152 bytes independently
verified, logging continued. USB deletion of archive 18 then passed at 09:30.
Source now defaults DIAG_USB_TEST_FIXTURE=0; enable it explicitly to reproduce
these tests. The running boot 60 still has it enabled. See the checkpoint and
bench results for remaining checks; Stage 1B is not yet fully accepted.

JP authorized this control after two browser-throttled tests ended on other
USB guards before 120 seconds. We are testing the firmware's current-file pause
limit, not a browser timeout. The normal browser reader must keep draining USB.

With DIAG_USB_TEST_FIXTURE=1 (already enabled for the synthetic archive), use:

- `log test slow on`: while idle, arm the next file download for one successful
  data line every 100 ms (at most 1440 file bytes/s). Listing does not consume it.
- `log test slow off`: disarm it, or restore normal speed during a download.
- `status`: includes `[LOG USB TEST] slow_armed=... slow_active=... data_interval_ms=100`.

The test defaults off at boot. Success, failure or cancellation of the file
transfer switches it off; shutdown clears it too. It is absent when the fixture
flag is off. It uses no new task, allocation or blocking delay. All safety checks
run before the pacing wait; only complete successful data lines count as progress.
No Live-specific policy has been restored. The 120000 ms current deadline,
5000 ms stall limit, 1000 ms connection-loss guard and 50% queue rule are unchanged.
The retained failure snapshot now includes timeout elapsed/idle times and bytes.

Next single test for JP:
1. Build and flash the core 3.3.11 profile with fixture=1, fault hooks=0 and
   PSRAM writer enabled. Only src files changed; companion.ino was not edited,
   so this update does not require deleting its generated build intermediate.
2. Reload and connect the web page with all three browser test switches off.
   Keep hotspot and MQTT connected, Live stopped, and the card installed.
3. Send `log test slow on`. Wait for `slow_armed=1 slow_active=0` in its reply.
4. Download current.log (currently about 250 kB, larger than the roughly
   172800 bytes this test can send in 120 seconds). Expect `Device: timeout`
   at about two minutes. A rejected partial download must not be saved.
5. Send `status`. Expect result=timeout, active=0, paused=0, slow_armed=0,
   slow_active=0, logger ready and zero drops. Retained elapsed_ms should be
   around 120000, with recent progress. Then download current.log normally
   without rearming, and send `status` again. Paste the full console.

If a different guard fires, report it rather than repeating or changing limits.
Do not delete archive 18 yet. A later separate test will check a paced archive
past 120 seconds, then use slow off to finish at normal speed.

Local validation: 16 connection/pacing source simulations and 19 browser checks
pass. These are not a firmware build or hardware validation. JP builds/flashes;
no commit or push is authorized by this change.

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


## Retained USB stall diagnostics (2026-09-17 follow-up)

JP authorized a diagnostic-only firmware change after the normal retry failed
in 1054 ms, too soon to be the five-second no-progress timeout.
`status` and `log status` now append two ordinary console lines after @@USB:

```text
[LOG USB FAIL] valid=0 at_ms=0 elapsed_ms=0 idle_ms=0 phase=none path=none
[LOG USB SEND] file=none bytes=0 line_bytes=0 tx_free=-1 write_bytes=-1 check=none stop=none
```

With valid=1 these describe the most recent stalled transfer, captured before
cleanup and independently of whether the original error reply arrives.

- at_ms is device uptime at the failure. elapsed_ms is time since request
  acceptance; idle_ms is time since the last complete line was enqueued.
- path=stop_guard identifies the normal limit check. path=send_failed
  identifies a negative send result. path=stop_recheck identifies the
  pre-write guard, including its original stop reason if it later clears.
- check identifies the last send step: length, connected_before_space,
  space, connected_after_space, stop_recheck, complete or short_write.
- tx_free is the last availableForWrite return; write_bytes is the last
  write return. -1 means that operation was not reached. line_bytes is the
  requested complete wire-line length. bytes counts completed file payload.
- The snapshot survives listing, successful retry, late abort and serial
  reconnection. Another stall replaces it; reboot clears it. A zero write
  return alone cannot distinguish the driver's internal reasons.

Storage is a fixed writer-owned RAM struct; no dynamic allocation, new task,
per-frame print or SD record. The transfer protocol, cleanup order, USB settings,
limits and Stage 0 probes are unchanged. Only explicit status output grows.
The page's requested introductory paragraphs and bottom bench instructions
were removed. Signal controls remain; unsupported Web Serial reports an error
in the console. Existing browser checks pass.

JP builds/flashes profile amoled-1-8-core-3-3-11, keeps hooks=0 and PSRAM writer
enabled. This change is in src/ only; companion.ino was not edited.
First test: reload the HTML page, connect with the established explicit
signals, send status, download current normally, then send status again.
Paste the entire output, especially both new LOG USB lines. If this passes,
repeat the paused-reader/recovery sequence with separate instructions.
No firmware build or flash performed by Codex; hardware results pending.


## 2 MiB synthetic archive and safe serial deletion

JP authorized creating and later deleting the test archive without removing
the card. This bench build sets DIAG_USB_TEST_FIXTURE=1 and leaves
DIAG_TEST_HOOKS=0 and DIAG_WRITER_STACK_PSRAM=1. The fixture flag must return
to 0 for ordinary builds after testing. It is separate from panic, watchdog,
clock and storage fault hooks; enabling both hook flags is rejected at compile
time. No automatic fixture is created at startup.

Commands (type in the web console):

- `log test file`: create one synthetic 2097152-byte archive. Wait for
  `[LOG FIXTURE] result=ok archive=NNNNNNNN bytes=2097152 errno=0`.
- `status`: includes `[LOG FIXTURE]` with active, archive, byte count, result
  and errno while this bench flag is enabled.
- `log test del 18`: example deleting archive 18. Substitute the actual
  number printed by creation. Wait for result=deleted, then Refresh files.
  This is NOT a general log-delete command. It refuses wrong sizes or any
  byte that differs from the synthetic pattern, including normal SD logs.

Implementation and ownership:

- Existing writer owns every file operation. It writes or validates only
  1024 bytes per turn, using the existing PSRAM formatter and normal 20 ms
  idle yield. Events, clock work and flushes retain priority. No new task,
  PSRAM allocation, NVS write or transport setting is introduced.
- Creation uses O_EXCL on /logs/usb-fixture.tmp. It refuses an existing temp
  file, insufficient free space or retention headroom rather than pruning
  diagnostic evidence for test data. It reserves an archive generation
  above both current generation and archive high-water mark.
- The complete file is fsynced, closed, checked for destination collision,
  then renamed to archive-NNNNNNNN.log and inventoried. Generation gaps are
  expected; current.log stays open and is not replaced or padded.
- Each of 32768 64-byte lines starts USB_TEST_FIXTURE line= followed by an
  eight-digit zero-based index and a space, dots to byte 62, then LF.
  These are labelled synthetic records, not normal diagnostic event records.
- The test archive participates in normal archive listing and retention.
  Creation queues USB_TEST_FIXTURE in current.log; deletion queues USB_TEST_DELETE.
- Listing and downloading are refused while fixture creation/validation is
  active; status remains available. Start only when no USB transfer is active.
- Deletion opens only an exact numeric archive path, checks size, compares
  every byte in bounded batches, closes its reader and then unlinks it.
  Normal pruning closes a fixture-validation reader before unlinking.
- Shutdown cancels pending work and closes its handle. Failed creation removes
  only the temp file exclusively created by this invocation. Failed validation
  preserves the target. A pre-existing temp left by abrupt power loss is
  preserved and causes temp_open_failed; report it instead of repeatedly trying.
- Generation and validation have a 120-second operation deadline checked
  between batches. This is separate from USB download limits. Creating or
  verifying for deletion should take roughly a minute at 1 KiB per 20 ms turn;
  status reports progress. Do not benchmark Live while generating/deleting.

Known reference: size 2097152, CRC32 **8D218D21**, SHA256
`b79a649116ba358243b2c9388b68ac718b9f65cef94f241236ad8550394f65be`.
`python tools/verify_usb_fixture.py <downloaded-file>` independently compares
every byte on the computer. No card-reader copy is needed for this synthetic
fixture; this does not replace the real current.log prefix integrity test.

### Next bench test only

1. JP compiles/flashes amoled-1-8-core-3-3-11. Prepared flags above need no edit.
   Keep the SD card installed. No companion.ino changes, so this src-only
   update does not require deleting its generated .ino.cpp.
2. Reload the web page. Connect with explicit DTR=true, RTS=false. All browser
   fault switches off; hotspot on; Live stopped. Send status.
3. Send log test file once. Wait for result=ok and retain the archive number.
   If it fails, stop and paste the console. Do not send the command repeatedly.
4. Click Refresh files and download the new archive showing 2097152 bytes.
5. Send status and paste the console plus the saved file path. Report whether
   the board stayed responsive. Keep the archive for the remaining large-file
   tests; log test del <number> will remove it when done.

Acceptance here: CRC and independent expected bytes, duration at most 40 s
for the initial 120 s >= 3x measured transfer-time margin, no unexpected
stall/reset/drop or memory/stack failure. If throughput requires a longer
current limit, discuss and set that before accepting the gate. A successful
2 MiB archive does not validate current-file timeout or MQTT overlap by itself.

Desktop verification: 14 existing/extended browser checks pass, including
full 2 MiB sequencing and CRC; independent Python verifier accepts the
reference and rejects a changed last byte, truncation and ordinary log text.
Writer code reviewed for exclusive create, bounded work, collision checking,
shutdown cleanup and delete validation. Firmware was not compiled or flashed
by Codex; on-board generation and deletion are still unverified.


## Connection-loss tolerance prepared after boot 51

JP approved this correction. The writer stops sending immediately on a false
USB connection reading, but aborts as disconnected only after 1000 ms of
continuously observed loss. Recovery before that resumes the pending line.
The five-second no-progress deadline remains active during loss or flapping;
connection checks do not reset progress. Current retains its 120-second limit.
Archives still have no overall time limit. Abort, queue pressure and shutdown
checks continue to run. No sleep, new task or per-line serial output was added.

Both pre-write connection checks now return retry without writing on a false
reading. A final stop check that observes loss also prevents the write.
Control replies use their existing bounded retry period for transient loss.
Short or failed writes still terminate the transfer; they are never retried
as a whole line, which would risk duplicate or corrupt protocol data.
A connection change inside the core's write call remains possible; this
change cannot make connection checking and driver writing atomic.

Retained LOG USB FAIL now covers stalled and disconnected outcomes and adds
reason and loss_ms. LOG USB LINK reports losses, max_loss_ms, pending and
grace_ms=1000 for the latest transfer. These observations reset at the next
list/download, so request status before Refresh files or another download.
The failure snapshot survives successful retries, listing and late aborts.
A sustained disconnect still closes the reader and resumes current appends
before release; no reply is attempted while classified disconnected.

Desktop validation: ten source-level guard simulations and fourteen browser
protocol checks pass. Simulation covers brief loss, continuous loss, loss
at each pre-write check, flapping, unchanged deadlines, abort/queue/shutdown,
short writes and control replies. This is not a C++ firmware build or a
hardware validation. JP builds and flashes from VS Code. No commit or push.

Next bench test: build/flash the 3.3.11 profile, keep the card installed and
archive18 unchanged, connect with explicit DTR=true RTS=false, all browser
fault switches off, hotspot on and Live stopped. Download archive18 once,
then send status before any refresh/retry. Send the whole console, saved-file
path if successful, and whether the board stayed responsive. Do not regenerate
or delete the fixture yet. Only src changed, so this update does not require
removing the generated companion.ino.cpp. Large-file acceptance remains CRC
and independent byte verification, no unexpected failure, and at most 40 s
for the initial 120-second current-file deadline's three-times margin.


## Hardware checkpoint: 2026-09-17 15:56

Fixture creation and two full 2 MiB downloads now pass, with independent exact
byte verification. Ordinary transfer 20.39 s; MQTT-overlap run 22.67 s includes
waiting for the command to be processed after a blocked main-loop connect.
Completion occurs during the next blocking MQTT attempt. Connection-loss
observations 3 ms and 4 ms recover; zero logger drops/errors. MQTT restores normally.
Keep the 120-second current deadline (required 3x margins: 61.17 s and 68.01 s).

This is a bench checkpoint, not Stage1B acceptance. Remaining cases include
large Live overlap, remaining abort/current pause and timeout tests, and
on-board fixture deletion. Leave fixture 18 installed and DIAG_USB_TEST_FIXTURE=1
until these finish; then use log test del 18 and restore flag 0. Do not rerun
fixture generation. See docs/sd_diagnostics_bench_results.md for measured status.


## Historical Live-only pacing trial (reverted at JP request, 2026-09-17)

The A/B/A runs measured 3.021 fps without download, 2.766 during a 2 MiB
transfer, then 3.002 without download. JP saw normal video but the concurrent
run missed the 5% target. JP approved an easy adjustment only.

While USB is active and Live is running, the existing writer now sends at
most one transfer-protocol line per turn and yields for three scheduler ticks
(pdMS_TO_TICKS(3)). The installed 3.3.11 sdkconfig has FREERTOS_HZ=1000.
Outside Live it keeps four lines and one tick. Non-USB idle turns remain 20 ms.
Both ordinary archive downloads and the current.log append-pause path use
this pacing. State is rechecked each turn, restoring ordinary speed after Live.
The Live boolean is now atomic so the writer can safely read it; the accessor
publishes no video buffers or other video state. No new task, buffer or priority.
Stage 0 probes, connection-loss tolerance, queue/abort/shutdown checks, CRC,
5-second no-progress and 120-second current deadlines remain unchanged.
Status/error output is not counted as file-data lines and retains its existing
bounded control handling. Slower downloading during Live is the intended cost;
actual throughput and FPS improvement await JP's measurements.

Ten guard simulations and fourteen browser checks pass; code paths and SDK
tick rate inspected. No firmware build/flash or commit. JP builds the 3.3.11
profile; src-only changes do not require removing companion.ino.cpp.
First take one full Live cycle without downloading, followed by status.
Then, on separate instruction, repeat with archive 18 starting about 10 seconds
into Live in the same sitting. Keep fault switches off and MQTT connected.

## Next SDK regression case: interrupted rotation recovery (2026-09-19)

First use Download all to preserve a PC copy. JP then builds and flashes
amoled-1-8-core-3-3-11 with fixture=0, hooks=1, PSRAM writer=1 and logging=1.
Keep the hotspot on, Live stopped and all browser test switches off.

1. Connect and send status; require logger ready and hooks=1.
2. Send log test rename once. Wait for writer paused at requested boundary.
3. Turn the board fully off with its normal power button, then restart.
   Unplugging USB alone does not power off this battery board. The hook has
   closed and renamed current but has not created its replacement.
4. Reconnect, refresh files and send status. Expect ready, a fresh current
   file and an additional archive preserving the old current file.
5. Download that new archive and current.log. Wait about 70 seconds and send
   status again. Expect CRC OK, current growth, zero drops and no logger error.
   Send the console; inspect the saved files for preserved content and headers.

Stop after this one case. Do not enable small retention limits. This tests a
completed filesystem boundary, not arbitrary power loss during an SD write.
Queue/pruning tests are already passed; no repeat is needed.
