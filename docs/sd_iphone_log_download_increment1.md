# iPhone log retrieval - increment 1 review handoff

Status: Claude review passed at `0ba72e5`; JP returned a passing first USB bench case
on September 21. **Full extraction acceptance remains pending.** No assistant build/flash.
Base: `8038b9c`, branch `iphone-log-retrieval`. Implements only the approved USB-only
extraction in revision 3 of [the spec](sd_iphone_log_download_spec.md), section 3.
Increment 2 is not approved.

## Changes

- `src/diagnostics/diagnostics_reader.h/.cpp` owns inventory, the single reservation,
  monotonically increasing session generation, reader and PSRAM storage, current-file
  pause/snapshot/resume, bounds, CRC and progress accounting.
- `diagnostics_usb.cpp` retains parsing, connection observations and grace, wire/base64,
  pacing fixture, control/status/error output and the USB phase machine. Public hook
  types are aliases to the extracted types; existing logger integration is unchanged.
- Main reserves or posts abort under the service lock. Start/read/progress/invalidate/
  cleanup/release execute on the writer. These calls add no consumer/network waits;
  existing synchronous SD calls retain their existing behavior. No new task is created.
- Invalidation retains the buffer and reservation while rejecting further progress.
  A matching release still succeeds after invalidation and SD cleanup; a stale or
  duplicate acknowledgement cannot free a newer reservation. USB releases synchronously
  on the writer after it stops using the buffer. Future asynchronous consumers need an
  explicit handoff; the writer-only view is not a cross-task publication interface.
- Original 144-byte chunks, 240-byte wire limit, combined PSRAM allocation, all-line
  USB progress clock, body-only byte/CRC credit, acceptance-time deadline, current
  snapshot ordering and resume-before-END behavior are retained. Offline abort handling
  consumes only the abort flag, leaving any queued request intact.

No HTTP, download-mode admission, socket lifecycle, config/profile, build tag, main sketch
or historical draft changes. Section 5's provisional HTTP teardown remains unresolved
and must be settled before increment 3.

## Host validation

**92 checks pass:** the existing 71 assertions across USB connection/pacing (16), logger
protection (12), browser protocol (19), network diagnostics (16), operation diagnostics
(8), plus 21 new reader/session checks. Existing USB tests follow the extracted source
locations; their behavioral assertions remain in place.

New coverage includes queued cancellation; invalidated-buffer retention and matching
release; stale progress/cancel/release after reuse; generation exhaustion; snapshot
ordering; CRC and pending-chunk retry; cleanup failures; shutdown; prune matching;
stop precedence; allocation failure; and offline abort/request separation. A combined
adapter/service simulation checks normal BEGIN/data/END framing, reconstructed bytes,
and append readiness before END.

These are JavaScript source simulations with mocked platform operations, **not C++
compilation or physical SD/USB evidence**. They do not establish actual append continuation,
firmware linkage, resource margins or hardware timing. `git diff --check` passes.

Command:

```text
node --test tools/tests/reader_session.test.cjs tools/tests/usb_connection_guard.test.cjs tools/tests/usb_logger_gate.test.cjs tools/tests/sd_log_browser.test.cjs tools/tests/network_diagnostics.test.cjs tools/tests/operation_diagnostics.test.cjs
```

## Requested Claude review

Review the diff from `8038b9c` against spec revision 3 and the original USB implementation.
Focus on exact progress/timeout semantics; current pause/open/resume and failure ordering;
reservation/abort races, lock ordering, cancellation and generation reuse; archive prune,
shutdown and offline behavior; buffer ownership and C++ initialization/type compatibility;
and whether the adapted tests preserve their assertions and cover the new invariants.
Report concrete defects or omissions. Do not build or flash.

JP performs the build/flash. No generated-sketch deletion is required for this increment:
`companion.ino` is unchanged. If the new translation unit is not picked up correctly, use
a clean build; send compilation errors to Codex.
The first bench gate remains **one normal current.log USB transfer**, status before/after,
CRC success and confirmed later append growth, using DTR=true and RTS=false. Give the
precise case only at that handoff. Queue-pressure, prune, cancel and close gates remain
required before accepting the extraction, issued one case at a time. No increment 2 work
without JP's explicit approval.

## Claude review disposition - September 20

Review: [Claude findings](sd_iphone_log_download_increment1_review.md), `0ba72e5`.
No firmware or test changes after that reviewed implementation. The four findings are
resolved for the first bench handoff as follows; deferred work is not silently closed:

1. **Deliberately defer refused progress/release diagnostics** until before a second
   adapter is introduced (no later than increment 3). Current writer-only USB call paths
   exclude these refusals, as independently reviewed. Adding a print alone would expose
   but not resolve a retained reservation; choose the visible error, recovery and bounded
   reporting policy together, then test injected failures. In particular, a legitimate
   idle `diagnosticsUsbStop()` has no reservation to release and must not become a false
   fault. This deferral preserves the reviewed extraction for its first regression gate.
   HTTP's missing-release error/recovery requirement remains mandatory and unresolved.
2. **Defer explicit scratch/read-only payload accessors** until before a second adapter
   exists, no later than increment 3. USB intentionally writes wire scratch in the shared
   allocation on the writer task; const protects the pointer, not its pointee. The current
   interface must not be treated as permission for cross-task payload mutation.
3. **Defer explicit-file extraction in `usb_connection_guard.test.cjs`** until the next
   change to that test/shared function layout, and before adding a second adapter. All
   current extracted signatures resolve correctly and Claude reconfirmed all assertions;
   the concatenation remains a known maintenance risk, not a weakened current assertion.
4. **Accept the build-note correction now.** No generated `companion.ino.cpp` deletion
   for this source-only increment. Clean build is the fallback for a source-discovery
   problem. No build or flash performed by Codex.

## First USB extraction bench case - passed September 21

[Recorded result](sd_iphone_log_download_bench.md): boot 95, 32598 bytes, CRC OK, later
append growth, zero drops/errors, internal largest block 51188 bytes. Procedure retained
below for provenance.

1. JP builds/flashes `amoled-1-8-core-3-3-11` in VS Code. Keep logging and PSRAM writer
   enabled, both diagnostic test flags zero. Send any compile error before proceeding.
2. Close the VS Code serial monitor, then connect COM4 in Chrome using
   `tools/sd_log_browser.html`, mode **Apply explicit values after open**, DTR=true,
   RTS=false. Let startup finish. Keep normal browser reads (no slow-reader fixture).
3. Clear the console once. Send `status`, then `log status`; retain their full output.
   Avoid starting Latest/Live or another download during this case.
4. Click **Refresh files**, then **Download** on the `current.log` row only. Wait for
   `CRC OK` and the saved file. Do not use the current-plus-three bundle.
5. Send `status` and `log status` immediately after completion. Wait **70 seconds** with
   the connection open (normal health records occur every 60 seconds), then send both
   commands again. No second download is needed.
6. Click **Save console**. Send the complete console capture and downloaded current.log,
   and report any unexpected reset, stall or visible issue.

Expected: CRC OK; logger ready; active=0, paused=0, result=ok after completion; zero drops;
no new logger error, stall or reset; same boot; later current_size larger than immediately
post-transfer. Check newest archive identity too: a rotation makes a simple size comparison
inconclusive and must be interpreted from the capture. Internal largest block stays above
20480 bytes. Append growth is now confirmed by the September 21 result; remaining gates are pending.

## September 21 follow-up

Cancellation/retry gate passed on boot 95: 1296-byte aborted transfer, successful
41702-byte retry (local CRC32 E936A9DC), append growth and no drops/errors. See the
[bench record](sd_iphone_log_download_bench.md). Next is only queue pressure using the
existing fixture flag; prune and close remain pending. No increment 2 approval implied.

## September 21 queue-pressure result

Gate 3 passed on boot 97: 8/16 triggered logger_busy; all eight test records saved;
61227-byte retry CRC OK, zero drops/errors. Next is the disposable-archive prune case
on the same fixture build, then close coverage. See the [bench record](sd_iphone_log_download_bench.md).
JP's local fixture=1 edit remains uncommitted for bench use. Increment 2 is not approved.

## September 21 prune result

Gate 4 passed: synthetic archive 22 removed during retrieval, 67600-byte normal retry
CRC OK, zero drops/errors. Next single case is orderly shutdown close/restart on the
same build; see [bench record](sd_iphone_log_download_bench.md). Fixture=1 remains local;
normal fixture=0 restoration is due before final handoff. Increment 2 remains unapproved.
