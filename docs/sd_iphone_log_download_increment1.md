# iPhone log retrieval - increment 1 review handoff

Status: implemented for Claude review; **not compiled, flashed or bench accepted**.
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

After Claude review and any corrections, JP performs the build/flash. Before that rebuild,
remove the selected profile's generated `build/build_amoled-1-8-core-3-3-11/sketch/companion.ino.cpp`.
The first bench gate remains **one normal current.log USB transfer**, status before/after,
CRC success and confirmed later append growth, using DTR=true and RTS=false. Give the
precise case only at that handoff. Queue-pressure, prune, cancel and close gates remain
required before accepting the extraction, issued one case at a time. No increment 2 work
without JP's explicit approval.
