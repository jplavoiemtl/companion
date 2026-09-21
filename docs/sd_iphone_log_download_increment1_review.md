# Increment 1 - Claude review

Reviewer: Claude. Date: September 20, 2026. Subject: `7976818` against base `8038b9c` and
[spec](sd_iphone_log_download_spec.md) revision 3, section 3.
Handoff: [increment 1](sd_iphone_log_download_increment1.md).

**Verdict: no blocking defects. Proceed to build and flash, then the first bench case.**
Four non-blocking findings below, none of which changes behaviour on hardware.

## What I verified independently

Not accepted from the handoff - checked against the source and by running the suite:

- **92 host checks pass**, counts confirmed per suite: 21 reader/session, 16
  connection/pacing, 12 logger gate, 19 browser protocol, 16 network, 8 operation.
- **Ownership is enforced by test, not just by review.** `reader_session.test.cjs:131`
  asserts `diagnostics_usb.cpp` contains no `opendir/readdir/closedir/stat/fstat/open/read/
  close` call, and `:132` asserts the reader contains no `USBSerial`, `HWCDC`, `encode64`,
  `@@` framing or `send(`. That is the spec's ownership split made falsifiable.
- **Buffer layout is byte-identical.** `SCRATCH = 241`, `CHUNK = 144` reproduce the original
  `wire[WIRE + 1]` and `raw[CHUNK]`, one combined PSRAM allocation.
- **Every preserved-behaviour item in spec section 3 checks out**: acceptance-time deadline
  origin (`startedAt = lastProgress = accepted.at`), all-line progress clock with body-only
  byte and CRC credit, `stopReason` precedence unchanged (closing, ready, transport,
  abort, queue, timeout, stalled), 1 s USB-loss grace retained in the adapter, pause before
  open, resume before `@@END` via `closeReaderAndResume(false,"ok",true)`, 50 % queue abort,
  prune ordering, shutdown with no transfer output, and offline abort consumption that
  leaves a queued request intact.
- **Lock ordering is sound.** `sessionMux` is never held across a hook call or a
  `heap_caps_free`, and never nested with `sd_diagnostics`'s `mux`. `release()` deliberately
  drops the lock before freeing and retakes it to clear the reservation.
- **Static initialisation is safe.** `diagnostics_usb.cpp` binds namespace-scope references
  through `diagreader::view()` at dynamic init, but `ReaderState` has only constant default
  member initialisers, so it is constant-initialised and its address is valid before any
  dynamic initialisation runs. No initialisation-order hazard.
- **`sd_diagnostics.cpp` compiles unchanged for a real reason.** `diagreader::Status` and
  `Hooks` reproduce the old `DiagnosticsUsbStatus`/`DiagnosticsUsbHooks` member order and
  types exactly, so `usbStatus()`'s 13-element brace initialiser and the
  `{usbStatus, usbBegin, closeFile, usbResume, usbEnd}` hook list still bind.
- **The invariant JP approved is implemented and tested.** An invalidated generation rejects
  `progress()` and `readChunk()` while retaining the buffer, and the matching `release()`
  still succeeds after SD cleanup (`reader_session.test.cjs:68`, `:84`). `release()` also
  correctly refuses while a reader is open or appends are paused, which `:84` exercises.
- **Release is generation-bound.** `:69` shows a stale generation cannot free, invalidate or
  credit progress against a newer reservation.

## Findings

### 1. Two return values are ignored at the call site - low, latent

`diagnosticsUsbTick()` discards both:

- `release()` calls `diagreader::release(sessionGeneration)` without checking. If it ever
  refused, `reserved` would stay set while `phase` is forced to `Idle`, leaving `busy()`
  permanently true and every later retrieval refused with `busy` until reboot, with nothing
  retrying.
- `diagreader::progress(sessionGeneration, ...)` is discarded. A `false` return means the
  wire line was already sent but its bytes and CRC were not credited, so `@@END` would
  under-report.

**Neither is reachable today.** Every `release()` call site runs `closeReaderAndResume()`
first, which clears `reader` and `paused` unconditionally, and `invalidate()` only runs
inside `finishError()`, which breaks the work loop. But this is exactly the silent-missing-
release shape spec section 5 requires a visible error state for in HTTP. Recommend recording
a diagnostic on a `false` return rather than discarding it - a few lines now, and the HTTP
adapter inherits the habit.

### 2. The "const view" is cosmetic for buffer contents - low

`view()` returns `const ReaderState&`, but `const auto& buffers = view().buffers;` is a
`Buffers* const&`, so `buffers->wire` and `buffers->raw` are writable. The adapter formats
into shared storage through a const-looking handle. That matches the original code and is
correct for USB, but the const-ness protects the pointer rather than the bytes. Before a
second adapter exists, consider an explicit accessor for the wire scratch so the writable
surface is stated rather than implied.

### 3. `usb_connection_guard.test.cjs` matches signatures across a concatenation - low

It now builds `source = usb + reader + header` and extracts bodies with `indexOf`, so a
signature appearing in both files silently resolves to the USB one. `usb_logger_gate`
takes the better approach, passing the file explicitly (`body(usb, sig)`,
`body(reader, sig)`). Worth aligning before more shared functions appear. No current
assertion is wrong.

### 4. Build note, not a defect

A new translation unit is added under `src/`. The handoff's instruction to delete the
generated `build/build_amoled-1-8-core-3-3-11/sketch/companion.ino.cpp` is harmless but
unnecessary here - CLAUDE.md's stale-build trap applies to edits of `companion.ino`, which
this increment does not touch. Files under `src/` update normally. If the first build
behaves oddly with a newly added source file, do a clean build rather than chasing it.

## What this review does not establish

The host checks are JavaScript source simulations with mocked platform calls. **Nothing
here has been compiled.** JP's build is the first C++ compilation of the extraction, and a
new translation unit plus type aliases is precisely where a compile error would surface. No
SD, USB or timing behaviour is evidenced; append continuation, resource margins and CRC on
real hardware all remain for the bench gate.

## Recommendation

Build and flash, then run the first gate: **one normal `current.log` USB transfer**, status
before and after, CRC success, and confirmed later append growth, with DTR=true and
RTS=false. Queue-pressure, prune, cancel and close gates follow one at a time before the
extraction is accepted.
