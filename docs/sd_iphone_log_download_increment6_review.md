# Increment 6 - Claude code review

Reviewer: Claude. Date: September 22, 2026. Subject: `719a862` against `af9523f`,
[increment 6 handoff](sd_iphone_log_download_increment6.md) and
[spec](sd_iphone_log_download_spec.md) revision 7.

**No blockers. Cleared for JP's build.** The pause ordering is correct, and two notes I
raised in earlier increments are now implemented as host checks.

## Pause, freeze and resume ordering

The sequence in `start()` is right, and the order is what makes `Content-Length` honest:

```cpp
if (isCurrent) { pausedAt = milliseconds(); paused = true; publish();
                 if (!hooks.pause()) return "logger_failed"; }
reader = open(path, O_RDONLY);
if (fstat(reader, &st) ...) ...
fileSize = st.st_size;
```

- `paused = true` is marked **before** the hook runs, so a pause failure still takes the
  cleanup path - the invariant from the original USB design, preserved.
- The size is frozen by `fstat` **after** `hooks.pause()` has flushed and closed the append
  descriptor, so the frozen length cannot grow underneath the transfer. That is what makes a
  fixed `Content-Length` correct rather than optimistic.
- `isCurrent` now covers `HttpCurrent`, so `CURRENT_MS` 120000 and the pause path both
  apply, while USB begin/end records are skipped exactly as for HTTP archives.

**Rotation cannot interleave with a snapshot**, and this falls out of the existing design
rather than new code: while `diagnosticsUsbPaused()` is true the writer loop takes its paused
branch, which pumps `diagnosticsUsbTick()` - and therefore `diagtransfer::tick()` - without
running a logging batch. No `writeRecord`, so no `rotate()`. The transfer progresses while
appends are closed, and the file it is reading cannot be renamed under it.

**Appends resume before the reservation is released.** `close()` runs `closeReaderAndResume()`
on the writer, which reopens the append descriptor; `finishRelease()` - which frees storage -
only runs once HTTP has posted its release. So logging restarts even if the HTTP task is
stuck, which is the property that matters most for a paused-append transfer.

`beforePrune()` returns false while `isCurrent`, so a snapshot can never be targeted by
pruning - correct, since pruning only removes archives.

## Cleanup telemetry

`pausedAt` is captured **before** the pause hook, so the reported duration conservatively
includes the flush and close work rather than excluding it. `readerClosedAt` is set after the
close attempt regardless of outcome, and `resumedAt` only on a successful reopen. The outcome
string derives from those:

```cpp
result.appends = closed.pausedAt ? (closed.resumedAt ? "resumed" : "resume_failed") : "unpaused";
```

and `paused_ms` is guarded against the underflow that would otherwise occur when resume
failed:

```cpp
(result.resumedAt ? result.resumedAt-result.pausedAt : 0)
```

I checked one subtlety: the three timestamps are reset inside `start()` **after** its validity
check, so a start that returns `aborted` would report the previous transfer's values. That
path is unreachable - after `request()` succeeds the reservation cannot be cleared before
`accept()` consumes it, and the shutdown path in `stop()` still reaches the reset before
`stopReason()` returns. Worth knowing the reset placement depends on that invariant.

`appends` stays `unknown` until close, since it is populated only from the identity-matched
closed snapshot. That correctly distinguishes "not yet known" from "never paused".

## Record capacity - my earlier note is now implemented

`END` drops `resume_ms=0 appends=unpaused` and the measured cleanup moves to a separate
`HTTP_GET_CLOSE`. That keeps the CRC-bearing record well inside the 456-byte field capacity,
where an oversized record would be **silently dropped** - about 287 bytes worst case now
against roughly 313 before, with `CLOSE` around 145. Both records are covered by explicit
maximum-width host checks, which is the guard I suggested rather than a recomputation every
time a field is added.

## Configuration pinning - also implemented

`stack_size`, `task_caps`, `max_open_sockets` and `lru_purge_enable` are now asserted by host
checks. That matters because 6144 exists only because a measurement demanded it; the number
now carries its reason.

## Filenames and routes

`nameFor()` produces the managed name for BEGIN, then `name[strlen(name)-4]=0` strips `.log`
to form the attachment stem, giving `110-1-current-964476.log` and leaving the archive form
`110-1-archive-00000017-308745.log` byte-identical to gate A. The subtraction is safe because
every managed name is at least `current.log`, and the invariant is commented. `/f/current` is
an exact match placed before the numeric parse, so no archive path can be reached through it.

## USB unchanged

The only USB-facing edits add `HttpCurrent` alongside `HttpArchive` in the two dispatch
conditions. Wire framing, 144-byte chunks, line progress, CRC, `STALL_MS`, queue-pressure
abort and USB event records are untouched, and the 16 connection/pacing and 12 logger-gate
checks pass unmodified.

**219 host checks pass** across ten suites, counts confirmed by re-running them.

## Two things to watch in gate C, neither a blocker

**The metadata wait now spans SD work.** For `current.log` the HTTP task's 5 s metadata wait
covers the pause hook's flush and close plus the open and fstat. Accepted evidence puts flush
max around 113 ms and `sd_max_us` around 68 ms, so the margin is roughly fiftyfold. If a slow
card ever exceeded it the result is `metadata_timeout`, and the writer still resumes appends
on its next tick - so the failure is bounded, not stuck. Worth reading `pause_ms` on the first
run to see where it actually lands.

**The queue-pressure window is the real subject of gate C.** At the 53 KB/s measured in gate
A, a roughly 964 KB snapshot is about 18 seconds of paused appends. Ordinary traffic in that
window is about one HEALTH/NET_HEALTH pair, far below the 8-event guard - so a *normal* case
should pass comfortably, and that is precisely why the controlled queue-pressure case is
needed separately to exercise the abort. The handoff already requires it, and is right that an
abort is evidence to analyse rather than grounds to enlarge the queue.

## What this review does not establish

Source simulations with mocked platform calls; **nothing has been compiled**. No real SD pause
timing, no hardware stack margin at 6144 under Safari headers, and no queue behaviour under a
real paused window. `companion.ino` is unchanged, so no generated-sketch deletion is required.

## Clearance

Cleared. Build and flash, then run the single normal `current.log` Safari snapshot. Read in
this order: `appends=resumed` with a plausible `pause_ms`, then `crc_check=match`, then
`http_margin`, then the exported phone bytes against the equal-length **prefix** of the later
USB file - the prefix comparison, not whole files, since appends resume the moment the freeze
ends.
