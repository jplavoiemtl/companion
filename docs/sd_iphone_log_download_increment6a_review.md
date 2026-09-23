# Increment 6A - Claude code review

Reviewer: Claude. Date: September 23, 2026. Subject: `bef58ea` against `229a344`, the
[6A handoff](sd_iphone_log_download_increment6a.md), the
[time design revision 2](sd_iphone_log_download_time_design.md) and my
[design review](sd_iphone_log_download_time_design_review.md). No build or flash.

**No blockers. Cleared for JP's build.** All three design-review blockers are fixed in the
code as agreed. I independently confirmed 241 host checks (43+35+20+15+16+8+28+29+19+16+12).
The notes below are non-blocking.

## Design blockers, as implemented

**B1 - current timestamps come from writer memory only.** No listing path opens current.log.
`direct()` and `writeRecord()` call `diagtime::written()` only after `rawWrite` succeeds, and
they parse the exact formatted `line`, so the timezone is never re-derived later. A partial
write goes through `rawWrite`'s `disable()` and returns false, so it never advances metadata.
All lifecycles are covered:

| Path | How the endpoint is set |
|------|-------------------------|
| New / salvage / rotation | `createCurrent` -> `direct(FILE_OPEN)` -> `written(first=true)` resets both ends |
| Empty recovery | same `direct(FILE_OPEN)`, `generation` assigned before the call |
| Append boot | `restoreCurrent` from the header already in `line`, inside `if (valid)` only |
| Append + incomplete tail near the cap | `rotate("incomplete_tail")` resets via FILE_OPEN |
| Recovery newline | raw `rawWrite("\n")` does not call `written`; the TAIL_RECOVERY record after it does |

The inventory copies `diagtime::current()` on the writer task, the same task that calls
`written()`, so there is no cross-task access. The current row's size now comes from the
writer's `sizeBytes`. That matches the `fstat` a later snapshot freezes after pause flushes.

**B2 - no archive descriptor crosses a writer turn.** `diagtime::archive()` holds the
descriptor for a single call: each `open` that succeeds reaches exactly one `close` through
the `do { } while(false)`, and a close failure turns both ends into `Io`. The metadata
phase returns after each read, so prune, rotation and transfer dispatch cannot run while a
descriptor is open. No dependency on prune calling preempt was added.

**B3 - recovery validation unchanged.** I diffed `headerGeneration`, `headerToken` and
`decimal` against `229a344` (CRLF-normalized): all identical. The test asserts the same body
from `git show 229a344`. `restoreCurrent` is a side call and does not affect `valid`,
`saved` or the salvage decision. `diagnosticsHeaderValid` is a pass-through defined at global
scope after the anonymous namespace closes (line 1295), so it can reach `headerGeneration`.

## Scratch lifetime and reader positioning

**The scratch buffer cannot be freed under `readStart`.** The lifecycle stop sequence
(`diagnostics_http.cpp:475-490`) waits until `handlers == 0 && !diagtransfer::busy()`, then
calls `httpd_stop`, and only then `diaginventory::dispose()`, which frees the scratch.
`box.reserved` stays set until the writer has closed the reader and released it. The scratch
is allocated in `diaginventory::start()` before the server starts listening, so it is never
null while a download can be requested. The pointer is written on the lifecycle task and read
on the writer, and both hand-offs pass through `mux` critical sections that act as barriers.
Inventory and `readStart` share the scratch only serially on the writer task.
`allocate()` refuses when a scratch already exists, and `start()`'s failure path frees only
the scratch it just allocated.

**Positioning is correct.** `readStart` reads immediately after `open`/`fstat`, at position 0,
before any `readChunk`. It then requires `lseek(reader,0,SEEK_SET)==0`; `-1` also fails that
comparison. A failed restore returns `metadata_seek` before `box.metadata` is published,
through the existing post-open error path (close, then resume). A read error returns
`read_failed` after the restore attempt. The head bytes never touch `sentBytes`, `crc` or
`lastProgress`. A second `stopReason` check follows. USB start never calls it.

## Timestamp parsing and filename safety

The parser matches the revision 2 grammar: exact separators, digit-only fields, year
2024-2099, leap years (`%4` is exact in this range), `|offset| <= 840` with minutes 0/30/45,
local/quality consistency, uint64 overflow guards, whitelisted level and event, and records
of at most 1023 bytes ending in a newline. `format()` and `prefix()` re-check `validDate`
before printing, so a bad `Time` cannot reach output. Only `Synced` produces a dated filename;
approx, test, unknown and malformed all give `start-unknown`. The `Content-Disposition` value
contains only digits, `-`, `+`, `T`, `_` and the managed stem. The page contains only
literals and numbers, so nothing needs escaping.

The tail logic handles every case: offset 0 counts as a boundary, a window that begins
mid-line (`base && !begin`) is rejected, a trailing fragment is flagged, and a header-only
file is recognized by byte position (`base+begin==0 && end-begin==firstLength`), not by
comparing values. From the window arithmetic, `begin >= 3` for any valid record, so
`base && !begin` fires only on invalid content.

## Cache, retries and ordering

- **Carry-forward** reads the published slot, which is immutable while pinned and never the
  staging slot. The key is (number, size), and zeroed `calloc` memory equals `Pending`
  because `Pending` is enumerator 0. A revision change, busy reader, stop or preempt
  discards staging through `endScan()`, which now also resets the metadata phase. Published
  progress survives.
- **Retries are bounded.** An `Io` result sets a 5 s `retryAt` and ends reads for the cycle.
  Within that window the failed entry carries forward and is not reread. Remaining pending
  rows still trigger an immediate cycle, so one bad file cannot starve the others.
- **Sort** is correct, and I traced mixed cases: current stays last whatever its readdir
  position.

## Memory and C++ layout

`Time` is 12 bytes (2+2+2+5+1, 2-byte alignment). `Ends` is 26, and `Entry` is 56 (16 +
26, padded to 8, + 8), within the 64-byte assertion. Two slots take 28,672 bytes. The
transfer `View` and `Result` each grow by 16 bytes after padding. `state.opened = {}` and
`Entry entry{}` are valid aggregate initialization with default member initializers.
`diag::Quality` and `diagtime::Quality` are in separate namespaces, as are the two anonymous
`decimal` helpers.

Row budgets hold for the actual strings. Opened line ≤ 43 (budget 44). Archive last line
≤ 57, current last line ≤ 49 (budget 60; current never has a fragment). The current name
line is 96 (budget 112). The host renders 256 worst-case rows.

## Non-blocking notes

1. **`token()` accepts NUL and control bytes.** `decimal`, `strcmp` and `strlen` stop at an
   embedded NUL, so `seq=12\0\0` parses as 12, and `level=INFO\0x` passes. A timestamp cannot
   be corrupted this way, because the exact 29-character `strlen` check rejects it, but a
   stale tail containing NULs could pass the plausibility check. Rejecting bytes below 0x20
   inside `token()` closes this. The JS harness uses string semantics and cannot show it.
   Fix it in a later change; it is not needed before this build.
2. **Salvaged archives always show `last unavailable (inconsistent)`.** An invalid header
   makes the check impossible, not failed. `(unverified)` would be the accurate label.
   Cosmetic only.
3. **The insertion sort is O(n²) on 56-byte PSRAM entries.** 256 entries in reverse order
   would copy about 1.8 MB in one writer tick. The realistic case is ≤ 31 mostly ascending
   entries, which costs nothing. Note only.
4. **Many distinct persistently failing files run immediate cycles back to back** until each
   has a retry deadline. Each file is still retried at most once per 5 s, and a truly failing
   card fails the logger first. Acceptable.
5. **`local=unknown time=approx` parses as malformed** rather than unknown. The firmware
   emits this only if the epoch leaves 2024-2099 while quality is approx. Negligible.
6. **`log_time.test.cjs` needs the `229a344` object** (`git show`), so a shallow clone would
   fail that assertion. That is acceptable for this repo.

## First bench case - additions

As proposed in the handoff: one immutable archive with a synced valid header, Safari byte
equality/CRC, and the listing endpoints checked against the file's actual first and last
lines. Please also:

- Capture **both** `http_margin` and the writer stack minimum after the listing has rendered
  and after the download. Parsing now runs on every write, and the listing path has about
  96 more bytes of locals.
- Confirm the exported name has the `YYYY-MM-DDTHHMMSS-0400_` prefix, and that its seconds
  are the header's seconds truncated, not rounded.
- Load the listing at least once from cold (fresh mode entry) and note how long until no
  rows show `pending`.

Rebuild the same 3.3.11 profile. companion.ino is unchanged, so no generated-sketch deletion
is needed. Increment 7 remains unapproved.
