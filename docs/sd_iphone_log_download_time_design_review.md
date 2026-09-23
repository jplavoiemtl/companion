# Event-time discovery - Claude design review

Reviewer: Claude. Date: September 23, 2026. Subject: `a9e25c1`,
[event-time discovery design](sd_iphone_log_download_time_design.md), checked against
current source and the [spec](sd_iphone_log_download_spec.md). Increment 6 accepted.
Design review only: no firmware changes, builds or flashes.

**Three blockers, all fixed by amending the design rather than adding scope.** Two of
them *remove* machinery: one writer-side change eliminates every current.log SD read, and
one descriptor rule makes the resumable read state machine unnecessary. The listing-layout
question is resolved below as one page with a larger PSRAM buffer, not pagination. With
these amendments the design is ready for JP's implementation decision.

## Blockers

### B1. Reading current.log while the writer holds it open is prohibited by FatFs

The design keeps current.log metadata fresh with "bounded tail refresh, never a forced
flush or pause", which means opening `current.log` read-only while the writer's
`O_APPEND` descriptor is open. FatFs forbids this: a file may be opened more than once
only if every open is read-only, and an open file must not be renamed or deleted. It
warns that breaking these rules can corrupt data. The toolchain will not catch it:
core 3.3.11 builds with `CONFIG_FATFS_FS_LOCK=0`, so the duplicate `f_open` succeeds
silently. No existing path does this. USB and HTTP current transfers both pause (flush
and close) first, and the inventory only calls `stat()`, which does not open the file.

**Resolution - the writer already knows both endpoints; read nothing.**

- **Start:** `createCurrent()` builds the `FILE_OPEN` stamp itself. `openStorage()`'s append
  path already reads and validates the header into `line` (`sd_diagnostics.cpp:765-771`).
  Parse the start from that same buffer before it is reused. The `empty_recovery` and
  salvage paths go through `direct()`/`createCurrent()` and are covered the same way.
- **Last record:** store the `when` of the last record whose `rawWrite` succeeded.
  `writeRecord` and `direct` are the only producers.

This value is more accurate than a tail read. It covers records written but not yet
flushed, and those are exactly the records a later frozen snapshot will contain, because
pause flushes. It also deletes several parts of the design: generation+size rechecks for
current, stale/advisory relabelling, "first header reused for same generation", and "never
scan while paused". The inventory reads writer memory on the same task. HTTP still sees
only the published copy. The row label becomes "last record written"; the file is still
marked growing.

### B2. A metadata descriptor must not outlive a writer tick; prune does not preempt

The design says "Prune, rotation and shutdown call existing preemption". Only rotation
does (`rotate()` calls `diaginventory::writerPreempt()` at `sd_diagnostics.cpp:657`).
`pruneArchive()` calls `diagnosticsUsbBeforePrune()` and `diaginventory::writerChanged()`,
and `writerChanged` only bumps the revision. Today that is harmless because the inventory
holds a `DIR*`, not a file. But a resumable machine that keeps an archive descriptor open
across ticks could have that archive unlinked under it by any of the writer's `prune()`
calls (batch loop, rotation, closing drain). That is the "open file must not be deleted"
violation.

**Resolution:** each candidate file is handled whole within one `writerTick`: open,
`fstat`, head read, tail seek+read, close. No descriptor crosses a tick. The writer is one
task, so prune, rotation, a transfer start and shutdown all happen between ticks. A
reservation that arrives mid-tick is taken by this same task only after the tick returns,
so the existing `diagreader::busy()` check at the top of the tick still makes the shared
reader win. The worst delay to a transfer start is one file's metadata read. The resumable
state machine shrinks to a cursor ("next entry index") and needs no per-phase
descriptor/revision rechecks. Pinning this with a host assertion is cheaper than adding a
`writerPreempt` call to `pruneArchive` and relying on it forever.

### B3. "No weakening" of header validation must be "no change to recovery acceptance"

`headerGeneration()` decides at boot whether current.log is appended to or salvaged into
an archive. Its `local=` check is shape-only: length 29, `T`, `.`, sign
(`sd_diagnostics.cpp:721-722`). No calendar or digit validation. If the strict calendar
parser the design asks for is merged into it, a header that is valid today could be
salvaged at the next boot. That would be a silent recovery behavior change that needs its
own bench gate. **Keep `headerGeneration()` byte-identical.** Put the strict timestamp
parser in a separate layer that accepts a start only if `headerGeneration()` accepts the
line **and** the strict parse passes. Add a host matrix that pins `headerGeneration()`'s
current acceptance, including the shape-valid, calendar-invalid cases it accepts today.

## Timestamp accuracy

Agreed as written: endpoints are navigation hints, per-record offsets, unknown means
unavailable, approx keeps start-unknown in the filename, no swap when end precedes start.
Source facts that sharpen the semantics:

- **`synced` means "anchored by SNTP this boot, with no unannounced jump over 2 s since"**
  (`diagnostics_clock.cpp:58-78, 208-246`), not "recently verified". It is sticky for the
  whole boot, and `clockPoll` downgrades to approx only on a jump over 2 s. That is fine for
  navigation. Do not describe it as more than that.
- **In the car, start will often be unavailable for the file current at boot, and often
  available for files created by rotation.** An ignition start is `ESP_RST_POWERON`, so the
  clock is Unknown until SNTP syncs. current.log appends across boots, so its `FILE_OPEN`
  may be days old: correct "File opened" semantics, but worth a line in the listing legend.
  A file created by rotation usually opens after the sync. Because archives are contiguous
  in number, a neighbour's last record brackets a file whose own start is unavailable. That
  is one more reason for sorting by number (layout below).
- **Test-clock records must not show as approx wall time.** `formatLine` appends
  ` clock_source=test` as the final token with quality approx. Test builds have already
  written such records to real card logs (boot 30, `bench_data/sd_logs_2026-09-16_1802`),
  and a test-clock record can be a file's last record. Treat that exact trailing
  token as "test clock": unavailable in the filename, labelled test in the listing. Matching
  only the final token before the newline is consistent with "do not trust embedded text",
  because `formatLine` controls that position.
- **`empty_recovery` headers stamp the recovery time**, not the original creation time. It
  is still the time the first byte was written, which is acceptable. Record it as known
  behavior.
- **Strict parse bounds:** year 2024-2099, matching `plausible()`; month 01-12; day valid
  for the month, including leap years; hour 00-23; minute and second 00-59 (`localtime_r`
  never produces 60); ms 000-999; offset sign, hours 00-14, minutes 00/30/45; exact
  separators; nothing after the offset except the single space.

**Add a tail plausibility check (not a blocker, but it cheaply closes a real hole).** After
power loss, an SD card's own write ordering can leave a size that covers stale bytes from
a reused cluster, such as the pruned archive from 30 files ago. Stale bytes that end in
`\n` look like a valid complete record with a wrong, older timestamp, and the existing
tail recovery (last byte only) would not notice. The source gives a free invariant.
`sequence` restarts at 0 each boot (`sd_diagnostics.cpp:162`) and the boot counter is
monotonic NVS, so **(boot, seq) strictly increases through every record of a file**.
Require the last record's (boot, seq) to exceed the header's, or show "inconsistent" as
unavailable. A volatile session (`persistent=false`, boot 0) fails the check, which is the
right conservative outcome.

## Bounded reads and window arithmetic

The 1024 head / 2048 tail / 2049 scratch sizing is correct against `formatLine`'s
`< LINE_CAPACITY` bound. A trailing fragment is at most 1022 bytes (a record without its
newline), the preceding record at most 1023, and its preceding newline 1: 2046 ≤ 2048.
Two precision points:

- **Offset 0 is a record boundary.** When the file is shorter than the window, or the
  window starts at 0, do not discard the first line as "leading partial".
- **Bound reads by the descriptor's `fstat`, not the directory-pass size.** If they differ,
  mark the entry changed/unavailable for this cycle.

Fixture archives with 1024-byte lines (test builds) exceed the bound and fall back to
unavailable, as designed.

## Cache and invalidation

Keying by (number, size) and scoping to the mode lifetime is right. The design leaves open
where completed reads live, and that decides whether progress is monotonic:

- **Carry forward from the published slot.** The published slot is immutable, and the
  writer may read it while HTTP holds pins. The staging slot is always the unpinned other
  one (`diagnostics_inventory.cpp:102-104`). During the directory pass, copy completed
  metadata for matching (number, size); leave the rest pending. No third array is needed.
- **Cap reads per cycle, then publish** (for example 8 files). Pending rows display
  "pending". If pending entries remain, schedule the next cycle immediately instead of
  after 5 s. A cold listing of ~31 files then fills in about a second of 20 ms writer
  turns. Preemption loses at most one cycle's reads, and progress survives through the
  published slot.
- **Optional:** at rotation, the writer holds the outgoing current's exact start and last
  (from B1) and could seed the new archive's entry, but only while the mode is active.
  Leave this out of version 1 unless it proves necessary.

## Memory and page capacity - listing layout resolved

**The current page has no room.** `static_assert(256*112+2048+1 <= PAGE_CAP)` evaluates
to 30721 of 32768, leaving 2047 bytes, about 8 per row. Real row maxima are 74 bytes
(archive row, 20-digit size) and 87 (current row). Any readable endpoint pair costs 60+
bytes per row.

**Decision: one page, one entry per managed file, a larger PSRAM page. No pagination.**

- **Pagination rejected.** `ARCHIVE_LIMIT = 30` (`sd_diagnostics.cpp:39`), so steady state
  is at most 31 managed rows, and the 256-entry case exists only as the directory-limit
  bound. A pagination path would almost never run in the field, so it would be unexercised
  code. It would also add query parsing (new request input) and page boundaries over an
  unsorted snapshot that refreshes every 5 s, which skips or duplicates rows.
- **Layout: three lines per file** inside the existing `<pre>`, readable at iPhone width
  without horizontal scroll:

  ```text
  archive-00000021.log  2097152 bytes
    opened 2026-09-22 14:10:03 -04:00
    last   2026-09-22 15:32:10 -04:00 approx
  current.log  1477034 bytes (growing; snapshot on download)
    opened 2026-09-12 08:02:11 -04:00
    last   written 2026-09-23 07:58:40 -04:00
  ```

  Unavailable endpoints print a short fixed reason (`unavailable (clock unknown)`,
  `(malformed)`, `(pending)`, `(test clock)`, `(inconsistent)`). A trailing fragment adds
  `(tail fragment)`. Every string is a compile-time literal.
- **Sort by archive number ascending, current last.** Existing order is `readdir` order,
  which is not number order after pruning reuses directory slots. Number is the trusted
  identity, so this is "identity ordering", and it puts neighbours side by side for
  bracketing. Insertion sort of ≤256 entries at publication, on the writer.
- **Budget:** line 1 keeps the existing 112 bound. Line 2 ≤ 44, line 3 ≤ 60, so 216 per
  entry; use 224. The fixed part grows by a one-line legend, which must fit inside the 2048
  fixed budget (verify with a host render at maximum field values).
  256 × 224 + 2048 + 1 = 59393, so **`PAGE_CAP` = 65536**. Express the `static_assert`
  from named per-line maxima rather than the bare 112, and add a host test that renders 256
  worst-case entries and checks it against `PAGE_CAP`.
- **Cost:** +32 KiB PSRAM only while the server runs, allocated where the page is today
  (`diagnostics_http.cpp:439`), with the same refusal on allocation failure. Internal heap
  is unchanged. Worst-case send is about 59 KB, roughly 1.2 s at the measured 51 kB/s, under
  the existing 5 s I/O bound. A realistic 31-row page is about 7 KB.

Entry storage: a 64-byte cap per inventory entry is ample. Store per endpoint the numeric
local components, ms, signed offset minutes, quality, boot and seq (for the check above),
plus status flags. Enforce `sizeof` with `static_assert`. Two slots of 256 × 64 = 32 KiB
PSRAM plus 2049 scratch, as proposed.

## Download-reader positioning

The design is sound: head read on the actual reader, after pause for current (the append
descriptor is closed by then, so B1 does not apply), and published atomically with the
frozen size. That fits the existing single critical section at
`diagnostics_http_transfer.cpp:120`. Tighten three points:

- **Read the head immediately after `open`+`fstat`,** while the position is known to be 0.
  Then require `lseek(fd, 0, SEEK_SET) == 0` explicitly. Do not use `pread`: its position
  restore is hidden in the VFS layer.
- **Why the explicit check matters:** `readChunk()` reads sequentially with no offset. A
  position left at *h* would stream bytes *h..*, and both CRCs would be computed over the
  same wrong bytes, so `crc_check=match`. The only symptom would be `read_failed` at EOF,
  after a full-length transfer. A host test must cover a failed or incorrect restore aborting
  before headers.
- **`View` grows:** it is copied by value on the HTTP stack in the body loop. Keep the
  added fields compact (≤ 16 bytes) and require `http_margin` in the first bench case
  (2420 measured at increment 6).

Filename agreed: `2026-09-22T141003-0400_116-2-archive-00000021-2097152.log`, and
`start-unknown_…` otherwise. Truncate to seconds, don't round. One semantic note for the
listing legend: the `116` is the **download** boot, while the date is the file's open time.
For current.log the two can be days apart.

## Confirmed as written

Design source facts: `FILE_OPEN` fields and position, `LINE_CAPACITY`, recovery behaviors,
inventory double-buffer/yield/preempt behavior, and HTTP never opening SD files. Also
agreed: no SD format, NVS, sidecar or USB wire changes; archive suffix not required to
equal header generation; endpoint offsets from records; metadata failures never affecting
file bytes; no tail reads on the transfer path; 120000/5000 limits unchanged; follow-up
6A accepted separately from increment 6.

## Next step

Amend the design with B1-B3, the plausibility check, the test-clock rule and the layout
above. Then JP can approve implementation. Add to the future host checks: a writer-memory
endpoint tracking matrix (create, append-boot, empty recovery, salvage, rotation); no
descriptor crossing a tick; pinned `headerGeneration()` acceptance; worst-case page render;
position-restore failure; sort order. Add to the first bench case: `http_margin` and a
listing whose rows are checked against the actual first and last lines of each file.
Increment 7 remains unapproved.
