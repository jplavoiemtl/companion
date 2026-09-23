# Increment 6A - event-time file discovery implementation

September 23, 2026. JP approved 6A against design revision 2 at 1517de0. Implemented for
Claude code review; no firmware compilation, build, flash or hardware measurements.
Increment 7 remains unapproved. Historical draft and companion.ino unchanged.

## Implemented behavior

Listing shows file opened and last complete archive record; current is labelled last
written and growing. Archive-number ascending order, current last. Dates retain stored
local offsets and approximate labels. Unknown, malformed, pending, test clock, inconsistent
and I/O cases have fixed unavailable labels. No raw SD strings become HTML or header text.
Current timestamps come exclusively from writer memory. direct/writeRecord parse the exact
formatted line only after successful rawWrite; timezone is not reformatted later. Append
boot restores the start from the existing pre-append header read. New/salvaged/empty-recovery/
rotated FILE_OPEN resets endpoints. Logger failure makes the published inventory stale;
failed rawWrite never advances metadata. Boot records establish last-written before ready.

Recovery headerGeneration body is unchanged. A wrapper delegates to it for metadata's
separate strict parser; host tests pin the recovery body at 229a344 (normalizing CRLF only)
and exercise shape-valid/calendar-invalid recovery acceptance. Metadata validates dates,
offsets, quality and common uint64 fields. Final exact clock_source=test is labelled test.
Archive tail boot/sequence must follow the header, except the same header-only record at
byte 0. This is only a stale-data plausibility filter, not integrity proof. File naming
continues to work with start-unknown when reliable start time is unavailable.

HTTP attachment names add YYYY-MM-DDTHHMMSS+/-HHMM_ for synced valid starts, otherwise
start-unknown_. Existing download boot/ID/file identity/frozen size retained. HTTP readers
perform one bounded 1024-byte head read after open/fstat (after pause for current), require
lseek(reader,0,SEEK_SET)==0, then publish start timestamp with size. Read error or bad restore
fails before body/metadata; malformed complete header falls back to unknown name. This read
never credits bytes/CRC/progress and another guard check precedes successful start. USB
start does not call the metadata reader. No current listing read descriptor, sidecar,
on-card rename/migration, NVS changes or timeout tuning.

## Ownership, allocations and cache

New diagnostics_log_time.{h,cpp} owns pure parser/formatter, writer endpoint state and one
2049-byte PSRAM scratch allocated before inventory enable, freed only after writer/cache
quiescence. Shared scratch is writer-only, used serially by archive inventory and HTTP
reader startup; transfer start preempts inventory before use. HTTP task receives compact
Time copies, never scratch or SD descriptors. Time <=16 bytes by static assertion.

Inventory wraps original FileEntry in a separate Entry, adding endpoints plus retry deadline;
Entry <=64 bytes total by static assertion, two 256-entry slots <=32768 bytes. Original USB
FileEntry and wire schema unchanged. Matching archive(number,size) metadata carries forward
from immutable published slot. Current row refreshed from writer memory at publication.
One uncached archive per tick: open/fstat/head<=1024/tail<=2048/close in the same call,
including errors. Bound by descriptor fstat; mismatch gives unavailable. No metadata file
descriptor crosses a writer turn; do not rely on prune calling inventory preempt.

At most 8 new archive reads per publication cycle, then pending rows are published and next
cycle starts immediately. Read/seek/close failure ends reads for that cycle and imposes a
per-entry 5 s retry deadline carried in published metadata: failing entries cannot monopolize
immediate pending cycles. Stop/revision/busy cancels unfinished staging; published progress
is immutable and survives. Archive cache is disposed with mode lifetime. Byte budgets do
not bound SD syscall latency; hardware must measure the added cost.

PAGE_CAP=65536 PSRAM, +32768 only while server allocated. Named row budget 224, fixed 2048,
256 rows plus NUL gives 59393; actual maximum-field renderer test fits that bound and verifies
pin release on overflow. Wrapped preformatted rows avoid intentionally fixed-width overflow;
iPhone rendering remains unmeasured. No pagination/query-input change. Allocator failure
retains existing startup rollback. HTTP stack 6144/internal and lifecycle 4096/PSRAM unchanged;
Time adds <=16 bytes per copied View/Result plus local formatting buffers, and writer parser
adds call depth. Read both http_margin and writer stack minimum on first bench case.

## Validation and review focus

241 host checks pass: lifecycle 43, HTTP transfer 35, new log-time 20, reader 28, retrieval 29,
media 15, USB connection 16, USB logger 12, browser 19, network 16, operation 8. git diff --check
passes. Source-body JS adaptations/mocked SD only, not C++ compilation or FatFs/RTOS proof.

New coverage: strict calendar/offset grammar with unchanged recovery; unknown/approx/test
clock naming; uint64 overflow and full precision; header-only and cross-boot tail plausibility;
incomplete/malformed/oversized tails; read/seek/close errors and descriptor-close count;
explicit restore-to-zero; writer endpoint boot/rotation/empty-recovery updates and write-success
ordering; full cache cold fill/sort/carry-forward, busy/pinned/preempt/stop/revision cases;
failure retry progress; allocation field bounds. Existing renderer executes the expanded
actual formatPage with 256 worst-case rows, and existing USB checks remain passing.

Please scrutinize before JP builds:
- C++ reference/struct/alignment behavior beyond source simulations; compact stack footprint.
- Scratch lifetime through inventory stop and pending HTTP reader startup.
- FatFs descriptor close/failure behavior; same-turn close is attempted exactly once, I/O
  failure is surfaced as unavailable and retries bounded, not silently treated as success.
- Metadata parser trust boundaries and recovery body preservation; no header metadata may
  affect boot salvage or file bytes.
- Cache starvation/preemption and same-generation writer metadata across append/rotation.
- Actual worst-case formatting and safe attachment path with reader offset restored.

## Next steps after review, not issued as a bench case yet

Claude review first. After clearance JP builds/flashes the same 3.3.11 profile; companion.ino
unchanged, so no generated sketch deletion required. First single case: choose an existing
immutable archive with a synced valid first header; compare listing endpoints and dated
export filename to log bytes, require exported Safari byte equality/CRC and unchanged
memory gate, inspect HTTP and writer stack margins. Request actual file list/reference
before choosing an archive if existing references no longer match. Then current growth/
rotation and unknown-time behavior separately, one case at a time. No hardware result is
claimed by this handoff and no new tests are requested from JP until review clearance.
