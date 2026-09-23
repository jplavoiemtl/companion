# Event-time log discovery - revision 2, for JP implementation approval

September 23, 2026. JP accepted increment 6 after complete gate C evidence at 2d3ff16
and asked to proceed. This document is the agreed next design-review step, not firmware
implementation approval. Increment 7 remains unapproved. No build, flash or new test now.
Historical sd_iphone_log_download_plan.md remains verbatim.

Revision 2 resolves Claude review 229a344 (B1-B3), adopts the single-page layout and
adds the qualifications below. No firmware changes. JP implementation approval is next.

## User outcome and scope

Find a log after an unusual car event without knowing its archive number. Add readable
file-start and last-record timestamps to the cached download listing, and a trustworthy
start timestamp to the exported filename. Retain existing SD filenames and managed routes;
no migration, sidecar files, new NVS state, on-card index or change to log record format.

Listing labels are **File opened** and **Last complete record** for archives,
**Last record written** for current.log, with local
UTC offset on each endpoint. For example: archive 21, opened Sep 22 14:10:03 -04:00,
last record Sep 22 15:32:10 -04:00. Current.log is marked growing; its listing reflects
cached bytes, not the later frozen download. Keep archive number and byte size visible.
Sort archives by number ascending, current last, on the writer before publication.
Do not sort by uncertain dates. The previous directory order is not a stable ordering.

These are navigation hints, not a guaranteed min/max interval containing every event.
Queued events carry their capture time and can predate FILE_OPEN after rotation; wall
clock adjustments and multiple boots can also break monotonic order. Boundary-only reads
cannot detect internal time discontinuities. Do not label endpoints as complete coverage
or silently swap them if end precedes start. A full time index is outside this proposal.

## Source facts checked

- sd_diagnostics.cpp createCurrent writes FILE_OPEN with local/time/boot/up_ms and
  generation; openStorage normally appends across boots. Boot time is not file-start time.
- Existing format is local=YYYY-MM-DDTHH:MM:SS.mmm+/-HH:MM plus time=unknown/approx/synced.
- LINE_CAPACITY is 1024; valid generated records are shorter than that including newline.
- Recovery can preserve a malformed header under an archive identity or add a newline
  to an incomplete tail. Such bytes must not be assigned an invented date.
- diagnostics_inventory.cpp currently scans only names/stat on the writer, publishes
  pinned immutable double buffers, yields every eight directory entries, invalidates on
  changes, and preempts scans for shared reader work. HTTP itself never opens an SD file.

## Timestamp interpretation

Parse only complete newline-terminated records, with bounded tokens, strict calendar,
time/offset and numeric validation. Do not trust arbitrary text embedded in fields.
File-open time is accepted only from a valid first FILE_OPEN record, format=1, with
valid generation and common fields. Keep headerGeneration() byte-identical, including
its shape-only timestamp acceptance: it decides recovery/salvage and must not become
stricter. A separate metadata layer first requires that existing validation, then applies
strict date parsing. Calendar-invalid but shape-valid headers remain recovery-valid and
metadata-unavailable. Pin those distinctions with host tests.
Do not insist archive suffix equals header generation: salvage/fixture history can differ.
A malformed or missing first header means start unavailable, not first later synced event.

Synced timestamps are shown normally; approximate ones are explicitly labelled approximate.
Unknown time means unavailable. No inference from FAT mtime, download time or current clock.
Last complete record uses the same quality labels; do not skip an unknown/malformed last
record in search of a nicer timestamp. A trailing non-newline fragment is flagged incomplete;
show the preceding complete record only when its full boundaries are within the bounded
window. If no safely parsed endpoint exists, keep size/identity/download available and show
unavailable. Endpoint timezone offsets come from each stored record, not today's DST offset.

### Clock and tail qualifications

Strict metadata calendar bounds: year2024-2099, actual month/day including leap years,
hour00-23, minute/second00-59, ms000-999, exact separators. Offset signed, minutes00/30/45,
absolute total at most14 hours (14:30/14:45 rejected). Unsupported offsets give unavailable;
recovery acceptance unchanged. Synced means SNTP-anchored this boot with no detected
unannounced jump over2 seconds, not a guarantee of recent revalidation.

Recognize only the exact final clock_source=test token before newline as test-clock metadata;
show test clock, never an ordinary approximate date or a trustworthy filename prefix.
File-open time may be days before the download boot because current.log spans boots.
Empty-recovery FILE_OPEN marks first bytes written after empty recovery, not lost history.
Unknown initial time stays unknown even if later records have synced times.

For archives, parse header and last record boot/seq pairs as uint64. Both boot IDs must
be nonzero; absent/invalid identifiers make tail plausibility unavailable. Require the
last pair lexicographically greater than the header pair when these are different records.
Exception: a header-only file whose final complete record is the same record at offset0
may have equal pairs. Establish sameness by byte position, not equal values. A lower/equal
pair in a different record is inconsistent. This detects some stale tails, not arbitrary
corruption: NVS reset/card migration and stale bytes with larger counters can defeat the
assumption. Do not describe this as proof of integrity or alter logger recovery based on it.
Keep endpoint hints and raw download available even when tail is inconsistent.

## Writer-memory current metadata

Never open current.log for listing metadata while the append descriptor is open. FatFs
sharing restrictions make a concurrent read handle unsafe; no new descriptor is needed.
The inventory obtains a writer-only copy of current metadata, then publishes immutable
cached values for HTTP. Current rows remain growing/advisory due to publication age.

Track FileEntry identity/size and endpoints on the writer. createCurrent and empty recovery
set opened only after successful FILE_OPEN write. At append boot, parse the existing header
from the buffer already read before append open; do so before buffer reuse without changing
headerGeneration or its recovery result. Set last-written after every successful complete
record through direct/writeRecord, using the actual formatted record's timestamp/quality/
test flag and boot/seq. Freeze the formatted endpoint before rawWrite; publish it only on
success. Do not reformat a Stamp later using potentially changed timezone state. Partial
writes never advance last-written metadata; mark metadata stale/unavailable on logger failure.
The recovery newline alone is not a new complete-record timestamp. Boot records establish
the new last-written endpoint before ready. Reset per-file state on creation/salvage/rotation;
never carry an old file endpoint into a new generation. Include tests for every lifecycle.
No current-file tail reads, forced flush, or pause for listing. Writer-memory last-written
can include unflushed bytes; it is not a durability guarantee. Transfer pause still flushes.

## Bounded archive reader and cache

One writer-owned2049-byte PSRAM scratch while inventory enabled. Per archive: open,
fstat, up to1024 head bytes, up to2048 tail bytes, close ALL within one writer turn.
Bound windows by descriptor size; if it differs from directory size, mark changed for this
cycle. Offset0 is a record boundary. Otherwise discard a leading partial record; do not
scan further to compensate for malformed/oversized records. Short reads give unavailable.
No archive descriptor persists across ticks, including errors, cancellation and allocation
failures. Therefore writer prune/rotation/transfer dispatch cannot run while it is open.
The earlier claim that prune calls inventory preemption was wrong: it only invalidates
revision. No new dependency on prune calling preempt is introduced.

One uncached file per writer tick maximum, with ordinary queue work between ticks; a
reservation arriving mid-read waits for this bounded byte-work unit and then wins before
another candidate. Check run/readiness/reservation before starting. No resumable per-read
phase machine. SD calls themselves can block; measure, do not promise a hard millisecond
bound from byte limits. Close failure stops further metadata scanning for that cycle,
reports unavailable/error and must not leak a descriptor across turns by retry logic.

Archive cache key(number,size), scoped to one mode lifetime. Copy matching completed
metadata from immutable published slot into unpinned staging slot during directory pass.
No third cache array. Cap new archive reads at8 per publication cycle, publish remaining
rows as pending, then immediately schedule next cycle while work remains; use normal5 s
refresh after completion. Each archive read still occupies its own writer tick. Discard
unfinished staging on revision/preemption; published progress survives. Writer offline,
new mode lifetime and card replacement discard cache. Bound failure retries at normal5 s,
not an immediate retry spin. Current row comes from writer memory each publication.
Optional rotation seeding is omitted. No whole-file scans or HTTP-task SD access.

## Single-page layout and resource bounds

No pagination. Three short lines per file: linked name/size, opened endpoint, last endpoint;
current is marked growing and snapshot-on-download. Display fixed legends explaining
per-record offsets, download boot versus file-open time, and endpoint hints. Wrap long
labels/status text with CSS (including preformatted rows) rather than asserting three
lines alone guarantees no horizontal scroll; verify on iPhone13 Pro. Retention normally
means30 archives plus current, but support all256 directory entries.

PAGE_CAP=65536 in PSRAM while server allocated (was32768); no HTTP internal-stack increase.
Named row maxima112+44+60 rounded up to224, fixed part2048, NUL1:
256*224+2048+1=59393 <=65536. These are design budgets, to be proved by maximum-value
host rendering of actual markup, escaping, labels and CSS. Never silently truncate rows.
If actual layout exceeds this bound, simplify markup or return for design review; no
unapproved allocation increase. Existing startup rollback handles allocation failure.
Do not infer a hard5 s send guarantee from previously measured average throughput.

Inventory-specific entry <=64 bytes TOTAL including FileEntry and metadata; two slots
<=32768 bytes plus2049 scratch. Keep endpoint boot/seq only in temporary parser state for
plausibility checking; do not duplicate four uint64 counters into every cached entry.
Store compact local timestamp/quality/status per endpoint. Enforce actual sizeof limits
with static_assert. Writer current metadata is fixed-size, no per-write heap allocation.
Download View added timestamp payload <=16 bytes, since View is copied on HTTP stack.
PSRAM page grows32 KiB; metadata adds bounded PSRAM plus small writer state. Actual internal
heap/stack overhead still requires measurement; do not claim all internal use unchanged.

## Export filenames tied to the actual reader

Proposed synced filename: 2026-09-22T141003-0400_116-2-current-1477034.log (or archive ID).
Offset retained, no colon; existing boot, transfer ID, identity and frozen size retained.
Start-unknown filenames retain those identifiers with a start-unknown prefix. Approximate
start is shown as approximate in listing but uses start-unknown in filename for version 1,
so a filename cannot silently imply trustworthy synchronization. Formatting only uses
validated numeric components, never raw SD text or browser headers.

Do not copy an advisory current-listing timestamp blindly into Content-Disposition.
After writer opens/fstats the actual download reader (after pause for current), perform
one bounded head read immediately after open/fstat while positioned at0, then require
explicit lseek(reader,0,SEEK_SET)==0 before metadata publication or readChunk. No pread
with hidden restoration. A negative OR nonzero return aborts before HTTP headers.
Publish parsed start metadata atomically with frozen size. Metadata bytes are not body
progress, do not update CRC or no-progress clocks, and are never emitted as body. Any
seek/position-restore failure aborts through normal close/resume. An unreadable or malformed
header can produce unknown metadata only when reader offset is known restored and subsequent
normal reads remain safe. No separate HTTP-task SD access or independent snapshot reader.
This validates attachment metadata for archives too, including changed/unavailable cache.
Do not add tail reads to the actual transfer path. Preserve cancellation/queue checks and
120000/5000 limits across this added startup work; metadata wait remains measured.

## Review and future validation gates

Claude should challenge: parsing trust and legacy fallbacks; endpoint semantics versus
true time coverage; generation/cache invalidation; writer scheduling and preemption;
allocation and page capacity; descriptor positioning and partial-read failure; immutable
HTTP publication; filename compatibility and preserved USB regression assertions.
The bounded single-page layout above resolves the layout decision. Recommend this
as a separate follow-up (6A) before increment 7, with acceptance distinct from increment 6.

Future checks after implementation approval: host parser matrices for synced/approx/unknown,
malformed calendar/offset/header, incomplete tail, short/large files, boot changes and
clock reversal and header-only files; unchanged recovery acceptance; writer endpoint
create/append-boot/empty-recovery/salvage/rotation tracking; no descriptor crossing a tick;
cache lifecycle/preemption, sort order; failed/nonzero position restore abort before headers;
transfer offset and CRC preservation; record,
filename and worst-case page capacities. Keep existing host suites intact.

First future bench case, issued only after Claude code clearance and JP build: one existing
immutable archive with a synced first header; compare listed boundaries and exported name
to actual log bytes, checking listed endpoints against available reference files and
requiring byte equality/CRC, http_margin and unchanged memory gates. Follow with
current growth/rotation and unknown-time handling separately, one case at a time. Do not
relabel old files or alter the clock to manufacture a case without a separately reviewed
procedure. After this follow-up, return to the still-required failure/resource/competition/
server-off regression and car-readiness increments. This proposal does not waive them.


## Review disposition

B1 resolved by writer-memory current endpoints; B2 by one-turn archive open/read/close;
B3 by byte-identical recovery validator with separate strict metadata validation. Adopted
single-page64 KiB PSRAM layout, archive-number ordering, test-clock labels, explicit seek
restore and bounded View. Qualified tail plausibility for header-only files and its limited
ability to detect stale data. No firmware implementation has started. JP may approve6A
implementation; code will go to Claude before JP builds, then one bench case at a time.
