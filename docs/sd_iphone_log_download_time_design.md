# Event-time log discovery - design for Claude review

September 23, 2026. JP accepted increment 6 after complete gate C evidence at 2d3ff16
and asked to proceed. This document is the agreed next design-review step, not firmware
implementation approval. Increment 7 remains unapproved. No build, flash or new test now.
Historical sd_iphone_log_download_plan.md remains verbatim.

## User outcome and scope

Find a log after an unusual car event without knowing its archive number. Add readable
file-start and last-record timestamps to the cached download listing, and a trustworthy
start timestamp to the exported filename. Retain existing SD filenames and managed routes;
no migration, sidecar files, new NVS state, on-card index or change to log record format.

Recommended listing labels are **File opened** and **Last complete record**, with local
UTC offset on each endpoint. For example: archive 21, opened Sep 22 14:10:03 -04:00,
last record Sep 22 15:32:10 -04:00. Current.log is marked growing; its listing reflects
cached bytes, not the later frozen download. Keep archive number and byte size visible.
Keep existing archive ordering/identity as authoritative; do not sort by uncertain dates.

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
valid generation and common fields. Reuse/refactor header validation with no weakening.
Do not insist archive suffix equals header generation: salvage/fixture history can differ.
A malformed or missing first header means start unavailable, not first later synced event.

Synced timestamps are shown normally; approximate ones are explicitly labelled approximate.
Unknown time means unavailable. No inference from FAT mtime, download time or current clock.
Last complete record uses the same quality labels; do not skip an unknown/malformed last
record in search of a nicer timestamp. A trailing non-newline fragment is flagged incomplete;
show the preceding complete record only when its full boundaries are within the bounded
window. If no safely parsed endpoint exists, keep size/identity/download available and show
unavailable. Endpoint timezone offsets come from each stored record, not today's DST offset.

## Bounded metadata reader and cache

Recommended: one writer-owned PSRAM scratch allocation of 2049 bytes, only while inventory
metadata is enabled. Read at most 1024 head bytes and 2048 tail bytes per candidate file,
using frozen stat size; never a whole-file scan or unbounded backward search. Overlapping
windows on small files may reuse bytes. Tail parsing discards a leading partial record
when the window begins inside a line. Oversized/malformed legacy records fall back to
unavailable. Do not spin on short reads; incomplete windows yield explicit unavailable.

Turn inventory metadata into a resumable state machine. One metadata read phase per writer
tick; return to normal queue draining between phases. Check mode stop, logger readiness,
shared reader reservation and inventory revision before each phase. Shared reader wins:
close metadata descriptor and discard unfinished candidate before transfer start. Prune,
rotation and shutdown call existing preemption; no metadata descriptor survives those
operations. Byte bounds do not bound a blocked SD syscall; retain existing measured SD
latency limitations, and measure the added work before claiming a timing guarantee.

Cache immutable archive metadata by managed identity AND size in the current mode lifetime.
Reuse completed entries during five-second listing refreshes; do not reread every archive
on every refresh. Do not persist cache across unmount, writer offline or a new mode lifetime.
Unplug/replacement and unexpected revision discard candidates. Current metadata is keyed
by current generation plus captured size; recheck that identity before publication and
mark old results stale. New appends cause bounded tail refresh, never a forced flush or
pause merely to render a listing. If unflushed bytes are not visible, label the snapshot
stale/advisory, not frozen or current to the millisecond. First header can be reused for the
same generation. Never scan while a current snapshot has appends paused.

Use inventory-specific entries wrapping existing FileEntry plus endpoint metadata; do not
expand USB wire format or USB inventory structures merely to support HTTP labels. Store
compact endpoint values and quality flags, format text only into the existing PSRAM page.
Target <=64 metadata bytes per entry (<=32 KiB added across two 256-entry slots), plus
2049-byte scratch. Enforce actual sizeof and allocation bounds with static/host assertions;
use fixed allocations, no per-file strings or heap churn. Allocation failure follows the
existing cache/server startup refusal; parser/read failure affects metadata availability,
not file bytes. Recompute worst-case listing output against PAGE_CAP=32768 before coding;
if full labels cannot fit, use explicit bounded pagination, not silent row truncation or
unreviewed internal allocation. Exact layout/pagination is a pre-implementation review item.

## Export filenames tied to the actual reader

Proposed synced filename: 2026-09-22T141003-0400_116-2-current-1477034.log (or archive ID).
Offset retained, no colon; existing boot, transfer ID, identity and frozen size retained.
Start-unknown filenames retain those identifiers with a start-unknown prefix. Approximate
start is shown as approximate in listing but uses start-unknown in filename for version 1,
so a filename cannot silently imply trustworthy synchronization. Formatting only uses
validated numeric components, never raw SD text or browser headers.

Do not copy an advisory current-listing timestamp blindly into Content-Disposition.
After writer opens/fstats the actual download reader (after pause for current), perform
one bounded head read using that descriptor and restore offset to zero before readChunk.
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
Resolve bounded page layout before requesting JP implementation approval. Recommend this
as a separate follow-up (6A) before increment 7, with acceptance distinct from increment 6.

Future checks after implementation approval: host parser matrices for synced/approx/unknown,
malformed calendar/offset/header, incomplete tail, short/large files, boot changes and
clock reversal; cache lifecycle/preemption; transfer offset and CRC preservation; record,
filename and worst-case page capacities. Keep existing host suites intact.

First future bench case, issued only after Claude code clearance and JP build: one existing
immutable archive with a synced first header; compare listed boundaries and exported name
to actual log bytes, require byte equality/CRC and unchanged memory/stack gates. Follow with
current growth/rotation and unknown-time handling separately, one case at a time. Do not
relabel old files or alter the clock to manufacture a case without a separately reviewed
procedure. After this follow-up, return to the still-required failure/resource/competition/
server-off regression and car-readiness increments. This proposal does not waive them.
