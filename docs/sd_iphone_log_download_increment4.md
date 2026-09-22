# Increment 4 - small immutable archive HTTP download

Status: implemented for Claude review, September 22, 2026. JP approved increment 4 after
accepting increment 3 at d2095fa. JP reconfirmed no capability token for actual log bodies
on his trusted hotspot. No firmware build, flash or hardware measurements performed.
Increment 5 remains unapproved; historical draft is untouched.

## Behavior and response contract

Archive rows link to /f/<eight-digit archive number>. This managed identity never falls
back to a newer file. current.log stays visible but USB-only. Missing archives return
404; occupied reservation/unavailable logger returns 503. HEAD/other methods remain
405/Allow GET before reservation; favicon remains 204 and SD/session-free. Range and
If-Range arrival (including empty values) is recorded, but reply is full 200. Second
connections still queue behind the synchronous HTTP handler; interleaving is unchanged.

Downloads use application/octet-stream and Content-Disposition attachment with
boot-transferID-archive-number-frozenSize.log. Content-Length is the frozen file size.
The installed esp_http_server.h documents httpd_send at line 1452 as raw handler output,
returning actual positive bytes through the configured send override. Headers are built
explicitly; body is not chunked. After a post-header failure the handler closes without
adding error text. Header output has its absolute 5 s deadline; body output slides only
on positive writes. The shared reader also applies its unchanged stall guard, including
request-to-first-body time. Archive appends never pause; CURRENT_MS stays 120000 and
STALL_MS stays 5000. No throughput tuning, new task or task-stack change.

The last-result page shows expected and transport-accepted bytes, prefix CRC32 and result.
It explicitly distinguishes device send completion from verified phone save. Incidental
requests do not overwrite the result. No token, arbitrary path, upload/delete, CORS or
external asset. STA-only/local-address admission, deliberate powered entry and five-minute
inactivity expiry remain. JP deliberately reconfirmed the hotspot trust boundary.

## Ownership and cancellation

New diagnostics_http_transfer.{h,cpp} is a fixed mailbox, with a monotonic transfer ID
separate from the shared reader reservation generation. The existing writer-side USB
queue consumer dispatches HttpArchive to this adapter. USB paths, framing, 144-byte chunk,
CRC/line progress, limits and existing assertions remain. Archive start omits USB event
hooks; HTTP emits its own BEGIN/META/END records.

The writer publishes a copied 144-byte chunk with absolute offset. HTTP snapshots it into
private storage and reports cumulative positive bytes; writer progressBytes consumes only
the delta, updates CRC, keeps the unsent tail and never credits headers/prefetch. The mailbox
is overwritten only when the complete previous chunk is acknowledged. No writer holds
a lock across filesystem or network calls. Nested admission locks are transfer -> reader;
no inverse nesting. online is cleared before terminal queue drain so a late request cannot
reserve after the writer has gone away.

A: cancellation invalidates progress, closes reader and leaves append activity independent
of HTTP. Prune calls the same close path before unlink. B: HTTP posts matching release
only after its final send and stops using request/socket/buffer; component socket cleanup
still occurs on the HTTP task. Invalidated identity still accepts matching release;
stale identity cannot release new storage. Worker teardown waits for handlers plus the
HTTP reservation. Writer refusal to release remains retained and escalates through main.

**Review the terminal ownership refinement:** if B arrives after the SD writer has already
published Parked/Deleted, main's existing diagnosticsUsbOfflineTick calls a no-SD finalizer.
Only already-closed/released storage can be freed there. No wait is added to diagnosticsClose,
and no network wait is added to the writer. This is an explicit extension of writer-only
release after terminal publication, recorded in spec revision 6. Review the lifecycle
publication ordering and late acknowledgement interleavings particularly carefully.

An HTTP task waits at most 5 s for metadata, and at most 5 s for writer cleanup before
posting release with a pending-close error. These waits are on HTTP only. A pending writer
cleanup requests lifecycle failure; no forced buffer free. If writer cleanup subsequently
finishes, last-result close time/reason updates. A record emitted before that completion
cannot claim its later close time. Power/SD failure may prevent persistence as before.

## Records and timing gate A

HTTP_GET_BEGIN: transfer ID, archive ID, request time, Range/If-Range presence; no size/CRC.
HTTP_GET_META: frozen expected bytes, header completion time, reader-start time.
HTTP_GET_END: transport bytes, exact accepted-prefix CRC32, result, first/last body time,
maximum progress gap, terminal gap, cancellation publication, reader close and transport
release. resume_ms=0/appends=unpaused explicitly denotes archive behavior, not measured
current.log pause/resume. HTTP_GET_MEM supplies endpoint internal free/largest and HTTP
stack margin. Existing status/health supply queue high/drop, writer margins and sampled
minima. These are instrumentation, not measurements made by Codex.

The first bench gate after Claude review and JP build is timing gate A on one small
immutable archive. Select an existing small archive from inventory; do not silently
substitute a 2 MiB archive or generate/delete logs to obtain one. Capture USB reference,
laptop curl full headers/body and byte/CRC comparison, actual iPhone Safari save exported
for byte comparison, device BEGIN/META/END/resources and final append/no-drop status.
All belong to that single selected-file gate; exact steps are issued only after review.
No current.log HTTP, stress case or next increment is authorized here.

## Validation and review request

202 host checks pass across ten suites; git diff --check passes:

| Suite | Checks |
|---|---:|
| HTTP lifecycle | 43 |
| HTTP transfer | 22 |
| Reader/session | 22 |
| Retrieval mode | 29 |
| Media admission | 15 |
| USB connection/pacing | 16 |
| USB logger gates | 12 |
| Browser | 19 |
| Network diagnostics | 16 |
| Operation diagnostics | 8 |

New checks execute actual handler/mailbox/reader bodies under mocks: partial sends,
independent known CRC values, empty/missing archives, full Range reply, shared busy refusal,
shutdown-before-dispatch, partial mailbox acknowledgement, prune, stale release, blocked
send cancellation, body versus header deadlines and retained failed release. Existing
USB assertions remain; mocks were adapted to the idle HTTP branch. These are not a C++
compile, RTOS/lwIP proof, throughput measurement or hardware resource result.

Claude: review particularly raw httpd_send semantics/partial accounting; cancellation
between send and acknowledgement; offline release ownership; lock order and queued
shutdown; prune safety; visible stuck recovery; response/status and no post-header error
body; exact CRC prefix; 4096-byte HTTP stack and added fixed memory. Independent review
precedes JP build/flash. companion.ino is unchanged, so generated sketch deletion is not
required. No code has been compiled by Codex. Existing minor naming/idle-worker polling
notes remain deferred; this increment does not change lifecycle scheduling.


## Claude review follow-up - dual CRC, September 22

Claude's review at 0bb2eb0 found no blocking defects and independently confirmed 202
checks. Codex implemented the recommended writer CRC comparison before JP builds;
this follow-up requires Claude's quick diff review. No firmware build/flash.

The writer publishes its final acknowledged byte count and finalized CRC atomically
with closed=true. HTTP_GET_END keeps crc32 as the HTTP transport-accepted prefix CRC,
and adds writer_bytes, writer_crc32 and crc_check. The last-result page carries both.
Comparison values: match, mismatch (equal lengths only), prefix_diff (different covered
lengths), unavailable (writer not closed or identity not matching). Equal-prefix mismatch
turns an otherwise ok result into crc_mismatch; an existing abort/error reason is retained
alongside crc_check=mismatch. No payload or progress accounting is changed.

An in-flight send may be accepted after writer cancellation invalidates acknowledgement,
so HTTP can legitimately cover a longer prefix. Reporting both byte counts prevents that
from being called corruption. Late writer close updates the retained last result's writer
coverage/comparison; an already-emitted END remains an observation at its emission time.

208 host checks pass: HTTP transfer 28 (six added); other counts unchanged. Added checks
force equal-length CRC divergence in the actual handler and assert END contains mismatch,
verify match, unequal cancellation prefixes, unavailable identity/close, late close and
maximum-width END fitting the existing 456-byte field buffer. Host uint32 assignments
are explicitly modelled so JavaScript signed XOR does not create a false mismatch.
git diff --check passes. USB code and companion.ino are unchanged by this follow-up.

Review's low notes: terminal fallback type/assertion hardening and narrower hot-path view
accessors remain deliberately deferred to avoid expanding this correction; the existing
reviewed call-site exclusion still applies. The two added writer coverage fields modestly
increase View/Result copies; measure HTTP_GET_MEM http_margin first in gate A, before
interpreting throughput or CRC. No claim of measured stack safety is made by host tests.
Increment 5 remains unapproved. No generated sketch deletion needed for these src changes.


## Claude clearance and gate A preparation - September 22

Claude cleared the CRC follow-up at 08479cd and reran all 208 checks. No firmware change
after clearance. His low note about a missing record-size assertion is already covered
by the final test in tools/tests/http_transfer.test.cjs, added at 01a116e: it substitutes
maximum uint64 widths, CRCs and longest current labels into the actual END format and
asserts length below the actual field capacity. It evaluates to 447 bytes (<456), not
approximately 140 bytes of headroom at pathological widths. No duplicate test needed.

JP may build/flash amoled-1-8-core-3-3-11: logging and PSRAM writer on, both test switches
zero. companion.ino unchanged, so no generated sketch deletion required. Compile errors
go directly to Codex. Gate A starts with selection of one existing small immutable archive:
after flash, console DTR=true/RTS=false, USB power and hotspot connected, capture status,
log status and log list. Send actual listed names and byte sizes (the console capture may
only summarize the list, so copy the Files table if needed). Keep mode OFF and do not
start HTTP transfers yet; choose the small archive from evidence before issuing the
single-file download steps. No new fixture or larger archive is silently substituted.

After selection, gate A will capture server stack margin first, dual CRC comparison,
then exported Safari bytes against the reference. No hardware result yet. Increment 5
remains unapproved.
