# Increment 6 - current.log HTTP snapshot

Status: implemented for Claude review, September 22. JP accepted increment 5 and approved
increment 6 after gate B at 33b992a. No firmware compile, flash or hardware measurements.
Increment 7 remains unapproved. Historical draft unchanged; companion.ino unchanged.

## Behavior

Exact GET /f/current appears on the cached listing. Listing size remains advisory; the
writer pauses appends, opens current.log, and fstats the reader to freeze Content-Length.
Attachment is <boot>-<transfer>-current-<bytes>.log. Archive attachment names unchanged.
BEGIN now uses the managed filename (current.log or archive-NNNNNNNN.log) instead of
only eight archive digits. HEAD/method/body rejection, Range full-200 policy, fixed
length, no-store, no token on JP's trusted hotspot and shared-session exclusion unchanged.

New Request::HttpCurrent follows both writer dispatch and queued-shutdown dispatch.
Shared reader treats it as current for pause and CURRENT_MS=120000, but skips USB begin/end
records, as HTTP archives do. STALL_MS=5000, half-full queue guard (8/16), closing, logger
failure, mode exit and explicit abort remain in force. No additional network wait on
writer/main. Cancellation resumes appends independently of the HTTP task's release;
matching invalidated-generation release remains valid, stale identities remain rejected.
USB wire behavior, USB events and its progress/deadline accounting are unchanged.

## Measurement and review focus

Reader records pausedAt before pause hook, readerClosedAt after reader close attempt,
resumedAt only after successful resume. These reset at each accepted start, including
archive reuse. Pause timing conservatively includes the pause hook's flush/close work.
Mailbox copies final timings with CRC coverage before publishing closed=true. Both
release-first/close-first paths update retained result; late completion cannot rewrite
an already-emitted record. Appends outcomes: unknown until closed, unpaused if never
paused, resumed if reopen succeeded, resume_failed otherwise. Logger failure remains
visible in result even if resume succeeded after a failed reader close.

END retains CRC fields, removes literal resume_ms=0/appends=unpaused; new HTTP_GET_CLOSE
carries measured pause, reader-close, resume, duration and outcome. This avoids expanding
END beyond its fixed 456-byte field capacity. Host maximum-width checks cover both records.
Last-result page carries cleanup evidence too. Failed/pending close has no completed pause
duration; paused_ms=0 must be read with outcome, not treated as a measured zero duration.

Review a corrected pre-existing spec assumption: BEGIN is queued after reserve; writer
may freeze before that queued record persists. Do not require BEGIN in its own snapshot.
END/CLOSE cannot be in their own snapshot. The later USB log supplies terminal evidence.
No network/SD ordering barrier has been introduced just to force BEGIN into the snapshot.

Three added uint64 timestamps enlarge reader state and View by 24 bytes, and Result by
24 bytes plus an outcome pointer/alignment. HTTP stack remains 6144 internal; lifecycle
worker remains 4096 PSRAM. Host assertions now pin these reviewed HTTP configuration
choices: stack size/caps, three sockets, LRU disabled. Recheck HTTP stack margin in gate C;
no claim that host simulations establish hardware stack safety or real SD timing.

219 host checks pass: HTTP lifecycle 43, transfer 35, reader/session 26, retrieval 29,
media 15, USB connection 16, USB logger 12, browser 19, network 16, operation 8.
New checks cover HTTP current frozen-after-pause ordering without USB events; queue,
stall, overall timeout and shutdown guards with resumed appends before reservation release;
open/pause/reopen failure telemetry; timestamp reset; queued HTTP current shutdown;
late-close result update; exact route/method policy; current attachment size/CRC; record
capacity and lifecycle configuration. Existing assertions retained; extraction signatures
adapted. These execute C++ bodies through JS adaptation and mocks, not firmware compilation.

## First bench gate after Claude clearance and JP build - not issued yet

One normal current.log Safari snapshot with ordinary logging, no simultaneous transfers.
Capture before/after status, HTTP result, exported phone bytes and a later USB current.log.
Compare the saved phone snapshot to the equal-length prefix of the later USB file, not
whole files: appends resume after the freeze. If rotation occurs, locate the matching
archive by contents/identity; do not compare an unrelated new current file. Require device
and independent CRC, exact bytes, measured pause/resume, unchanged memory gate, stack
margin, continued append growth and zero drops. A queue-pressure abort is evidence to
analyze, not permission to enlarge the queue, suppress normal events or extend timeouts.

Gate C also requires controlled queue-pressure early-abort/resume evidence before increment
6 acceptance, issued separately after the first normal case. Do not claim archive passes
establish pause safety. Slow-client failure work remains later and limits remain unchanged.

JP builds/flashes only after Claude clears this diff. No generated companion.ino.cpp
removal needed because companion.ino was not edited. The date/time filename and listing
proposal remains deferred for review after increment 6 acceptance, before final UI/car work.
