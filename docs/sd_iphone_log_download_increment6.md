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


## Claude clearance bb9db3e - first gate C case issued

Claude reports no blockers and independently confirms 219 checks. JP may build/flash
amoled-1-8-core-3-3-11 with logging/PSRAM writer enabled and both test switches zero.
Actual sketch.yaml profile uses core 3.3.11, PSRAM enabled. No code changes after clearance;
companion.ino unchanged, so no generated sketch deletion. Assistant does not build/flash.

One normal Safari current.log snapshot, ordinary logging, phone awake/foreground:
1. USB power, hotspot connected, console DTR=true/RTS=false. Capture status and log status.
2. log mode on, await ACTIVE. Open reported URL on iPhone Safari; refresh listing and
   select current.log once. Leave ordinary logging running and avoid other transfers.
3. After completion, capture Last result and export the actual saved <boot>-<id>-current-
   <size>.log unchanged to PC Downloads. Keep distinct from the later USB current file.
4. Capture log mode status and status; log mode off, then log mode status confirming OFF.
5. Download current.log over USB with CRC OK, wait about 10 seconds, capture log status
   and log list. Send the console, Safari export, result capture and USB file/local paths.

Read HTTP_GET_CLOSE appends=resumed and paused_ms duration, then END crc_check, MEM
http_margin and saved bytes against equal-length prefix of the later USB file. pause_ms
is an absolute monotonic timestamp; paused_ms is the duration. Rotation after resume can
require a matching archive reference, selected from evidence rather than a whole-file
comparison against the new current.log. No before-download USB reference is required.

If Safari reports partial/failure, preserve it and capture the same status/USB evidence;
do not retry or change limits before review. Normal transfer result remains unmeasured.
Controlled queue-pressure case follows separately, not mixed into this case. No next
increment or timestamp feature implementation yet; both remain gated as recorded above.


## Gate C normal leg passes - controlled pressure remains

Boot 114 Safari current snapshot equals the later USB prefix, 1444367 bytes, CRC06AD67A4,
dual match, pause28230 ms, appends=resumed, HTTP margin2420, zero drops, clean104 ms exit.
See bench record. Next single case uses paced on-device Latest refusals during the paused
HTTP current transfer, at most ten taps one second apart, expecting logger_busy and resume.
No firmware change/build/flash. Gate C and increment 6 acceptance await this pressure evidence.


## Gate C complete - explicit acceptance pending

September23 boot116 controlled pressure repeat produced logger_busy, resumed appends after
12116 ms, zero drops, valid independently checked HTTP/writer prefix CRCs with one-chunk
prefix_diff, HTTP margin2416 and clean110 ms mode exit. See bench record for exact evidence
and the observed tap-count deviation. Combined with normal boot114 Safari prefix equality,
gate C passes. No further gate C results or rebuild needed. Await JP's explicit increment6
acceptance. Event-time discovery proposal is next for design review after acceptance;
implementation and increment7 remain unapproved. Timeout/stall bounds unchanged.


## JP acceptance - September 23

JP explicitly accepts increment 6 after gate C evidence at 2d3ff16. Earlier pending statuses
are historical. Next authorized work is the recorded event-time discovery design review:
sd_iphone_log_download_time_design.md. No new implementation or increment 7 approval inferred.
