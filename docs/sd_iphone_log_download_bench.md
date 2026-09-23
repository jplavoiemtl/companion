# iPhone log retrieval - bench cases and results

## September23 - increment7 current cancellation/resumption passed, boot118

Console attachment5ef70ee5-ccdd-4bf6-b5b6-e3a01ed90c9c, Downloads/118-current.log
(1646157 bytes, USB CRC OK), listing.PNG and last-result.PNG inspected.
HTTP id5 current expected1642899; cancelled after643680 bytes, result=mode_exit,
writer bytes643680, both CRC555AAE7E, crc_check=match. Independently computed CRC
of first643680 bytes of later USB file is555AAE7E. No phone partial file supplied.
Cancel4123177 -> reader close4123178 (1 ms), append resume4123181 (4 ms),
transport release4123181 (4 ms). Total paused12547 ms; terminal gap4 ms,
max progress gap41 ms. Mode OFF4123338,161 ms after cancellation. Re-entry5 ms;
second exit107 ms, confirmed by record despite no final mode-status command.

HTTP_GET_MEM largest42996, margin2576; writer minimum2920, retained internal
largest31732, drops0/truncated0/errors none. Slow-write counter1 already present
before case and unchanged. No reset or stuck release. Post-USB growth1646157 to
1646324 (+167 bytes). Listing confirms resumed appends and recovered inventory
(stale0/busy0), current1644616 last-written10:04:20, retaining opening timestamp.

Correction after JP challenged the preview: original last-result.PNG re-rendered as
PNG at higher resolution clearly contains transfer5 mode_exit,643680/1642899,
CRC555AAE7E matching writer, appends resumed and the correct cleanup timestamps.
The earlier blank-page claim was an assistant image-preview interpretation error,
not evidence of a Safari/device failure. Withdraw the proposed page-reload check.
Earlier blank-image observations from the same preview workflow are unverified and
must not be used as evidence of a recurring UI defect. No firmware fix warranted.
6A natural rotation and overall increment7 acceptance remain pending.


## September 23 - increment7 archive cancellation and re-entry passed

Boot118, HTTP id4, mode generation4 then5. JP interrupted Safari archive21 using
log mode off about13.4 s into body. Safari removed the progress box without an
explicit error message; do not generalize prior visible-error expectations to Safari.
Console attachment f0c58d4f-f58f-4ef4-b0df-bfd14a2bf539 and
Downloads/118-current (1).log (1629527 bytes, console USB CRC OK) inspected.
JP subsequently supplied listing.PNG. Inspected: recovered listing is available,
stale=0/busy=0, age190 ms, current1627849 bytes / last-written09:52:16. It retains
id4 mode_exit,675216/2097146 bytes, CRC1EB9E912 matching writer and appends unpaused.
This independently confirms the refreshed Safari page after re-entry.

HTTP id4 expected2097146, accepted675216, result=mode_exit. Writer length675216,
both CRC32 1EB9E912, crc_check=match. This was cancellation during active body,
not after successful completion. Cancel3389399, reader close3389424, release3389425:
25 ms to close, 26 ms to release. Last progress3389397, terminal gap28 ms,
max progress gap62 ms. Archive appends remain unpaused throughout. Exit ok3389564:
165 ms from stopping to OFF. Re-entry succeeds in4 ms; subsequent exit103 ms.
Partial phone bytes were not supplied; CRC agreement here is device-side only.

HTTP_GET_MEM internal_largest49140, margin2660; retained console http_min2576,
writer stack_min2920, internal_largest31732, all unchanged/above gate. Drops0,
truncated0, no logger error/reset or stuck release. Slow write count1 /124355 us
already present in pre-case status, unchanged after: not attributed to cancellation.
Later USB current snapshot1629527 grows to1630722 (+1195), confirming appends.
Mode OFF, boot118 unchanged; natural rotation still pending (newest21).

Next single increment7 case: cancel current.log body with log mode off, confirm
append resumption and cleanup before re-entry. Existing firmware, no rebuild.
6A rotation and overall increment7 acceptance remain pending.


## September 23 - 6A current snapshot reference supplied: gate passed

JP downloaded current again. Downloads/118-current.log is1607658 bytes, same
file generation22. Its first1598639 bytes are byte-identical to Safari snapshot
2026-09-22T103643-0400_118-3-current-1598639.log, CRC32 45E93E67.
HTTP id3 END confirms both prefixes1598639, both CRCs45E93E67, crc_check=match,
result=ok, gap_ms=42. CLOSE reports paused_ms31464, appends=resumed, 6 ms
reader-close to resume. MEM internal_largest49140, http_margin2660 for this
transfer (console retains lower lifetime http_min2576). Exit completed110 ms
after stopping record. Current-file growth, naming, snapshot bytes and cleanup
checks pass; previous missing-reference limitation is resolved.

Remaining 6A hardware coverage: actual rotation metadata transition. Production
configuration has test hooks disabled; do not force rotation with test commands
or change firmware just to accelerate it. Allow normal logging to reach the2 MiB
limit, then inspect archive22 and newly opened current file. Latest reference
1607658 bytes is489494 bytes below2097152; elapsed time depends on event rate.
Next evidence before choosing downloads: log status/log list and refreshed Safari
listing once newest advances from21 to22. No build required, no gate waived;
6A acceptance and increment7 remain pending.


## September 23 - 6A current growth/snapshot, boot 118: USB reference pending

JP reports test ran fine. Console attachment207e34f3-27fa-4602-b0e5-8b0ad85c7c38,
Downloads/listing-1.PNG, listing-2.PNG, latest-result.PNG and
2026-09-22T103643-0400_118-3-current-1598639.log inspected.
Listing current row grows1596424 ->1597446 (+1022); last-written advances
09:26:57 ->09:27:18 on September23. Opened stays September22 10:36:43 -04:00.
Actual header is synced FILE_OPEN 2026-09-22T10:36:43.102-04:00, generation22,
boot105 seq36. Filename correctly truncates milliseconds and uses opening time,
not download date. Listing observations agree with writer-memory endpoint behavior.

Exported Safari snapshot is1598639 bytes, independently computed CRC32 45E93E67.
Last-result screenshot reports id3 ok, both lengths1598639, both CRCs45E93E67,
comparison match, appends resumed. Pause1997319, close2028777, resume2028783:
31464 ms paused, 6 ms close-to-resume. Screenshot is device telemetry; exported
file CRC independently matches. Snapshot ends with HTTP_GET_BEGIN id3, as expected;
END/CLOSE/MEM records occur after frozen snapshot and require later USB reference.

Post-transfer console: writer stack_min2920, retained http_min2576, internal_largest
31732 (>20480), logger ready, high7, drops0, truncated0, no error/reset. Mode OFF
confirmed09:30:34. USB current transfer reports1600849 bytes / CRC OK; subsequent
current_size1602387 confirms growth. Brief4-5 ms USB link loss, no pending loss/failure.

Later USB file is absent from Downloads at inspection (only Safari log and three PNGs).
Therefore equal-length prefix comparison and HTTP_GET_MEM/END/CLOSE inspection remain
pending; do not mark current-file gate complete. Ask JP to restore the1600849-byte USB
file or supply its location. No rebuild or Safari repeat needed. If unavailable, another
USB current download may supply the reference while generation22 remains current.
Rotation transition and 6A acceptance remain pending; increment7 unapproved.


## September 23 - 6A synced-date archive gate passed, boot 118 generation 2

JP reports no noticeable delay or pending message. Console attachment
1e3877f8-0d2c-46b9-b706-446253dbec41 and Downloads/listing-3.PNG inspected.
Safari 2026-09-20T193038-0400_118-2-archive-00000021-2097146.log equals
118-archive-00000021.log byte-for-byte: 2097146 bytes, CRC32 0B301A04,
SHA256 8f9aa99aa6b8bbc73518531469f0bb0c2762d27f651f786a8e8987f1e84b6433.
Header FILE_OPEN is 2026-09-20T19:30:38.993-04:00, synced, boot 92 seq 365;
filename correctly omits milliseconds without rounding 38.993 to 39.
Tail is 2026-09-22T10:35:50.794-04:00, synced, boot 105 seq 35.
Both listing endpoints match those bytes, including offset. Listing has no pending
rows; user observation is qualitative, not a measured cold-cache completion time.

118-current (1).log contains HTTP id=2: result=ok, bytes=writer_bytes=2097146,
CRC32=writer CRC32=0B301A04, crc_check=match, gap_ms=121. First accepted byte
1205795 to last 1247492 = 41697 ms. Appends unpaused; no cancellation.
HTTP_GET_MEM internal_largest=47092 and http_margin=2576. Before transfer,
writer stack_min=2920 and retained http_min=2656; after, writer remains2920,
http_min=2576. HTTP minimum is retained across mode entries and is not an isolated
listing-only measurement. Lowest captured internal_largest=31732 (>20480).
Drops=0, truncated=0, no logger error/reset, mode OFF confirmed 09:17:30.
USB reference/current downloads both CRC OK. One 5 ms USB link loss at final status,
within 1000 ms grace, no pending loss or failed transfer. Current grew from snapshot
1584721 to1586258 after transfer. No SD maximum latency increase (67459 us).

Listing-3 is before this transfer and correctly retains previous successful id=1;
new id=2 result is verified in the downloaded current log. Current row shows same
opened 2026-09-22 10:36:43 and last-written advanced to September23 09:14:21.
Unknown-start and synced-archive gates now pass. Next single case is current.log
listing growth plus dated Safari snapshot and equal-length later USB prefix.
Rotation transition not yet exercised on hardware; do not claim full 6A acceptance.


## September 23 - 6A first hardware run, boot 118: unknown-start archive passed

Firmware reviewed at 337cdc9; JP reports test ran fine. Console attachment
24f0c364-375c-435c-8f7b-1320337fc541, plus Downloads/listing-1.PNG,
listing-2.PNG, last-result.PNG, start-unknown_118-1-archive-00000017-308745.log,
118-archive-00000017.log and 118-current.log inspected.

Safari and USB archive 17 are byte-identical: 308745 bytes, CRC32 541A8F0C,
SHA256 c592bf6b49d7b52defab68fda97c4476ad2c33b737191ed99546d839da5dc28e.
Opening record is local=unknown time=unknown, boot=38 seq=53, FILE_OPEN generation=17.
Final complete record is local=2026-09-19T09:29:59.274-04:00, synced, boot=69 seq=8.
First dated record is CLOCK_SYNC on 2026-09-17T11:01:36.509-04:00, boot=39;
it is not evidence of the file creation time. Both listing screenshots correctly show
opened unavailable (clock unknown) and last 2026-09-19 09:29:59 -04:00.
The start-unknown filename is correct. This validates unknown-start fallback, not
synced-date filename generation: archive 17 did not satisfy that selection condition.

HTTP id=1 END reports result=ok, both prefixes 308745 and CRCs 541A8F0C,
crc_check=match, gap_ms=42. Appends unpaused. HTTP margin=2656 bytes;
writer stack_min=3208 at post-download status, worker minimum=2304 after stop.
Lowest captured probe largest_min=34804, HTTP internal_largest=42996, post-download
status internal_largest=36852: above 20480 gate. No drops, errors or boot change.
Current grew from USB snapshot 1564764 to 1565275 after transfer. Mode OFF confirmed
at 08:59:37; no stuck release. Separate listing-only stack snapshot was not captured.

Both listing screenshots have no pending rows, sorted archives 14..21 then current.
Cold pending-clear duration was not reported and cannot be inferred from screenshots.
Current row advanced from size 1560061 / last-written 08:56:09 to size 1562102 /
last-written 08:57:11, retaining opened 2026-09-22 10:36:43 -04:00.
last-result.PNG is blank; listing-2 nevertheless captures the full successful result.

Next single case: same firmware, archive 21 (2097146 bytes), whose listing has synced
opened 2026-09-20 19:30:38 -04:00 and last 2026-09-22 10:35:50 -04:00.
Verify its actual header/tail and Safari dated filename and compare with USB bytes.
Repeat cold-list observation and separate listing/transfer status captures in this case.
6A acceptance remains pending; no new implementation or increment 7 authorization.



## September 23, 07:53-07:56 - gate C controlled queue-pressure abort passes

Console attachment 80cebdf5-9bfb-4234-813d-c88567b372ea/Pasted text.txt and Downloads/
116-current (1).log (1482958 bytes, USB CRC OK) inspected. Same boot116, generation2;
HTTP transfer id1/current.log expected1477034. At 07:54:19 console confirms active=1,
paused=1, writer bytes217008, queue1/16. Latest refusals and ordinary motion records
then accumulated. Result is logger_busy, not a stalled/mode-exit abort.

HTTP request=401150, reader=401183, pause=401185, header=401192, first=401193,
last=413294, reader close=413294, resume/cleanup close/cancel publication=413301,
transport release=413302 (monotonic ms). Pause12116 ms, appends=resumed, terminal gap8 ms,
maximum progress gap43 ms. Reader close to successful reopen7 ms. Cancel timestamp is
publication during writer cleanup, not a separately sampled threshold-detection time;
do not infer pressure-detection latency from it.

HTTP accepted612288 bytes CRC0188C8D9; writer credited612144 CRC424AD70D. Independent
CRC32 over each corresponding prefix of the later USB log matches exactly. Difference144
bytes equals one mailbox chunk: compatible with positive in-flight send after writer
invalidation, correctly labelled crc_check=prefix_diff rather than mismatch. No partial
Safari export supplied or needed for this intentional-abort gate; this verifies source
prefix integrity and device accounting, not the phone's saved partial bytes.

Five Latest refusal records occur before END along with META and two ordinary motion
records; further taps occur after resume. Console contains12 taps rather than maximum10,
approximately0.6-0.9 seconds apart. The guard still fired and no overflow occurred; this
is bounded paced UI pressure with ordinary motion contribution, not an exact injected
queue-count experiment. Queue high-water rose6->11/16, drops=0/truncated=0. Eleven is
consistent with queued events plus END/CLOSE/MEM after cleanup; it is not proof the abort
threshold became11. The unchanged reader guard checks >=8/16. Logger remains ready,
error=none, slow=0, same boot. Subsequent status active=0/paused=0/queue=0 and log growth
1477034->1481117->1481284 confirms resume; after USB snapshot, current1483124 (+166).

HTTP_GET_MEM internal_free88052, internal_largest45044, http_margin2416. Post-case status
internal_min78200, internal_largest36852 (>20480), writer margin2936, worker2304.
Server error=none/release_stuck=0, accepted6/rejected0. Mode exit471702->471812 =110 ms;
successful exit record confirms OFF and stopped server despite no final mode-status line.
Later USB current.log succeeds with CRC OK and final loss counters0; the previously
recorded1 ms link loss predates this HTTP case. No reset or stuck reservation evidenced.

Disposition: controlled pressure leg passes. Together with boot114 normal snapshot and
exported prefix comparison, gate C required evidence is complete. Present increment6 for
JP's explicit acceptance. No more files, rebuild or bench measurements needed for gate C.
120000 ms current limit and5000 ms stall bound unchanged; slow-client/full-bound and
other failure cases remain future work, not claimed passed here. After acceptance revisit
recorded event-time listing/download-name proposal for design review; implementation and
increment7 require authorization. No firmware changes in this result recording.



## September 23, 07:47-07:50 - pressure attempt did not exercise HTTP transfer

JP reports pass; console d432db1b-998c-4eb8-8581-fe2a9868efd8/Pasted text.txt and
Downloads/116-current.log (1471707 bytes, USB CRC OK) inspected. Boot116, server generation1.
Server entered ACTIVE, but final active status has accepted=0, rejected=0, http_min=0.
All sampled transfer states before the later USB retrieval show active=0, paused=0,
bytes=0, result=none. Boot116 log contains zero HTTP_GET records. Thus no HTTP download
reached this server; no evidence establishes why (no assumption about Safari/user action).

Twelve Latest clicks generated twelve IMAGE_REFUSED trigger=latest reason=download_mode
records. With appends unpaused these drained normally; queue high stayed6/16, drops and
truncated remained0. Logger ready/error none, continued growth and no reset. Mode exit
163654->163768 ms, 114 ms, confirmed in log. USB retrieval succeeded; final size1472880,
1173 bytes beyond snapshot. One1 ms USB link loss after retrieval, no pending loss/failure.

Disposition: healthy refusal/logging behavior, NOT a pressure-abort pass. Prior normal
gate C pass remains valid; pressure leg and increment6 acceptance remain pending. No
firmware change or rebuild. Repeat the same single case with an explicit prerequisite:
start current.log from the iPhone Safari listing, accept Safari's Download prompt if shown,
then while it is transferring issue log status and require paused=1 before touching Latest.
If paused=0, stop and send status plus what Safari shows; do not continue button presses.
Once paused=1, tap Latest once/second, maximum10, stop on transfer failure. Capture Last
result, status/log status/log mode status, exit and confirm OFF, retrieve current.log via
USB, wait10 seconds and log status. Expect logger_busy, resumed appends and zero drops;
no successful phone file is required for the intentional abort. No later work authorized.



## September 22, 16:40-16:43 - gate C normal current snapshot passes

JP reports pass; console attachment dff28b32-b1b2-4951-be17-07788f0f6e36/Pasted text.txt.
Boot 114, generation 1. Downloads files read: 114-1-current-1444367.log (actual Safari
export) and 114-current.log (later USB, 1446892 bytes, CRC OK). Safari file equals exactly
the first 1444367 bytes of the USB log. Independent CRC32 06AD67A4; SHA256
677df2ac8163db1a91c981f36450ecc5cf77796c2797cc09d71b7aa0c32cf6b9.
The latest-result screenshot is supplied, but underlying records and saved bytes establish
the verdict. No whole-file equality expected; no rotation occurred (generation 22).

HTTP id=1, current.log: request=123465, reader=123498, pause=123500, header=123525,
first body=123526, last=151725, reader close=151726, resume/cleanup close=151730,
transport release=151731 (monotonic ms). Paused 28230 ms with appends=resumed;
request-to-release 28266 ms, about 51.1 kB/s, first body after 61 ms. Maximum progress
gap=61 ms, terminal gap=6 ms, cancel=0. Expected/HTTP/writer counts all 1444367;
both CRCs 06AD67A4, crc_check=match, result=ok. Reader-to-header interval includes
pause/open/fstat and scheduling, not an isolated measurement of flush duration.

MEM internal_free=81488, internal_largest=38900, http_margin=2420. Later status
internal_min=74236, internal_largest=34804 (>20480), writer margin=3128, worker=2304
after shutdown. Queue high=7 before and after, drops/truncated=0, slow=0, logger error=none.
Snapshot grew by 2525 bytes to USB capture; final current size 1448068, confirming continued
appends. Final completed transfer inactive/unpaused, queue 0; subsequent list temporarily
reports active=1 while listing, not a retained download. USB link losses=0.
Mode exit stopping at 203061, ok at 203165: 104 ms. Console confirms OFF/server=off,
release_stuck=0, accepted=9/rejected=0, no reset. Normal gate C leg passes.

Gate C remains pending a separately controlled queue-pressure abort/resume case. This
normal successful pause does not validate the full 120-second cap or change stall limits.

### Next single case - paced Latest refusals while current snapshot is paused

Same firmware, no rebuild. Keep dashboard Latest button visible, USB power and hotspot,
DTR=true/RTS=false. Capture status/log status, enter mode and await ACTIVE. Start one
Safari current.log download. About 3 seconds into the transfer, send log status to capture
paused=1 (if transfer already ended, stop and send evidence rather than continuing taps).
Then tap the companion's Latest button once per second, at most ten taps, stopping early
if Safari reports failure. No other action or concurrent transfer during these taps.

Source: Screen1 invokes buttonLatest_event_handler; requestLatestImage's download-mode
branch queues exactly one unsuppressed IMAGE_REFUSED trigger=latest per click, before any
image side effects. Appends are paused so records accumulate; writer stopReason aborts
at >=8/16 queued. Paced taps allow writer scheduling and should produce logger_busy,
close/resume, a truncated HTTP body and zero dropped records. This is a real bounded UI
stimulus, not test-hook injection. Ordinary records may cause the threshold before tap 8.

After terminal result capture Last result, log mode status, status and log status. Exit
mode, confirm OFF/server=off, download current.log via USB with CRC OK; wait 10 seconds,
capture log status. Save any partial phone file if available but do not require Safari
to expose it. Send full console, last-result screenshot and later USB file. Do not retry
or increase tap rate if abort does not occur. Evaluate result=logger_busy, prefix coverage
(match or legitimate prefix_diff), appends=resumed, cleanup timing, queue/drops, continued
growth and no stuck reservation/reset. Retain 120000/5000 limits. Increment 6 not yet
accepted; increment 7 and deferred timestamp implementation remain unapproved.



## September 22, 15:19-15:23 - gate B representative archive passes

JP reports test passed. Console attachment:
ce58c6da-e797-47f4-86e8-1cd04a894cb8/Pasted text.txt. Same boot 112, server generation 2,
unchanged accepted firmware. Downloads evidence read: 112-archive-00000021.log (USB CRC OK),
112-2-archive-00000021-2097146.log (actual Safari export), 112-current (1).log (1167643
bytes, USB CRC OK). Both archive files are byte-identical, 2097146 bytes (2 MiB minus 6),
independent CRC32 0B301A04, SHA256
8f9aa99aa6b8bbc73518531469f0bb0c2762d27f651f786a8e8987f1e84b6433.
The latest-result.PNG was supplied; verdict is verified from the underlying records and
saved bytes rather than relying on the screenshot. Reference is USB-retrieved SD data,
not a physical-card reread.

HTTP id=2, file=00000021: request=2838073, reader=2838094, header/first=2838115,
last=2879553, close=2879554, release=2879555 (monotonic ms). Time to first body=42 ms;
request-to-release=41482 ms, about 50.6 kB/s, consistent with the small Safari case's
about 50.4 kB/s. Maximum no-progress interval=42 ms, terminal gap=2 ms, cancellation=0,
resume=0, appends=unpaused. No measured throughput collapse over the larger file; aggregate
throughput and maximum gap do not establish a per-second rate distribution or phone save
duration. Range=0/If-Range=0 for this request only.

END expected/HTTP bytes/writer bytes all 2097146; both CRCs 0B301A04, crc_check=match,
result=ok. MEM internal_free=87344, internal_largest=42996, http_margin=2524.
Post-transfer status internal_min=76344, internal_largest=34804 (>20480); writer margin
2936, lifecycle worker margin 2304. Queue high-water=8/16 and slow=1 (147311 us maximum
flush) already existed before the case and did not increase. Drops/truncated=0 and logger
error=none throughout. No new stall or reset evidenced. Current size 1163376 before
reference retrieval, 1167129 after HTTP, 1168840 after snapshot; 1197 bytes appended after
the 1167643-byte current snapshot. Final USB inactive/unpaused, queue=0. One 2 ms USB link
loss was recorded by final status, below the 1000 ms grace, with successful CRC and no
pending loss or transfer failure; it was absent in the immediate post-HTTP status.

Console omits the requested final log mode status, but current.log supplies successful
exit: stopping=2928594, result=ok at 2928696, 102 ms. Source diagnostics_retrieval.cpp
emits that successful exit only after !diagnosticsUsbBusy() and diaghttp::stopped(), and
after setting Mode::Off. This closes shutdown evidence without repeating a bench action.
Accepted connections=6, rejected=0; console stopping state has error=none/release_stuck=0.

Gate B complete and passing. Present increment 5 for JP's explicit acceptance; increment 6
remains unapproved. No further files, build or measurement required for this normal case.
Next proposed implementation is current.log HTTP snapshot retrieval after approval and
Claude review before JP builds. CURRENT_MS/STALL_MS unchanged: successful archives do not
prove current-log pause safety or justify stall tuning. The event-time listing/filename
proposal remains scheduled for design review after increment 6 acceptance.



## September 22, 14:34-14:37 - gate A Safari continuation passes

JP reports the test passed and supplied console attachment
b1ccd0b5-370b-4b12-8abf-7f8f644b82e6/Pasted text.txt. Boot 112 on the rebuilt
6144-byte HTTP task stack. Evidence read from Downloads:
112-1-archive-00000017-308745.log and 112-current.log (1007893 bytes, USB CRC OK).
The exported Safari archive is byte-identical to 110-archive-00000017.log, the retained
USB reference: 308745 bytes, CRC32 541A8F0C, SHA256
c592bf6b49d7b52defab68fda97c4476ad2c33b737191ed99546d839da5dc28e.
This compares Safari export against SD bytes retrieved over USB, not a physical-card reread.
latest-result.PNG was supplied too; image-tool sandbox failure prevented viewing it.
The underlying END record independently supplies the result and both CRCs.

HTTP_GET_BEGIN id=1 started_ms=126819, range=0, if_range=0. META reader_ms=126844,
header_ms=126861; first body=126862 (43 ms from request), last=132943, close=132943,
release=132944. Request through release: 6125 ms, about 50.4 kB/s. Maximum progress
gap=43 ms, terminal gap=1 ms, cancel=0, resume=0, appends=unpaused. END expected,
HTTP bytes and writer bytes all 308745; both CRCs 541A8F0C, crc_check=match, result=ok.
These are device transport times, not a measurement of Safari's final filesystem save.

HTTP_GET_MEM: http_margin=2744, internal_free=89132, internal_largest=47092.
Post-transfer status reports internal_min=78792 and internal_largest=36852, still above
20480. Writer stack margin=3224; lifecycle worker minimum=2304 after teardown.
Queue high-water=7/16, drops=0, truncated=0, logger error=none, USB link losses=0.
Current size grew from 1004042 before entry to 1007385 after transfer and to 1008057
after the 1007893-byte USB snapshot: continuing appends, including 164 bytes after snapshot.

Exit record stopping at up_ms=191030, result=ok at 191133: 103 ms measured mode exit.
Console subsequently confirms OFF/server=off, error=none, release_stuck=0, accepted=5,
rejected=0, same boot. Later WiFi offline at 14:38:17 and reconnect at 15:01:51 occur
after this gate and mode shutdown; their cause is not established by this evidence.

Gate A evidence is complete and passing, including retained laptop leg and actual Safari
export. Increment 4 is presented for JP's explicit acceptance; increment 5 remains
unapproved. No further measurement or rebuild is needed for gate A. Next proposal is
representative 2 MiB archive work and timing gate B, one case at a time. CURRENT_MS and
STALL_MS remain unchanged; this successful archive does not establish current-log or
slow-client safety. No firmware changes in this result recording.



## September 22, 14:24-14:25 - gate A laptop shutdown confirmed

JP supplied the follow-up console: log mode off at 14:24:59.275, STOPPING at
14:24:59.279, OFF/server=off at 14:25:10.639. Generation 1, error=none,
release_stuck=0, accepted=1/rejected=0. HTTP minimum remains 696 bytes;
lifecycle worker minimum after teardown 2304. Probe largest_min=49140 throughout
the supplied idle windows, above the 20480 gate. The observation interval does not
measure actual stop duration. This closes the earlier missing shutdown evidence;
no repeat required. Stack increase c4bbba4 still awaits Claude clearance before
JP rebuild; gate A still needs headroom remeasurement and Safari export comparison.


## September 22, 14:19-14:21 - gate A laptop transfer integrity passed; stack headroom correction

Boot 110. JP reports test passed and supplied console/curl output. Local evidence read:
C:/Users/photo/Downloads/110-archive-00000017.log (USB reference, 308745 bytes),
gateA-17-http.log (308745), gateA-17-headers.txt, and 110-current.log (970203).
Direct byte comparison is equal; independent CRC32 541A8F0C for the HTTP body.
SHA256 c592bf6b49d7b52defab68fda97c4476ad2c33b737191ed99546d839da5dc28e.
This compares HTTP with SD bytes retrieved over USB, not a physical-card reread.

HTTP headers: 200, application/octet-stream, Content-Disposition attachment filename
110-1-archive-00000017-308745.log, Content-Length 308745, close/no-store/no-referrer.
Curl first response 0.080719 s, total 5.926519 s. Device id=1: request 276815,
reader start 276848, header 276856, first body 276857, last 282657, close 282658,
release 282659 ms. Request-to-release 5844 ms, maximum progress gap 42 ms,
terminal gap 2 ms, no cancellation and archive appends unpaused.
HTTP_GET_END bytes=writer_bytes=308745, both CRCs 541A8F0C, crc_check=match, result=ok.
HTTP_GET_MEM internal_free=90296, internal_largest=47092, **http_margin=696**.
Current USB download CRC OK; later size 971377 confirms 1174 bytes append growth,
logger ready, drops=0, queue=0/16, USB inactive/unpaused, no link loss. The supplied
capture and current.log contain no mode-off evidence for this case; do not mark exit
verified. No reset in the supplied boot-110 segment.

696 bytes on the 4096-byte HTTP task is a small measured margin on a normal path,
not an observed overflow or failure of a pre-agreed stack threshold. Proposed focused
correction: HTTP task stack 4096 -> 6144 internal bytes, +2048 only while server exists;
PSRAM lifecycle worker unchanged. Largest internal block was well above 20480 but the
new allocation still needs hardware remeasurement. Claude quick review before JP rebuild.
Keep the passed integrity/timing evidence; gate A remains incomplete for stack recheck
and actual Safari export comparison. Do not issue the Safari portion on this build yet.
No assistant build/flash. Increment 5 remains unapproved.


## September 22 - gate A archive selected; laptop portion issued

JP supplied Files table: current=964476; archives 21=2097146, 20=2097024, 19=153,
18=5177, 17=308745, 16=7942, 15=7936, 14=8059 bytes. Select immutable archive 17
(308745 bytes), enough payload for meaningful first timing while well below 2 MiB.
Do not use current or the tiny archives as substitutes. No transfer result yet.

Same gate A, first portion: keep USB console DTR=true/RTS=false, PC and companion on
hotspot, USB powered and media idle. Download archive 17 through USB as reference and
retain CRC-success capture. Enter download mode, wait ACTIVE, use reported address for
one laptop curl GET /f/00000017 with headers and body saved separately, 120 s client cap,
status/size/time summary. Capture mode status and status immediately afterward, exit mode,
confirm OFF, download current.log over USB for HTTP_GET_END/HTTP_GET_MEM, then log status.
Send console/curl output, reference archive, HTTP body/headers and current log. Review
http_margin first, then dual-CRC and byte equality before issuing the Safari save/export
portion of this same selected-file case. No build/flash or further implementation.
Gate A is not complete until exported Safari bytes are compared; increment 5 unapproved.


## September 22, 14:16 - increment 4 gate A preflight; archive selection pending

JP supplied the post-build console directly. Boot 110, WiFi/MQTT connected, logger ready,
hooks=0, PSRAM writer placement valid. Drops=0, queue high=7, slow=0, error=none;
writer margin 3224, internal_min=90768, reported internal_largest=51188; probe
largest_min=47092, above 20480. Current size 964476, generation 22, newest archive 21,
eight archives plus current. No HTTP transfer or HTTP stack measurement yet.

At 14:16:26.355 console says Listed 9 managed files but omits filenames and byte sizes.
The preceding active=1 is sampled during the list command, not evidence of a stuck
reservation. Request the existing web console Files table (screenshot or copied text),
not a repeat build/list/transfer. Select one small immutable archive only after its size
is known. Timing gate A remains pending; increment 5 unapproved.

## September 22, 11:57-11:59 - incomplete-header cancellation PASSED

Evidence: b384075b-ab28-41be-bf09-e97da29fb3d3/Pasted text.txt and PowerShell output.
Partial header sent at 11:58:50.468; log mode off TX at 11:58:51.954, STOPPING RX
11:58:51.955. Client Read=0 at elapsed_ms=1517, before the five-second header deadline.
Off follows the send by 1.486 seconds; EOF follows at roughly the same point (about
30 ms after off from host timestamps). That approximation is not an instrumented
firmware stop latency or a general bound: the stopwatch starts before the write and
console timestamps reflect host observation. The sequence supports cancellation of
an incomplete-header connection rather than the natural five-second timeout.

Generation 7, accepted=1/rejected=0, OFF/server=off confirmed at 11:59:12.469,
error=none/release_stuck=0. Same boot 108, WiFi/MQTT connected, logger ready,
drops=0/high=7/slow=0, USB inactive/unpaused, queue=0/16. Current grows to 241595.
Worker/HTTP margins 2304/1688, writer 2920, internal_min=81136 and largest=47092,
above gate. No reset or new link loss. This completes the cancellation gate; the
previous successful re-entry evidence is retained separately, not claimed to occur
following generation 7 in this capture.

PowerShell executed the intended block: timestamp plus Read=0/1517 confirms execution;
continuation prompts in the pasted transcript are not themselves errors. No practice
command or repeat needed. No firmware changes/build/flash. Startup-failure rollback
coverage is the next item to resolve before increment 3 acceptance; no new hardware
procedure issued in this response. Increment 4 remains unapproved.


## September 22, 11:51-11:54 - incomplete-header cancellation INCONCLUSIVE

Evidence: 484cbc10-63f7-413e-b4a7-ec6f117c13db/Pasted text.txt and JP's PowerShell
output. JP reports normal operation. Boot 108, generation 5 accepts one connection.
However, log mode off TX at 11:52:48.885 precedes the PowerShell partial-header send
at 11:52:56.762 by 7.877 seconds. Client Read=0 after 14 ms is EOF on that connection,
not evidence of cancellation while an incomplete header is pending or a stop-time bound.
Second off at 11:52:59.264 reports already_off. Connection creation time was not captured;
do not attribute the closure specifically to a header timeout.

The original helper connected before its readiness prompt, and the manual sequence was
ambiguous. Correct the helper: readiness prompt before connect, immediately connect/send,
then explicit SEND LOG MODE OFF NOW cue and timestamp. JP must send off only after that
cue and within two seconds, before the five-second header deadline. Repeat only this
cancellation check with final OFF/status; successful re-entry evidence is already present.

Retained successful evidence: generation 6 starts and accepts two connections, zero
rejections; subsequent OFF/server=off, error=none/release_stuck=0. Same boot throughout,
logger ready, drops=0/high=7/slow=0, queue=0/16, USB active=0/paused=0. Current grows
231103 -> 236691. Worker/HTTP minima 2304/1688, writer 2920; internal_min=81136,
final internal_largest=47092 above 20480. No new link loss or reset. No firmware defect
demonstrated, no code change/build/flash. Cancellation gate remains pending, increment 4
unapproved.


## September 22, 11:44-11:49 - increment 3 repeated entry/exit PASSED

Evidence: b895e915-f629-43b1-ba71-23b32ba82fa5/Pasted text.txt; JP reports test passed.
Same boot 108 throughout. Three entries advance HTTP generation 2, 3, 4; counters reset
on entry and each later shows accepted=1/rejected=0, error=none, release_stuck=0.
Generations 2 and 4 have explicit OFF/server=off observations. Generation 3's OFF query
was skipped; successful generation 4 entry confirms it reached OFF because entryRefusal
rejects any other mode and OFF follows completed teardown. Do not claim a directly
observed generation 3 OFF sample or exact stop latency.

Reported internal_largest remains 49140 bytes at all status samples; probe largest_min
bottoms at 47092, above the 20480 gate. Internal low-water minimum moves 88804 -> 87524
-> 87508; these historical minima do not measure retained free memory or prove a leak.
Worker/HTTP stack minima remain 2304/1688; writer minimum remains 2920. Logger stays
ready with drops=0, high=7, slow=0; WiFi/MQTT connected at status samples. Final USB
active=0/paused=0, queue=0/16. Current grows from 222001 to 228923 bytes. No reset,
new USB link loss or stuck release in the capture. This passes the bounded three-cycle
reuse/resource gate, not a long-duration leak proof. No firmware changes or rebuild.

Next case: cancel during incomplete HTTP headers using a single laptop TCP connection
on the hotspot. Send request line and Host line but no terminating blank line; timestamp
send/peer close, with an eight-second client read timeout. JP issues log mode off within
two seconds of sending, before the server's five-second header deadline. Capture console
and client output; a late command/timeout is inconclusive for cancellation, not a pass.
Then confirm OFF, re-enter and reload Safari once to prove reuse, finally stop and capture
status/log status. Increment 4 remains unapproved; startup-failure rollback remains separate.


## September 22, 11:31 - increment 3 first server gate PASSED (boot 108)

Final confirmation: JP confirms the iPhone listing matches the USB file list and the
Last result page displays "No HTTP transfer yet". Together with the recorded startup,
admission, favicon 204, manual OFF/server=off, memory, zero drops, USB CRC and append
growth, this completes the issued normal server start/list/favicon/stop gate. No further
capture, screenshot, downloaded file or measurement is required for this case. USB CRC
is evidenced by the console, not an independent file inspection. Earlier pending notes
below describe the sequence and are superseded by this confirmation. This passes the
first gate only, not all increment 3 validation; increment 4 remains unapproved.

Reviewed checkpoint 767cbbc; JP reports the test passed. Evidence: console attachment
265f1cd4-4897-4190-9b44-89cca49a461e/Pasted text.txt and pasted laptop curl output.
Boot 108, same profile amoled-1-8-core-3-3-11, normal diagnostic switches.

- STARTING at 11:31:31.764, ACTIVE at 11:31:31.768. Worker PSRAM placement and internal
  TCB confirmed; worker margin 2656 bytes, HTTP margin 1688 bytes after requests.
- Admission reaches accepted=2, then 3; rejected=0, last_reject=none, error=none.
  This hardware run verifies the corrected dual-stack admission on this connection path.
- Laptop favicon GET returns HTTP/1.1 204 No Content, Content-Length: 0,
  Connection: close, Cache-Control: no-store and Referrer-Policy: no-referrer.
- Logger ready, drops=0, high=7, slow=0; internal_min=88804, reported internal_largest=49140.
  Probe largest_min reaches 47092, above the 20480-byte gate. No reset in this capture.
- USB current transfer succeeds while mode remains ACTIVE: 210225 bytes, 2.13 seconds,
  CRC OK. Later current_size=210388, confirming 163 bytes of append growth;
  active=0, paused=0, queue=0/16, result=ok. One 5 ms USB link loss recovered within grace.

The requested manual stop was not executed: at 11:33:20.347 the command was literally
`Run log mode off`, rejected as unknown. At 11:33:31 mode was still ACTIVE. No OFF/server=off
observation follows. This is a procedure correction, not a demonstrated shutdown defect.
JP's message ends with "Then I got this in the web UI:" but the UI detail is missing;
listing-name comparison and Last result content cannot yet be independently marked complete.
The downloaded file itself was not supplied in this turn; CRC evidence is the console report.

Continue only the outstanding portion of this same case, without rebuilding: capture
log mode status; if already OFF through idle timeout, enter mode again; issue exactly
log mode off, wait two seconds, then log mode status, status and log status. Request the
missing web UI detail and whether listing/Last result matched expectations. No repeat
USB download needed on this evidence. Increment 3 gate remains incomplete; increment 4
unapproved. No firmware changes or assistant build/flash.


### Same-case manual shutdown completion - 11:37, boot 108

JP supplied the follow-up console directly. Exact log mode off at 11:37:12.920
produces STOPPING/reason=usb_command at 11:37:12.923. The next status at 11:37:22.391
confirms OFF/server=off, error=none, release_stuck=0, same generation 1 and accepted=3,
rejected=0. Worker minimum after teardown is 2304 bytes; HTTP minimum remains 1688.
The 9.47-second observation interval does not measure actual teardown duration and must
not be reported as stop latency or as proof of a tighter timing bound.
At 11:37:27 logger remains ready on boot 108, current_size=213763 (continued growth),
drops=0, queue=0/16, USB active=0/paused=0/result=ok. Probe largest_min=47092 remains
above the memory gate. Manual shutdown portion passes; no repeat hardware action needed.
Only the missing Safari listing-name comparison and Last result observation remain for
this first gate. Increment 4 remains unapproved. No code changes.

Running bench record for the wireless retrieval feature, one case at a time.
Design: [Codex review](sd_iphone_log_download_review.md) and
[Claude review](sd_iphone_log_download_review_claude.md). Historical draft:
[sd_iphone_log_download_plan.md](sd_iphone_log_download_plan.md).
Branch `iphone-log-retrieval`. Results are appended newest first as cases complete.

Status: design review closed on September 20, 2026 with six corrections accepted.
**Case 1 passed on September 20, 2026**, with transfer timing deliberately deferred. No
wireless firmware exists for this feature yet. The USB-only extraction is now implemented
and reviewed by Claude at `0ba72e5`; its host checks and issued first bench gate are recorded in the
[increment 1 handoff](sd_iphone_log_download_increment1.md). All five issued USB regression gates and the restored normal-build handoff passed on
September 21. JP accepted increment 1 and approved increment 2 on September 21. Increment 2 is
reviewed by Claude; its first hardware entry/exclusion/exit gate passes below. Historical acceptance-pending statements below predate that decision.

---

## Increment 3 first server case - September 22, 11:13-11:18 - INCOMPLETE / HTTP RESET

Evidence: attachment `0851f9d1-399c-4c56-8385-630ab4d31280/Pasted text.txt`, boot 106,
plus JP's untimestamped verbose curl transcript in the conversation. Curl established
TCP to 172.20.10.2:80, sent GET /favicon.ico, then received reset before response headers.
The curl transcript cannot be aligned precisely to individual mode-status timestamps.
No iPhone listing result or completed HTTP 204 is evidenced; the whole gate is not passed.

At 11:13:30 mode goes STARTING -> ACTIVE in 4 ms, worker_external=1, tcb_internal=1,
worker_min=2656. At 11:17:53 mode remains ACTIVE, idle_ms=262655, error=none,
http_min=0. At 11:18:36 it is OFF/idle_timeout/server=off, worker_min=2304. Same boot,
no reset in the capture. Thus first httpd_start and stop on the PSRAM worker succeeded;
there is no reason from this result to invoke the internal-stack fallback. This is not
proof of all PSRAM/lwIP operations. HTTP minimum staying zero supports failure before a
successfully admitted session; it is not a recorded rejection reason. Probe minima shown
remain above the 20480-byte gate. Initial logger status has drops=0 and one historical
slow flush (113459 us); there is no full post-case logger status or current.log download.

### Code defect and proposed correction, awaiting Claude review

Installed sdkconfig enables CONFIG_LWIP_IPV6. IDF 5.5.5 httpd_server_init therefore uses
PF_INET6 for its listener, serving IPv4 through mapped addresses. The implementation's
getsockname buffer was sockaddr_in and admission required AF_INET. This rejects valid
IPv4-mapped local addresses, consistent with the reset and http_min=0. Runtime family was
not captured by the old firmware, so this explains a verified code defect rather than
claiming a directly measured rejection reason for every curl attempt.

Correction uses sockaddr_storage and validates length/family, accepting native IPv4 or
exact ::ffff:IPv4 only when the final IPv4 bytes equal the hotspot STA address. Native
IPv6, non-mapped addresses, truncated addresses and other local addresses stay refused.
Status now retains accepted/rejected counters and last rejection reason per mode entry,
without emitting per-request SD records. No capability or native-IPv6 service is added.
Five new source-body checks cover these cases. No firmware build/flash by Codex.
Claude must review before JP rebuilds. Resume this same first case after review; no
additional case or increment is approved by this failed attempt.

---

## Increment 2 accepted - September 22, 2026

JP explicitly accepted increment 2 after gate 11, including the preceding proposal to
defer battery-only entry refusal and brief pending-handover entry refusal hardware checks
until controlled bench coverage before car deployment. All eleven issued cases passed;
the two deferred cases remain unproven on hardware. No additional test or flash is needed
for this acceptance. Earlier pending-acceptance statements below are historical.

Next: prepare increment 3's lifecycle/descriptor ownership design for Claude review,
along with provisional socket budget and capability decision required by spec section 12.
Increment 3 implementation remains unapproved and requires JP's separate authorization.

---

## Increment 2 gate 11 - 2026-09-22, 08:37-08:41 - PASS

Remote MQTT notification exclusion and still-to-Live recovery, boot 104; JP confirms pass.
Evidence: attachment `17e8481e-f4a5-4863-880e-f1d3f4ee314c/Pasted text.txt`, Downloads
`104-current (1).log`.

ACTIVE at 08:37:52; remote refusal at 08:38:11 is persisted as MQTT_IMAGE
result=ignored_download_mode seq=133, with no IMAGE_BEGIN/LIVE_BEGIN for that attempt.
The intermediate ACTIVE query after refusal was omitted; no exit is recorded until the
explicit off command at 08:38:45, followed by exit completion seq=135 and OFF status.
Fresh notification at 08:39:22 yields IMAGE_BEGIN trigger=mqtt, accepted MQTT_IMAGE,
and IMAGE_END displayed/HTTP 200, expected=received=34207. LIVE_BEGIN seq=146 has
trigger=motion_handover. LIVE_END seq=157 is duration/failure=none/HTTP 200,
**165 frames / 60368 ms = 2.73 fps**. This is functional recovery, not a paired FPS gate.

Download **1827146 bytes**, 16.92 s, browser CRC OK; local length matches and computed
CRC32 **1298A90B**. Post-Live full status: ready, drops=0, error=none, stack_min=2920,
internal_min=38032, internal_largest=26612 > 20480. After download, USB active=0,
paused=0, result=ok, queue=0/16, drops=0, no retained USB failure; current_size=1827313
exceeds the downloaded snapshot by 167 bytes, confirming append continuation. Same boot,
no unexpected reset visible; JP reports the test passed. No firmware changes.

### Acceptance coverage proposal - awaiting JP, not a change to the approved spec

All eleven issued increment 2 cases pass. Section 11 of the spec still calls for battery-only
entry refusal and entry refusal during the brief pending motion handover. Neither has been
observed on hardware. Host source simulations cover usb_power_required and display_pending;
the completed-display pending guard and real VBUS-loss exit have hardware evidence, but
these do not establish the two missing cases.

Recommend accepting increment 2 with those two hardware checks explicitly deferred to a
controlled bench procedure before car deployment, using later entry UI or an approved
bounded fixture if necessary. Do not silently mark them passed or assume a timed manual
command catches the short gap. JP must approve this coverage exception; otherwise design
one targeted case at a time. No further bench case is issued pending that decision.
After acceptance, propose resolving increment 3 descriptor lifetime and lifecycle teardown
ownership with Claude review before JP explicitly authorizes implementation. Increment 3
remains unapproved. Historical draft unchanged.

---

## Increment 2 gate 10 - 2026-09-22, 08:15-08:16 - PASS

History-image Back exclusion/recovery, boot 104; JP confirms pass. Evidence: attachment
`b34e9571-c1bc-4456-abce-1462e7212b7c/Pasted text.txt`, Downloads `104-current.log`.

Mode ACTIVE at 08:15:09.614; Back at 08:15:23 refused download_mode, with persisted
IMAGE_REFUSED trigger=history_back seq=37 and no IMAGE_BEGIN for the refused request.
Status remains ACTIVE. Exit requested 08:15:44.897; saved exit completion seq=42 precedes
allowed IMAGE_BEGIN seq=44 at 08:15:53.402. The explicit OFF query was omitted, but the
saved ordering confirms completed exit. IMAGE_END seq=49: displayed, HTTP 200,
34586 expected/received bytes, total_ms=1362. JP reports normal operation.

Current download **1795870 bytes**, 16.59 s, browser CRC OK; local size matches and
computed CRC32 **12B27089**. Before download: ready, drops=0, error=none, queue=0/16,
internal minimum=43708, largest=31732 > 20480, stack minimum=3208. No post-download
resource status was captured; these are pre-download observations. No unexpected reset
or stall is visible in this capture. This is a new sitting/boot, not a paired comparison
with yesterday. No firmware changes.

### Next single case issued: remote notification exclusion and handover recovery

Same build/no flash, USB and hotspot connected, DTR=true/RTS=false, browser test switches
off, dashboard idle. Clear console, send status then log mode on/status; require ACTIVE.
Use the normal entrance-camera system to send one real MQTT latest-image notification
(not movement of the companion itself). Expect refusal with no still or Live, then mode
status remains ACTIVE. Send log mode off/status; require OFF. Wait at least 15 seconds
after any preceding image display, then trigger a fresh camera notification. Expect the
normal brief still followed by Live; allow the full cycle to finish. Capture status and
log status, download current with CRC OK, then log status again to check unpaused/idle
and drops. Send console, downloaded file and visual observations. If a notification
cannot be triggered or received, report that rather than substituting a local button.

This covers real MQTT admission and successful still-to-Live handover after exit; it does
not prove entry refusal during the short pending-handover gap or battery-only entry.
Those coverage decisions remain open before increment 2 acceptance. Increment 3 remains
unapproved and requires lifecycle/descriptor decisions before implementation.

---

## Increment 2 gate 9 - 2026-09-21, 15:19-15:22 - PASS

Direct Live exclusion while ACTIVE, boot 103; JP confirms pass. Evidence: attachment
`6c60d715-acac-423c-93a6-00c7814708ea/Pasted text.txt`, Downloads
`103-current (9).log`.

ACTIVE at 15:19:26; button at 15:19:40 refused download_mode. Persisted LIVE_REQUEST
seq=338 is refused, with no LIVE_BEGIN for that attempt; subsequent status remains ACTIVE.
Mode exit completes seq=340 before the next LIVE_BEGIN id=4. The explicit OFF query was
omitted, but persisted completion establishes ordering. Allowed Live ends reason=duration,
failure=none, http_code=200, 183 frames/60397 ms = **3.03 fps** (not a paired benchmark).

Download **1224471 bytes**, browser CRC OK, 11.63 s; local length matches, computed
CRC32 **56240213**. Pre-download status ready, drops=0, error=none, queue=0/16,
internal minimum=34304 and largest=24564 > 20480, stack minimum=2920. Largest retained
block is lower than the prior 26612 observation; no leak or cause is inferred from these
boot-retained minima. No unexpected reset/stall visible. Same boot, normal flags.

### Next single case issued: history-image Back exclusion and recovery

Same build/no flash, dashboard, USB/hotspot connected, browser test switches off. Mode
on/status -> ACTIVE. Press the dashboard history-image Back button (not screen-return
navigation) once: expect Back refused: download mode, no image/loading screen. Mode
status stays ACTIVE. Mode off/status -> OFF. Press the same history-image Back button
once; expect normal older-image retrieval. After it displays, return to dashboard via
normal navigation; status/log status, then current download (CRC OK). Send console/file
and screen observations. If the server has no older image, report that result rather
than counting successful rendering. This covers the remaining local still caller;
remote MQTT/handover and battery-entry evidence still need an explicit coverage decision
before increment 2 acceptance. Increment 3 unapproved; no new firmware changes.

---

## Increment 2 gate 8 - 2026-09-21, 15:16-15:18 - PASS

Explicit mode exit during current USB transfer, boot 103; JP confirms pass. Evidence:
attachment `a71bd6ce-3153-4649-9902-a05ad1a5be00/Pasted text.txt`, Downloads
`103-current (8).log`. Earlier OFF status in this capture preceded the omitted on command;
JP then sent on successfully at 15:16:14.930. No defect was observed at initial setup.

Current transfer started 15:16:30.875; off sent 15:16:32.853, followed by STOPPING and
Device: aborted. Persisted order: mode stopping seq=322 up_ms=3070134; USB_GET_END
seq=323 up_ms=3070138, bytes=206208, duration_ms=1990, result=aborted; mode exit ok
seq=324 up_ms=3070158. This supports cleanup before completed mode exit; record spacing
is not a separately instrumented release/close latency measurement.

Subsequent mode status OFF/release_stuck=0, USB active=0/paused=0/result=aborted,
queue=0/16, drops=0, no retained failure. Retry **1217047 bytes**, browser CRC OK,
11.45 s; local size verified, calculated CRC32 **AFD46D5A**. Same boot. The only full
resource status is before the test (largest=26612, stack_min=2920); no post-retry memory
or status observation is claimed. Retry file preserves cancellation evidence and
records appended afterward. No firmware changes.

### Next single case issued: direct Live refusal while mode ACTIVE

Same build/no flash, USB/hotspot connected, dashboard idle, test switches off. Clear
console; mode on/status must be ACTIVE. Press Live once: expect Live refused: download
mode, no loading screen/frames/navigation. Send mode status, still ACTIVE. Mode off/status
must reach OFF. Press Live once and allow the full normal cycle to complete. Then
status/log status and current download (CRC OK). Send console/file and whether the first
press stayed on dashboard and the second played normally. Earlier live_busy gate tested
the opposite admission direction; this covers the direct Live guard while mode is active.
Remaining coverage is to be reviewed before increment 2 acceptance; increment 3 unapproved.

---

## Increment 2 gate 7 - 2026-09-21, 15:09-15:11 - PASS

USB-power-loss exit without reboot, boot 103; JP confirms pass. Evidence: attachment
`515d0129-20ce-43e5-b398-2ad7932114ae/Pasted text.txt`, Downloads
`103-current (7).log`.

ACTIVE confirmed before unplug. Host device removal 15:10:15.546, available again
15:10:23.350. Persisted boot-103 mode exit usb_power_lost/stopping at up_ms=2692876
and ok at 2692880. Post-reconnect status confirms OFF/reason=usb_power_lost,
release_stuck=0; same boot 103/continuous uptime. No explicit off command or reboot
explains this exit. Record spacing is not a physical VBUS-to-exit latency measurement.
Movement occurred while handling the board; power loss is the recorded exit cause.

Download **1209578 bytes, browser CRC OK**, 11.29 s; local length matches, calculated
CRC32 **54FDFD6B**. Pre-download status ready, drops=0, error=none, queue=0/16,
internal largest=26612 > 20480, stack minimum=2920, continued append growth. Initial
USB active=1 on reconnect belongs to automatic listing; subsequent status is idle.
No unexpected reset/stall visible. This is idle-mode VBUS exit, not battery-entry
refusal or power loss during a transfer. No firmware changes.

### Next single case issued: explicit mode exit during current USB retrieval

Same build/no flash, USB/hotspot connected, DTR=true/RTS=false, browser test switches off.
Capture status and mode on/status (ACTIVE). Pretype log mode off in the command box but
do not send yet. Download current.log; while progress is advancing (about two seconds
into the current roughly 11-second transfer), send that command. Expect STOPPING,
Device: aborted, no saved partial file. Send mode status and log status after the abort:
require OFF, release_stuck=0, active=0, paused=0, result=aborted. Stop on mismatch.
Download current normally without re-entering mode (CRC OK); status/log status. Send
console and successful file. If download finished before off arrived, report that timing;
it did not exercise cancellation. This validates main exit -> writer cleanup/release ->
OFF and subsequent reuse. Other admission/exclusion coverage remains for final review;
increment 3 unapproved.

---

## Increment 2 gate 6 - 2026-09-21, 15:04-15:07 - PASS

Wi-Fi loss/recovery retains mode and timer, boot 103. JP confirms pass. Evidence:
attachment `e7de58ac-b669-4db7-9842-c4d7fbc826e1/Pasted text.txt`, Downloads
`103-current (6).log`.

ACTIVE entry 15:04:45.934; offline status confirms Wi-Fi OFFLINE/MQTT DISCONNECTED
and mode ACTIVE/link=down/idle_ms=41560. Persisted RETRIEVAL_LINK down at up_ms=2386751,
up at 2420597: **33846 ms** observed link-down interval, not a reconnect latency measured
from hotspot re-enable. Wi-Fi/MQTT recover; mode ACTIVE/link=up/idle_ms=70408, consistent
with original entry (no reset). Explicit off completes OFF with release_stuck=0.

Download **1203556 bytes**, browser CRC OK, 11.39 s; local size verified, computed
CRC32 **08231D6E**. Last pre-download status ready, drops=0, error=none, queue=0/16,
internal largest=26612 > 20480, stack minimum=2920, no USB losses/retained failure;
same boot, ongoing append growth. This tests mode/link policy, not HTTP listener recovery
(no server exists) or HTTPS readiness. No firmware changes.

### Next single case issued: observed USB power loss exits mode

Same build, no flash, battery connected/charged and hotspot available. No media/downloads
in progress. Explicit DTR=true/RTS=false and all browser switches off. Clear console;
send mode on/status and confirm ACTIVE, then status/log status to record boot. Physically
unplug USB for about five seconds, then reconnect promptly (before the existing 30-second
power-loss grace). Do not send mode off. Reconnect Chrome, preserving console output,
and send mode status: expect OFF/reason=usb_power_lost/release_stuck=0. Send status/log
status, then download current once with CRC OK. Send console/file and whether board stayed
on without reboot. If boot changed, report it; OFF after a restart alone cannot prove
power-triggered mode exit. Use persisted RETRIEVAL_MODE exit evidence to evaluate the gap.
This is idle-mode power loss, not transfer interruption or battery-only entry admission.
Increment 2 acceptance remains pending; increment 3 unapproved.

---

## Increment 2 gate 5 - 2026-09-21, 14:52-15:01 - PASS

Panel-touch idle reset, boot 103; JP confirms pass. Evidence: attachment
`4456aa00-915b-4ccb-8e5b-46c8ec8ea528/Pasted text.txt`, Downloads
`103-current (5).log`.

Entry up_ms=1657076 (14:52:59.836). Touch is visible at host 14:54:59.687; mode status
at 14:55:04.092 shows ACTIVE/idle_ms=4417, proving reset. Persisted idle exit at
up_ms=2076925 (14:59:59.685) is **419849 ms after entry**, approximately five minutes
after touch. The original-deadline status query was omitted; the persisted later exit
plus reset observation establish the intended behavior without a repeat. Host touch and
device record timestamps are not an exact same-clock latency measurement. Movement
occurred shortly after entry without exiting mode; no extra panel touch is visible.

Final mode OFF/idle_timeout/release_stuck=0. Download **1190003 bytes**, browser CRC OK,
11.26 s; local length matches and calculated CRC32 **BA9A10A9**. Pre-download status
ready, zero drops/errors, queue=0/16, largest block=26612 > 20480, stack minimum=2920,
no USB loss/retained failure. Same boot, Wi-Fi/MQTT connected; appends continue.

### Next single case issued: Wi-Fi loss/recovery retains mode and idle age

No flash; same build, USB connected, dashboard idle, no board touches/media/downloads.
Use the actual hotspot/AP currently serving the companion; serial off is MQTT-only and
cannot test Wi-Fi loss. Clear console, mode on -> ACTIVE, mode status capture. Disable
that hotspot for about 30 seconds; send status and mode status, expecting WiFi disconnected
and ACTIVE/link=down. If Wi-Fi remains connected (for example another configured network),
stop and report that instead of claiming a loss. Re-enable hotspot, wait for status to
confirm Wi-Fi/MQTT recovery, then mode status: ACTIVE/link=up, idle_ms continuing forward
rather than near zero. Complete within five minutes of entry so idle expiry is not a
confound; report delay/expiry rather than re-entering silently. Mode off/status -> OFF;
status/log status and one current download with CRC OK. Send console/file and note any
touch or power interruption. No iPhone wireless download/server exists at this increment.
Other power/admission gates remain pending; increment 3 unapproved.

---

## Increment 2 gate 4 - 2026-09-21, 14:42-14:49 - PASS

Five-minute idle expiry, boot 103; JP confirms pass. Evidence: attachment
`5b0a46a7-973b-4b61-8a7a-da2f2a5d6ac5/Pasted text.txt`, Downloads
`103-current (4).log`.

Entry seq=149 up_ms=1053110. Intermediate mode status is ACTIVE/idle_ms=121654.
Persisted idle_timeout stopping and ok records seq=165/166 are both at up_ms=1353119:
**300009 ms after entry**. Thus the status request did not reset idle and exit occurred
independently before the final status query, which confirms OFF/release_stuck=0.

Download **1175630 bytes, browser CRC OK**, 11.06 s; local length matches, calculated
CRC32 **63123CC4**. Pre-download status ready, drops=0, error=none, queue=0/16;
internal largest=26612 > 20480, writer stack minimum=2920, no USB losses or retained
failure, same boot and Wi-Fi/MQTT connected. No screen touch visible during the interval.
No post-download status supplied; no stronger resource claim made. This is idle with no
transfer; future HTTP archive-mid-transfer expiry remains a separate later gate.

### Next single case issued: panel touch resets idle deadline

Same build, no flash; USB/hotspot connected, dashboard idle, all browser switches off.
Clear console, send mode on and confirm ACTIVE. At about two minutes after entry, tap
once on an unused dashboard area (not a media/navigation button), then immediately send
mode status: require ACTIVE and idle_ms near zero. Do not touch again. At five minutes
ten seconds from original entry, send mode status: require still ACTIVE, idle_ms about
190000. At five minutes ten seconds from the touch, send mode status: require OFF,
reason=idle_timeout, release_stuck=0. Use one clock/timer and note both entry/touch times.
If the touch does not reset idle, stop and send the capture. After expected expiry, send
status/log status and download current (CRC OK). Send console/file and any extra touch
or connection interruption. This is one touch-reset case; other link/power/admission
gates remain pending. Increment 3 unapproved.

---

## Increment 2 gate 3 - 2026-09-21, 14:37-14:39 - PASS

Active-Live entry refusal, boot 103. JP confirms pass. Evidence: attachment
`64484ff4-6ea6-4ef4-9541-e79a146d7cea/Pasted text.txt` and Downloads
`103-current (3).log`.

Live started 14:37:24; mode on at 14:37:30.561 refused live_busy, state OFF.
Live continued to duration completion: persisted LIVE_END failure=none, http_code=200,
173 frames / 60197 ms = **2.87 fps**, no performance comparison claimed. Mode entry
succeeded after completion; off then status confirmed OFF, release_stuck=0. Persisted
mode events confirm refusal, later entry and exit completion.

Download **1164408 bytes**, browser CRC OK, 10.96 s; local length verified, computed
CRC32 **11A657C5**. Pre-download status ready, drops=0, error=none, queue=0/16,
internal largest=26612 > 20480, stack minimum=2920. Same boot, no unexpected reset/stall
visible. Last resource status precedes retrieval. This proves Live-active entry refusal,
not a paired FPS benchmark or direct Live refusal while mode ACTIVE.

### Next single case issued: five-minute idle exit

Same build, no flash. Keep USB and hotspot connected, dashboard idle, no media/downloads,
no panel touch, browser test switches off. Send mode off/status, clear console, then mode
on and confirm ACTIVE. Start timing from that successful on. At about two minutes send
only log mode status: expect ACTIVE and idle_ms near 120000 (status must not reset it).
At five minutes ten seconds from entry send mode status: expect OFF/reason=idle_timeout,
release_stuck=0. Do not send mode off before observing that result. Then status/log status
and one current download (CRC OK); send console/file and report any panel touch or USB/
hotspot interruption. Persisted timestamps, not manual timing alone, determine expiry.
This tests idle expiry without touch; touch reset and link/power cases remain separate.
Increment 2 acceptance pending, increment 3 unapproved.

---

## Increment 2 gate 2 - 2026-09-21, 14:32-14:34 - PASS

Pending-display admission, boot 103. JP confirms the test passed, including the screen
behavior. Evidence: attachment `b9e78bbd-c14b-42aa-801f-55e98c6f1381/Pasted text.txt`
and `C:/Users/photo/Downloads/103-current (2).log`.

Latest completes at 14:32:56.699 (31713 image bytes, 1492 ms). On at 14:33:00.243 is
OFF/refused/display_pending, before screen unload at 14:33:10.268. After navigation,
on at 14:33:27.579 succeeds ACTIVE/ok. Off reports STOPPING; persisted seq=95 confirms
exit result=ok at up_ms=496900 even though final mode status was omitted. No stuck-release
record is needed for this ordinary exit. The saved log contains the refusal and re-entry.

Current download **1155516 bytes, browser CRC OK**, 10.89 s; local file size matches,
computed CRC32 **9FA07F97**. Last pre-download full status: ready, drops=0, error=none,
queue=0/16, internal largest=31732 > 20480, writer stack minimum=2920, no USB loss or
retained failure. Same boot, Wi-Fi/MQTT connected; no unexpected reset/stall visible.
No post-download status was supplied, so those resource observations precede retrieval.
This establishes completed-image display admission, not the brief motion handover gap.

### Next single case issued: refuse entry during Live

Same build, no flash. Dashboard, Wi-Fi/MQTT connected, DTR=true/RTS=false, browser test
switches off. Send mode off/status and confirm OFF. Start Live normally. While frames
are visibly updating (about five seconds in), send log mode on; expect OFF/refused/live_busy,
and Live should continue normally. Let the full cycle finish; return to dashboard if
needed. Send mode on (ACTIVE/ok), off, then mode status (OFF). Send status/log status,
download current once with CRC OK, save console. Send console/file and whether Live
continued normally. If command arrives after Live finishes, report the timing; that does
not exercise this refusal gate. Other increment 2 gates remain pending; increment 3 is
unapproved. No firmware change or assistant build/flash in this review.

---

## Increment 2 gate 1 - 2026-09-21, 14:25-14:29 - PASS

Entry/exclusion/exit with reordered steps, boot 103. JP reports the test worked, with
Latest-in-mode tested last. Source checkpoint `5ed5ed1` includes Claude's approval of
corrections `3b9d1cc`. Evidence: attachment
`1a8641d2-e734-4b53-aab7-a12d76035ede/Pasted text.txt` and Downloads
`103-current.log` / `103-current (1).log`.

- Initially OFF; on -> ACTIVE/ok at 14:26:20; repeated on refused not_off. Off ->
  STOPPING, then OFF confirmed; repeated off reports already_off.
- Latest with mode OFF succeeded: IMAGE_BEGIN id=1, IMAGE_END ok/displayed, 31713 image
  bytes, total_ms=1365 in persisted record. UI_ACTION processed belongs to that request.
- Second on -> ACTIVE at 14:28:52; Latest at 14:28:56 printed Image refused: download
  mode. Persisted seq=63 is IMAGE_REFUSED trigger=latest reason=download_mode, with no
  IMAGE_BEGIN or UI_ACTION processed for that press. Thus "worked" means exclusion
  worked, not that a new image was admitted while ACTIVE. No firmware correction needed.
- Both exits have persisted stopping then ok records. Second exit completes at up_ms
  237138; no RETRIEVAL_STUCK appears. Record spacing is not a measured close deadline.
- USB downloads: 1145827 and **1148832 bytes**, both browser CRC OK. Second file size
  verified locally, calculated CRC32 **BA86BA29**; it includes the final refusal/exit.
  Two complete downloads plus size growth establish ongoing logging in this interval.
- Last full status (before final refusal/download) is ready, hooks=0, drops=0, error=none,
  queue=0/16, largest internal block=31732 > 20480, stack minimum=3208. No final full
  status after the second download was supplied; no stronger final-memory claim is made.
  Same boot throughout, no unexpected reset/stall visible. Normal fixture lines absent.

### Next single case issued: refuse entry during completed-image display

Same build, no flash. DTR=true/RTS=false; all browser switches off. On dashboard send
log mode off, log mode status (OFF), status. Press Latest and wait for the image to finish
loading. While that image remains displayed (within its normal one-minute display window),
send log mode on. Expect OFF/refused/display_pending, with the image still displayed.
If image_busy appears, wait for completion and retry while still on the image; if it already
returned to the dashboard, report that timing rather than treating entry as a failure.
Use the normal navigation control to return to dashboard (not the history-image Back
request). Send log mode on: expect ACTIVE/ok, then off and mode status: OFF. Capture
status/log status, download current with CRC OK, send console/file and screen observations.
This tests the pending-display admission guard and its release on navigation, not remote
motion handover or active Live. Those remain separate cases; increment 3 unapproved.

---

## Increment 1 normal-build handoff - 2026-09-21, 10:37-10:39 - PASS

JP reports the test ran fine. Evidence: attachment
`e7d8aa31-31c1-4ddf-9802-fe572e2382d7/Pasted text.txt` and
`C:/Users/photo/Downloads/101-current.log`.

- Boot 101, hooks=0; fixture, USB TEST and USB GATE status lines absent. Source flags
  confirmed: logging=1, fixture=0, hooks=0, PSRAM writer=1; working tree clean.
- **103377 bytes, browser CRC OK**, 1.04 s. Local file length matches and computed
  CRC32 is **7F7FCDC7**. Numeric CRC is local; browser CRC OK supplies device-END
  verification. Current size later reaches **104680** (+1303 beyond snapshot), same
  generation 21/newest 20, no rotation. Writes 35 -> 41.
- Final ready, active=0, paused=0, result=ok, queue=0/16, drops=0, error=none;
  no retained transfer failure, USB losses or unexpected reset in capture. Wi-Fi/MQTT
  remain connected. Internal largest block **51188** exceeds 20480; internal minimum
  92004; writer stack minimum **2920**, matching the earlier normal-build cases.

**Increment 1 is ready for JP's explicit acceptance.** Five scoped regression cases
(normal current, cancellation/reuse, queue protection, selected-archive prune, orderly
close/restart) plus this normal-build restoration check pass. Host checks and Claude
review remain as recorded; no new code changes require their repetition. In-flight
shutdown timing and delayed stale acknowledgements were not measured on hardware;
the prior documented coverage limits and review deferrals remain. No additional bench
case is requested now. No increment 2 implementation or approval is implied.

Next action: JP may accept increment 1 and explicitly approve increment 2. Proposed
increment 2 is mode state/admission with USB command entry and no server, per spec;
implementation goes to Claude review before JP builds/flashes, with bench gates issued
one at a time. Historical draft remains untouched.

---

## Increment 1 USB gate 5 - 2026-09-21, 10:30-10:33 - PASS

Orderly shutdown and restart, boots 98 -> 99, fixture-enabled build. JP reports the test
ran fine. Evidence: attachment `281557af-8003-4e4e-b338-fbd7ae3d18b0/Pasted text.txt`,
`C:/Users/photo/Downloads/98-current.log` and `99-current.log`.

- Before: 78888 bytes, browser CRC OK, local CRC32 **6AF52D0E**. After: 86435 bytes,
  browser CRC OK, local CRC32 **FF9C81B8**. The first **78888 bytes match exactly**.
- USB removal at host 10:31:06.494 is expected for this test. Persisted boot 98 records:
  POWER_DECISION action=shutdown moving=0 usb=0 idle_ms=88833 at 10:31:37.471;
  SESSION_END reason=shutdown pending=0 at 10:31:37.531. Boot 99 records reset=power_on,
  context=append. This establishes orderly close and append preservation, not an exact
  diagnosticsClose latency: clocks and record intervals are not a caller-wait measurement.
- Post-restart CRC transfer succeeds; later current_size=87730 (1295 beyond snapshot).
  Same file generation 21/newest 20, archives=7; no rotation. Final ready, active=0,
  paused=0, result=ok, queue=0/16, drops=0, error=none, no USB losses or retained failure.
  Wi-Fi/MQTT connected. Internal largest block 51188 > 20480, internal minimum 94412,
  writer stack minimum 2984. Startup sd_max_us=159198 is reported, not treated as a
  transfer stall; slow write count=0.

All five issued increment 1 bench cases now pass. This close case is idle shutdown,
not in-flight close or deep-sleep repetition; host tests cover session shutdown and
previous accepted platform evidence remains applicable. Full acceptance belongs to JP.

### Next single case issued: restore normal build and verify configuration

Codex restored only the temporary fixture define from 1 to its tracked default 0;
DIAG_ENABLED=1, DIAG_TEST_HOOKS=0 and DIAG_WRITER_STACK_PSRAM=1 remain unchanged.
JP builds/flashes amoled-1-8-core-3-3-11 (no generated-sketch deletion). Connect Chrome
with explicit DTR=true/RTS=false and all test switches off. Send status/log status,
download current once (CRC OK), wait 70 seconds, then send both status commands again.
Send saved console and current.log. Verify hooks=0, fixture/test/gate status lines absent,
ready, no drops/errors, USB idle/unpaused, CRC success and append growth. This is the
normal-configuration handoff check, not a repeat of the fixture suite. After reviewing it,
present increment 1 for explicit acceptance and ask for increment 2 approval separately
or together. No increment 2 implementation has begun.

---

## Increment 1 USB gate 4 - 2026-09-21, 09:40-09:43 - PASS

Selected synthetic archive pruning, same boot 97/fixture-enabled build. JP reports normal
operation. Evidence: attachment `3496d409-128d-4ea3-9f04-89cfa38a2bb2/Pasted text.txt`
and `C:/Users/photo/Downloads/97-current (1).log`.

- Fixture creation completed with archive=00000022, bytes=2097152, result=ok. Two
  earlier list requests returned unavailable while creation was active; this is the
  documented guard, not a logger failure. After creation, list count was 9/newest=22.
- `log test prune 22` armed and fired once; result=pruned, outcome=pruned. Device
  returned pruned. Saved current records show archive-00000022.log completion with
  bytes=1584, duration_ms=61, result=pruned, followed by USB_PRUNE_TEST synthetic=true.
- After removal: count=8, newest=20, archives=7, pruned=1 (was 0), fixture result=
  pruned_by_test. These counts return to the pre-fixture inventory; the capture does
  not include individual FILE lines for a byte-by-byte inventory comparison.
- USB active=1 in the refresh snapshots corresponds to the concurrent listing. Later
  standalone status confirms active=0, paused=0, result=pruned before the retry.
- Normal current retry: **67600 bytes, CRC OK**, 0.68 s. Local size matches, computed
  CRC32 **F33D63CB**. Browser CRC OK verifies device-END comparison; numeric CRC is local.
  Later size=68902; ready, active=0, paused=0, result=ok, zero drops/errors, same boot,
  no new stall/reset, no USB link losses. Largest block=51188, stack minimum=2984.

This passes the production removal-helper regression for an actively read disposable
archive; it does not repeat retention threshold selection. Close coverage remains pending.
No firmware edit/build/flash by Codex. JP's fixture=1 change stays local and uncommitted.

### Next single case issued: orderly shutdown close and restart

Same build, no flash. Keep hotspot available and Live stopped. Save status/log status
and a fresh CRC-checked current.log as the pre-shutdown reference. Unplug USB from the
battery-equipped unit, leave stationary and untouched, and allow up to two minutes for
automatic Shutdown/screen-off. Do not force it with the power button; report a failure
to shut down. After power-off, reconnect USB and Chrome with explicit DTR=true/RTS=false.
Retain both console segments. Send status/log status, download current (CRC OK), wait
70 seconds, then send both status commands again. Send both saved files and the console,
plus observed shutdown behavior. Compare prefix preservation, prior SESSION_END
reason=shutdown pending=0, new boot append, later growth and zero drops/errors.
This is orderly idle shutdown/restart, not a measurement of an in-flight network or USB
cancellation deadline. Existing host tests separately cover shutdown during a session.
After reviewing close evidence, restore the normal fixture=0 build before final handoff;
no increment 2 work without explicit approval.

---

## Increment 1 USB gate 3 - 2026-09-21, 09:37-09:39 - PASS

Queue-pressure abort and retry, boot 97, fixture=1/hooks=0/PSRAM writer=1.
JP reports the test ran fine. Evidence: attachment
`7450c206-1d9e-47d3-b307-6c15820bc770/Pasted text.txt` and
`C:/Users/photo/Downloads/97-current.log`.

- `log test queue` armed; transfer fired once, added=8, queued_at_test=8/16,
  result=injected. Device returned logger_busy, browser reported the expected error.
- Saved retry contains all **eight USB_QUEUE_TEST records**, boot 97 seq=38..45,
  followed by seq=46 `USB_GET_END bytes=1584 duration_ms=54 result=logger_busy`.
  Thus the actual queued evidence was saved after abort, not merely reported injected.
- Retry without rearming: **61227 bytes, CRC OK**, 0.61 s. Local file size matches;
  independently calculated CRC32 **23E3FFE5**. Numeric CRC is local; browser CRC OK
  establishes the device-END comparison.
- Final gate retains fired=1, added=8, queued_at_test=8/16, result=injected,
  outcome=logger_busy; armed/active=none. USB active=0, paused=0, result=ok, queue=0/16.
  Logger ready, drops=0, error=none, no new stall/reset in capture. Queue high=9 is
  compatible with the completion record after the half-full trigger, not an overflow.
- current_size grows beyond the snapshot to 62520; same generation 21/newest 20,
  rotations=0. Largest internal block 51188 exceeds 20480; internal minimum 91928;
  stack minimum 3176 -> 2984 in this fixture build. No USB link losses.
- JP omitted the intermediate status between abort and retry. The persisted abort
  record, all eight records, successful retry and retained gate outcome substantiate
  cleanup/reuse; no intermediate status observation is claimed.

No firmware changes by Codex. JP's local DIAG_USB_TEST_FIXTURE=1 edit remains uncommitted
and untouched for the next case; tracked default stays zero. Prune and close regression
gates remain pending, and increment 2 remains unapproved.

### Next single case issued: prune an actively read disposable archive

No rebuild/flash; same fixture-enabled boot/build. Explicit DTR=true/RTS=false,
all browser test switches off; no Live. Capture status/log status. Send `log test file`,
wait for result=ok and note its new archive number N. Refresh files and preserve the
list. Send `log test prune N` using only that newly created synthetic archive number,
then download that archive. Expect Device: pruned and no saved partial file.
Refresh files and capture status/log status: only the fixture disappears, pruned rises
by one, result=pruned and outcome=pruned, logger ready and drops=0. Stop on mismatch.
Download current.log normally (CRC OK), then status/log status. Send full saved console
and that current.log. This exercises the production reader-close-before-unlink path;
it does not retest retention threshold selection. No separate fixture deletion is needed.

---

## Increment 1 USB gate 2 - 2026-09-21, 09:29-09:32 - PASS

Cancellation and reuse, same boot 95 and normal build. JP reports the test ran fine.
Evidence: attachment `cf8ccfce-6e10-4863-a073-1abd773a8e1b/Pasted text.txt` and
`C:/Users/photo/Downloads/95-current (1).log`.

- Deliberately damaged browser data caused `log abort` at 09:30:11.269; device replied
  `@@ERR reason=aborted` at .281 and browser reported abort confirmed. The 12 ms is
  host-observed command/response timing, not an isolated writer close measurement.
- Following status: active=0, paused=0, bytes=1296, result=aborted, queue=0/16.
  Retry snapshot contains `USB_GET_END bytes=1296 duration_ms=56 result=aborted`, seq=64.
- Normal retry: **41702 bytes, CRC OK**, 0.43 s. Local file has exactly 41702 bytes;
  independently computed CRC32 **E936A9DC**. Browser CRC OK establishes device-END
  comparison; the numeric CRC is calculated locally, not exposed in the capture.
- Post-retry current_size=41862 at 09:30:56, then 43139 at 09:32:07/14: **1277 bytes
  growth**. Same generation 21/newest archive 20, no rotation. Final active=0, paused=0,
  result=ok, ready, drops=0, error=none; no unexpected reset/stall in the capture.
- Internal largest block remains 51188 bytes, internal minimum 91952, writer stack
  minimum 2920. USB losses=0 for both new transfers; Wi-Fi/MQTT remain connected.

This proves cancellation cleanup and subsequent reservation reuse on hardware, not delayed
stale acknowledgements (those remain host-test coverage). Queue, prune and close gates
remain pending. Increment 2 is unapproved.

### Next single case issued: queue-pressure abort and normal retry

Use the existing fixture build: JP sets `DIAG_USB_TEST_FIXTURE=1` in
`src/diagnostics/diagnostics_config.h`, keeping DIAG_TEST_HOOKS=0, DIAG_ENABLED=1 and
DIAG_WRITER_STACK_PSRAM=1, then builds/flashes amoled-1-8-core-3-3-11. No generated-sketch
deletion is needed. This is existing bench instrumentation, not new firmware logic.
The checked-in default remains zero; restore it after fixture regression work.

Chrome: explicit DTR=true/RTS=false, all browser test switches off, no Live, Wi-Fi/MQTT
connected. Capture status and log status, send `log test queue` (expect armed=queue),
download current once (expect logger_busy and no partial save), then status/log status.
Require fired=1, result=injected, outcome=logger_busy, queued_at_test=8/16, added>0,
zero drops, and USB idle/unpaused. Stop on any mismatch. Download current again without
rearming (CRC OK), then status/log status. Send full saved console and successful file
so injected records and the terminal event can be checked. No later case issued yet.

---

## Increment 1 USB gate 1 - 2026-09-21, 09:23-09:25 - PASS

JP reports the test ran fine and the file downloaded. Checkout at review: `41dd46d`,
firmware implementation `7976818`, reviewed by Claude at `0ba72e5`.
Evidence: console attachment `bd3d1894-af58-4bd2-8264-ae1f7f796181/Pasted text.txt`
and local download `C:/Users/photo/Downloads/95-current.log` (09:23:31).

- Browser reports **32598 bytes, CRC OK**, 0.37 s, decoded 87865 B/s. Independent local
  file inspection confirms 32598 bytes and computes CRC32 **59978670**. The capture hides
  raw BEGIN/D/END frames, so that numeric CRC is a local calculation; browser CRC OK is
  the evidence for its device-END comparison. The snapshot ends with `USB_GET_BEGIN`,
  as expected; its own completion record cannot be in the frozen snapshot.
- Same **boot 95**, current generation 21, newest archive 20 throughout; rotations=0.
  Before transfer current_size=32438. At 09:23:45/50, post-transfer size=33890;
  at 09:24:52 size=34898; at 09:24:57 size=35032. Later growth **1142 bytes** confirms
  append continuation with no rotation ambiguity. The first post-transfer status was
  about 15 seconds after completion; the later samples still establish the gate.
- Post-transfer and final USB state: active=0, paused=0, bytes=32598, result=ok;
  queue=0/16, drops=0, failure valid=0. Logger ready, error=none, errno=0; writes 35 -> 45.
  No reset or new stall is visible in this capture. Wi-Fi and MQTT remain connected.
- One transient USB connection observation: losses=1, max_loss_ms=3, pending=0;
  recovered within the unchanged 1000 ms grace, with successful CRC completion.
- Internal largest block **51188 bytes**, above the **20480-byte** gate; internal minimum
  91952 bytes. Writer PSRAM stack minimum moved 3208 -> **2920 bytes** and then remained
  there in the later sample. This is one transfer, not a repeated-resource stability test.

The issued case requested DTR=true/RTS=false; opening is outside this cleared capture,
so those settings are procedural, not independently visible here. No build transcript
was supplied; running behavior is hardware evidence, not a compilation-log review.
No firmware changes, assistant builds or flashes in this result review. Queue-pressure,
prune-conflict, cancellation and close regression gates remain pending, one case at a
time. Increment 2 remains unapproved.

---

## Case 1 result - 2026-09-20, 18:21-18:55 - PASS, timing deferred

**Revised September 20 after Codex's evidence review (`6c3148e`).** The first write-up
stated several inferences as observations. Findings are now split into what was observed
and what was concluded from it. Corrections are marked; nothing in the pass verdict
changed, but four conclusions were withdrawn or narrowed.

**Rig.** Windows PC on the `iphone-jp` hotspot at `172.20.10.13/28`, phone at
`172.20.10.1`, Ethernet left connected throughout. Python 3.13.9
`python -m http.server 8000 --bind 172.20.10.13`, serving a scratch directory, never the
repository. The companion was powered and connected to the same hotspot for the whole
session. iPhone 13 Pro running **iOS 26.6.2**.

**Evidence available.** Server access log, `netstat` socket snapshots, a continuous ping
log, and JP's reports of what the phone displayed. **No request-header capture, no packet
trace and no Files screenshots.** Request headers were never recorded, which bounds several
findings below. Nothing here is packet-level proof.

**Verdict.** The listing loaded in Safari and files saved to the Files app at sizes
consistent with what was served. The case passes on its stated criterion.

### Observed

| # | Observation | Evidence |
|---|-------------|----------|
| O1 | Safari on the **hotspot-host phone** reached a **Wi-Fi client** of that hotspot and loaded the listing. | `172.20.10.1 - - [18:24:05] "GET / HTTP/1.1" 200` |
| O2 | One `GET /favicon.ico` accompanied the first listing load; the later listing load at 18:27:01 was not followed by another. | `"GET /favicon.ico HTTP/1.1" 404` at 18:24:05 |
| O3 | The same 262,144 bytes served as `application/octet-stream` produced a save prompt; served as `text/plain` it rendered inline as text. | `sample.log` versus `sample.txt`, identical bytes, differing `Content-Type` |
| O4 | Two distinct phone source ports appeared in `TIME_WAIT` after a download, on two occasions. | `:51950`/`:51952`, later `:51955`/`:51956` |
| O5 | Every logged request line used `GET`. No `HEAD` appeared. | Server access log, all entries |
| O6 | Files displayed `sample.bin` as **262 KB** and `big.bin` as **2.1 MB**, consistent with 262,144 and 2,097,152 bytes served. | JP's reports from the Files app |
| O7 | One interrupted transfer displayed on the phone as **"713 KB of 2.1 MB"**. | `big.log`, interrupted by a link loss |
| O8 | A 2 MiB body was served without server error, and a local fetch of the same file returned all 2,097,152 bytes. | Access log; local `Invoke-WebRequest` |

### Concluded, with its strength

- **C1 (from O1), strong.** The network path the feature depends on exists: the phone can
  reach an HTTP server running on a device attached to its own hotspot. **Not tested:**
  traffic between two tethered clients, and - the case that actually ships - reaching the
  **ESP32's** server rather than a laptop's.
- **C2 (from O2), moderate.** Safari requests a favicon. **Withdrawn: "once per session".**
  One non-repeat does not establish a caching rule. The firmware requirement does not
  depend on the frequency: **routing must acquire the reader reservation only for requests
  that need it.** A favicon may consume a socket; it must not take the session, touch SD or
  overwrite the last-transfer result. Under JP's rule it still counts as an HTTP request
  for the five-minute idle reset.
- **C3 (from O3), strong for MIME, untested for disposition.** Serving a log as
  `text/plain` puts it on screen instead of into Files, so the firmware sends
  `application/octet-stream`. **`Content-Disposition` was never exercised** - the Python
  server does not send it - so the attachment header is a **design decision**, taken
  because the filename carries the expected size and transfer ID, not a tested result.
  This evidence does **not** show that `text/plain` plus an attachment header would fail
  to download.
- **C4 (from O4), withdrawn as stated.** `TIME_WAIT` is a post-close state; two ports prove
  two **recently closed** connections, not two concurrent sockets, nor a purpose, nor "two
  per download". Two sequential requests preceded each snapshot and fully account for it.
  **The earlier claim that `max_open_sockets = 2` "would have been wrong" is withdrawn** -
  that setting counts clients and permits two, and nothing observed disproves it. Keep the
  2-3 proposal as a prudent start, measure live connection occupancy on firmware, and never
  tie a retrieval session to a TCP accept.
- **C5 (from O5), split.** *No `HEAD`* is supported for the observed request lines.
  ***No `Range`* is not established and the claim is withdrawn.** Verified independently:
  Python 3.13.9's `http/server.py` contains no `Range`, `Accept-Ranges` or `If-Range`
  handling at all, and `log_request` is called by `send_response()` and records only the
  request line and status. A `Range` request would therefore have been ignored, answered
  `200` with the full body, and left no trace. The honest statement: **observed downloads
  completed against a server with no resume support, so version 1 keeps full-body `200`
  behaviour** - not that the client never asks. Capture real `Range`/`If-Range` headers in
  firmware evidence.
- **C6 (from O6, O8), narrowed.** A 2 MiB transfer is feasible over this path. The phone
  sizes are **rounded Files displays, not byte-exact verification**; the local fetch
  validates the laptop's path, not the saved iPhone copy. Byte-exact confirmation still
  requires the exported-Safari-file gate. *Unit correction:* 262,144 bytes is **256 KiB**
  and 524,288 is **512 KiB**; the first write-up said 262 KiB and 524 KiB.
- **C7 (from O7), narrowed.** One interruption was visibly partial against `Content-Length`.
  This does **not** establish that every truncation is detected, nor what object Files
  finally retained. The deliberate truncated-response gate and the Safari-export comparison
  remain necessary.

Also noted: a logged `200` is written when the response starts, so **server status is not
evidence of a completed body**. Exact saved size and content are the stronger evidence.

### What this requires of the firmware

- **`application/octet-stream` with `Content-Disposition: attachment`** (C3).
- **SD-free, reservation-free handling of incidental requests** such as favicon (C2), with
  bounded servicing of a second request while a transfer streams - `max_open_sockets` alone
  does not make handlers concurrent.
- **Spare socket capacity above a single client** as a starting configuration, to be
  measured rather than assumed (C4).
- **Full-body `200` for version 1**, with an explicit, cheap and deliberate policy for
  `HEAD` and for a `Range` request that a future iOS may send (C5).
- `Content-Length` on every response, which is what made O7 visible.

### Two procedural findings

- **A Files "error" is not necessarily a failed download.** `big.bin` showed
  "Your device couldn't connect to the server" yet appeared at full size; Files was failing
  to *preview* an unrecognised binary, which it labels a "MacBinary archive". JP identified
  this. An earlier reading of it as a transfer failure was wrong and briefly motivated a
  `Range` hypothesis that C5 now shows the evidence cannot settle either way.
  **Confirm by size, not by whether the file opens** - and prefer an exact size to a
  rounded display.
- **The hotspot was absent from a PC scan while the companion stayed associated.** Treat
  this as a rig observation on this phone and date, not a universal rule about iOS.

### Timing: deliberately deferred

The 2 MiB duration was not obtained. Three attempts were interrupted by the PC losing the
hotspot (`ConnectionAbortedError 10053`, 12 ping timeouts, `netsh wlan` reporting
`disconnected`), and finally the PC could see the SSID but could not associate.

**Correction to the first write-up, which called this "the test rig".** The cause was not
established. The companion staying associated shows only that there was no total outage
affecting all clients; it does not identify what dropped the PC, and the SSID being absent
from a scan is hotspot-side behaviour. What can be said is narrower: **none of it involves
firmware, which does not exist yet**, and it is not a measurement of the feature.

Deferral stands. The number governing the 120 second `current.log` limit and the 5 second
no-progress abort is **ESP32-to-phone** behaviour, and average throughput alone would not
set either bound - the overall limit follows throughput, while stall safety depends on the
longest no-progress interval, queue occupancy and cancellation latency. The ESP32 is the
actual server but is **not yet proven to be the sole bottleneck**. For reference only, a
local fetch of the same 2 MiB file ran at roughly 1.2 MB/s. The firmware timing gates are
specified in [the implementation spec](sd_iphone_log_download_spec.md).

### Limits of this result

A laptop's Python server is not `esp_http_server`: response construction, timeouts, socket
limits and concurrency are all untested here. Findings are Safari's behaviour on one phone
running iOS 26.6.2, on one date, over plain HTTP with no TLS, with a single client, and
**without request-header capture**. A future iOS can change C2 through C5. The firmware
must pass its own reachability, response-contract and transfer cases.

---

## Case 1 - iPhone reachability proof (no firmware)

**Why this runs first.** Every later case assumes Safari on the iPhone can open a page
served by a device sitting on that same iPhone's Personal Hotspot. Nothing in the project
proves that yet: the companion talks *out* to MQTT and HTTPS, which is the opposite
direction. If this fails, the whole feature needs a different transport, and finding that
out now costs an evening instead of a firmware cycle.

It also answers two design questions cheaply, before any code is written:

- **Content type.** Does Safari render a `.log` inline instead of offering a download? That
  decides whether `Content-Disposition: attachment` and
  `Content-Type: application/octet-stream` are mandatory or merely tidy.
- **Link speed.** How long does 2 MiB actually take over the hotspot? That is the number
  behind the 120 second `current.log` limit and the 5 second stall bound.

**Nothing on the companion is involved.** Do not flash anything. Leave the companion
powered and connected to the hotspot as usual, so the subnet carries its normal client.

### Before you start

Record the iPhone's iOS version: **Settings > General > About > Software Version**. It goes
in the result and is needed context for every Safari behaviour observed later.

### Steps

1. **Turn on Personal Hotspot** on the iPhone 13 Pro. Connect the Windows laptop to it over
   Wi-Fi (not USB tethering - the companion uses Wi-Fi and that is the path under test).

   **Foreground Settings > Personal Hotspot only to establish or re-establish the
   association**, then switch to Safari to run the case. The two cannot be open at once on
   the same phone, so do not treat "keep that screen open" as a standing instruction. If the
   laptop later cannot rejoin, return to that screen, reconnect, and switch back.

2. **Create a scratch folder with three test files.** Use the session scratch directory, not
   the repository - a default directory server exposes everything below it.

   ```powershell
   $d = "$env:TEMP\hotspot_test"
   New-Item -ItemType Directory -Force $d | Out-Null
   # 256 KiB text file, roughly a real current.log
   $line = ("A" * 127) + "`n"
   Set-Content -Path "$d\sample.log" -Value ($line * 2048) -NoNewline -Encoding ascii
   # 256 KiB binary file
   [byte[]]$b = 1..262144 | ForEach-Object { $_ % 256 }
   [System.IO.File]::WriteAllBytes("$d\sample.bin", $b)
   # 2 MiB binary file, the archive size
   [System.IO.File]::WriteAllBytes("$d\big.bin", (New-Object byte[] 2097152))
   Get-ChildItem $d | Select-Object Name, Length
   ```

   Note the three sizes it prints. Those are the numbers you compare against on the phone.

3. **Find the laptop's hotspot address.** `ipconfig` - look for the wireless adapter
   connected to the iPhone. The address is normally `172.20.10.x`, with the phone itself at
   `172.20.10.1`. Confirm the link with `ping 172.20.10.1`.

4. **Serve the folder, bound to that address only:**

   ```powershell
   cd $env:TEMP\hotspot_test
   python -m http.server 8000 --bind 172.20.10.X
   ```

   Use your actual address. Binding it keeps the server off every other interface.

5. **Windows Firewall will prompt.** Allow the profile Windows actually assigned to the
   hotspot adapter - check it first with `Get-NetConnectionProfile`, since a hotspot is
   commonly classified **Public** and a Private-only rule would then never apply. Keep the
   rule narrow (one port, the hotspot address, the hotspot subnet). Do not disable the
   firewall. Remove the rule when the case is finished - step 9.

6. **On the iPhone, open Safari** and go to `http://172.20.10.X:8000/`. The directory
   listing should appear.

7. **Tap each file in turn and record what happens.** This is the actual data of the case:

   - `sample.log` - does Safari **display it inline** as text, or offer a download? This is
     the content-type question.
   - `sample.bin` - expect a download prompt. Save it to Files.
   - `big.bin` - start it and **time it**. While it runs, **lock the phone** for about ten
     seconds, unlock, and see whether the download resumed, continued or failed. Then repeat
     the download and **background Safari** (swipe to home) for about ten seconds instead.

8. **Check the saved sizes** in the Files app (long-press a file > Info, or the list view's
   size column) against the three numbers from step 2.

9. **Stop the server** with Ctrl+C and remove the firewall rule:
   `Windows Security > Firewall & network protection > Allow an app through firewall`, find
   the Python entry created in step 5 and remove it.

### What to send back

- iOS version.
- The laptop's hotspot IP, and whether `ping 172.20.10.1` worked.
- Per file: inline or download, and the size shown in Files against the expected size.
- `big.bin` duration in seconds, plus what happened on screen lock and on backgrounding.
- Anything unexpected: prompts, warnings, stalls, or a listing that loaded slowly.

### Pass criteria

The case passes if the directory listing loads in Safari and `sample.bin` saves to Files at
exactly the expected size. Everything else on the list is observation that shapes the
design rather than a pass condition.

### If the page does not load

Check the firewall rule and the `--bind` address **before** concluding that the hotspot
isolates its clients. A `ping 172.20.10.1` that works while the browser fails points at the
laptop, not the network. Only after both are ruled out is client isolation the explanation -
and that would be a genuine finding worth stopping on, because it would rule out the whole
local-HTTP approach.

### Limits of this case

A laptop's Python server is not the companion's `esp_http_server`, and passing here does not
certify the firmware's own responses, timeouts or socket limits. It establishes the network
path and Safari's handling of the two content types on this specific iOS version, on this
date. The firmware still has to pass its own reachability case later.
