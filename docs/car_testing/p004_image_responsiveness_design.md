# P004 - keep UI and IMU responsive during media network work

Revision 1, September 25, 2026. Author: Codex.
Status: **DEFERRED by JP on September 25, 2026**. No implementation authorized.
The proposed review/implementation/bench sequence below is inactive historical design.
JP accepted P005/P006 after both retained cases and authorized proceeding to this design.
Source baseline: 02bd7bf, including accepted P005/P006 implementation 734ad92.
No firmware changes, build, flash or new bench procedure accompany this document.

## JP decision - defer pending demonstrated practical impact

JP questioned whether keeping the UI responsive during occasional image network waits
justifies a dedicated worker and its ownership/cancellation complexity. Codex recommended
deferral, and JP agreed: address real problems without adding complexity for no valid reason.

The measured blocking remains valid evidence; it is not being dismissed or marked fixed.
However, JP has not specifically reported the two F003 image-request pauses as a practical
problem. The earlier G-meter/spinner complaints were associated with MQTT reconnection,
addressed separately by P001. P004 would improve responsiveness during network waits,
not necessarily make images arrive faster.

Do not implement P004, commission further design review, or run its proposed bench cases
as the automatic next step. Continue ordinary car use with the accepted MQTT improvements.
Reopen only when field observations establish meaningful image-related impact, or new
concrete evidence changes the cost/benefit assessment, and JP explicitly agrees. Record
the symptom, local time and supporting log; assess a narrower remedy before adopting
this worker architecture. Preserve the design and measured findings for reference.

This is a project decision: measured latency alone does not justify added architecture.
Prefer a demonstrated user problem, a clear expected benefit and the smallest effective
change. Deferral is not evidence that the blocking disappeared.

## 1. Problem, target and limits

F003 records IMAGE-request main-loop gaps of 5192 and 5264 ms, with HTTP work of
5087 and 5157 ms; shorter image/Live setup gaps remain after P001 fixes MQTT stalls.
These are network waits on main, not evidence of an IMU sampling defect. See
[field journal](field_journal.md), F003/I003 and Claude's review. P005/P006 improves
MQTT recovery timing; it does not change these image paths.

Target: during DNS, connection, headers, body waits and network cleanup, UI and IMU
service gaps <=100 ms, with no network-caused >=1000 ms main-loop gap. Slow/failing
requests may still take seconds; the spinner, touch and sensor servicing should continue.
Only main calls LVGL, rendering, screen navigation, touch/IMU, and application callbacks.

This design covers Latest/Back stills, MQTT-triggered stills, direct Live, motion-to-Live
handover, and Live reconnects between frames. It does not move or optimize JPEG decode,
blit or LVGL refresh. The existing synchronous still decode can take about 0.5 s per
source comments, and Live rendering also occupies main; those are residual limits,
not a promise of <=100 ms across every rendering interval. Measure them separately.
Do not hide rendering in a network metric to claim whole-UI responsiveness.

No WiFi radio tuning, P005 timeout retuning, new endpoint, image quality/geometry change,
Live pacing change, or new parallel media stream. No arbitrary URL or token logging.

## 2. Current source findings

- image_fetcher.cpp queues a pendingEndpoint but requestImage then calls blocking
  HTTPClient.GET() on main. Body processing drains an unbounded available loop using
  readBytes; the modulo-4096 LVGL call is not a time or IMU-service guarantee.
- cleanupImageRequest ends HTTP and stops the shared TLS client on main. Screen unload
  frees JPEG/pixel buffers. Those actions become unsafe if merely moving GET to a task.
- requestLatestImage may accept another notification during a still fetch; the old
  echo-suppression comment assumes MQTT was starved, which P001 already changed.
- video_stream.cpp borrows imageFetcherSecureClient(). ensureConnected blocks for
  DNS/TCP/TLS; sendRequest may block writing. Its polling loops have no per-turn byte
  bound. Calling something 'non-blocking' in a comment does not bound native TLS work.
- Live has one 64000-byte JPEG buffer. It sends frame N+1 before decoding N; N+1 waits
  in socket buffers until N is consumed. A worker must not overwrite N while main decodes.
- Stills cap JPEG at 128000 bytes; Live caps at 64000. Rendering formats, rotation,
  retained Live pixel storage and the 60-second Live duration are accepted behavior.
- diagnet::Span and diagop::Block manipulate main breadcrumbs; diagnosticsProbeBegin/End
  and diagmqtt service state are main-only. Do not transplant those into a worker.
- Power hooks currently invalidate MQTT and close diagnostics, but have no media-owner
  cancellation hook. A new worker requires one before radio/peripheral shutdown.

F003's retained DMA-largest low was 21492 bytes during Live, only 1012 above the 20480
byte gate. 'PSRAM available' alone is not a TLS admission argument. Do not add another
media TLS session or put a new 12 KiB task stack in internal RAM.

## 3. Recommended architecture

Add src/image/media_transport.h/.cpp: one persistent worker, created lazily on the first
admitted media request, priority 1/core 0, 12288-byte PSRAM stack and internal static TCB.
Check placement and allocation; refuse cleanly on failure, with no internal-stack fallback.
This repeats a placement pattern already exercised by the MQTT and HTTP lifecycle workers;
it does not replace the first real media handshake and stack measurement.

The worker is the sole user of one shared media WiFiClientSecure and one plain client.
Retain a single media secure object with boot lifetime; remove the public borrowed-client
accessor when both adapters migrate. Neither main nor MQTT closes/reads/writes it. Never
run old Live borrowed-client code beside the new still worker. Migrate both adapters in
one reviewed increment so there is no intermediate cross-task TLS loan protocol.

Main API is request/poll/cancel/release, never a synchronous connect or wait. Use fixed
mailboxes and generation IDs, with one media session and one pending request, no growing
queue. Critical sections copy metadata only, not HTTP payloads or pixel buffers. The
worker snapshots immutable endpoint configuration and request kind, never UI pointers.

Use bounded HTTP/1.1 transport on the worker for both adapters, factoring the existing
Live framing logic rather than keeping HTTPClient's hidden connect/header loops. Split
DNS, TCP and TLS using the same verified core interfaces as P001, with a pre-resolved
address plus hostname for SNI/certificate checking. No hostname reconnect fallback,
no setInsecure, and no certificate/endpoint-selection changes. Still's remote/local
choice continues to follow its existing ssid2 rule; Live retains its remote endpoint.
P006 network-priority mapping must not accidentally redefine this separate media policy.

The chosen raw HTTP path adds framing responsibility. This is an explicit design trade:
we need bounded phase waits and cancellation checkpoints without a second owner of
HTTPClient internals. Section 6 pins the supported response contract; review it closely.

For DNS, factor P001's bounded resolver into an instance-based helper, preserving MQTT's
existing behavior and tests. Each owner has its own two retained callback slots, so a
media cancellation cannot release a MQTT DNS slot. Each slot survives until its callback
acknowledgement, even after timeout; ERR_MEM/submission failure returns a bounded result.
No third allocation if both slots are retained. Never share request generations between
owners. This factoring requires full MQTT host regression; it is not a new MQTT policy.

## 4. State, admission and ownership

Main state: IDLE, STARTING, FETCHING, FRAME_READY, CANCELLING, FAULT_HELD.
Worker phase: idle, dns, tcp_setup, tls, request_write, headers, body, frame_held,
cleanup, fault_held. Include session generation and frame sequence in every mailbox.
No pointer or enum alone constitutes completion acknowledgement.

Before UI mutation or IMAGE_BEGIN/LIVE_BEGIN, main admits only if retrieval is OFF,
MQTT lease is not held, the media owner is idle, and existing screen/WiFi guards pass.
Publish the reservation synchronously before returning to the main loop; this closes
MQTT/retrieval races before worker dispatch. imageFetcherIsBusy/videoStreamActive or
a shared mediaTransportReserved predicate must include STARTING/CANCELLING/FAULT_HELD,
not just a visible loading screen. Both MQTT lease arbitration and retrieval entry check
that predicate. No second TLS handshake while MQTT owns the reconnect lease.

During an active still, additional Latest/Back/Live requests are refused as media_busy,
not automatically queued or used to free the active buffers. MQTT notifications use
imageNotification suppression, not an unsuppressed record per push. This is a deliberate
admission clarification: asynchronous execution exposes arrivals that used to be delayed
by main's blocking GET. A request after cleanup can be accepted normally. Preserve
post-display 10-second notification echo suppression and normal allowed-screen policy.

At a successful still display, the worker transport must already be closed and released.
Main retains display/handover flags as today: one second for motion still, 60 seconds
for a button still. A motion handover claims the idle media owner directly on main
without a free loop turn in between clearing the pending-handover flag and reservation.
If admission fails, use existing safe return behavior; never retry silently in a loop.
MQTT retry deferral and retrieval refusal during a pending display remain unchanged.

videoStreamStart returns 'accepted for asynchronous start', not proof of a handshake.
Existing callers must handle a later failure through main completion, restore previous
screen only when the media screen is still active, and never display stale success after
navigation. Stop is idempotent in all states and does not wait for the worker.

### Buffer ownership

| Resource | Owner / lifetime |
| --- | --- |
| TLS/plain clients and HTTP parser | Worker only, boot lifetime; transport closed at session end |
| Still JPEG <=128000 bytes | Worker allocates PSRAM; transfers pointer once on valid ready adoption after transport cleanup; main then decodes/frees |
| Still pixel buffer and LVGL descriptor | Main only; detach/replace widget reference before freeing |
| Live JPEG 64000 bytes | Retained PSRAM allocation; explicit per-frame write/read loan below |
| Live decoded pixels / descriptor | Main only, retained as currently required by LVGL |
| Commands/results/timing metadata | Fixed-size snapshots under lock; no credential strings in events |

On an unadopted still result, worker retains ownership; cancellation/stale-ready handling
must acknowledge discard and let worker free it. After adoption worker never touches it.
Use a handshake under the mailbox lock so main cannot free a pointer still being used.
Unloading a screen cancels a generation and detaches/frees only main-owned buffers;
it must not free worker-owned JPEG storage. Replace the existing unload/free path.

Live keeps one JPEG buffer: worker writes N, publishes immutable frame N metadata, and
holds that buffer unchanged until main release(N). Worker may send the GET for N+1 and
parse its bounded headers while main decodes/blits N, but MUST NOT read its body into
the JPEG buffer before release(N). Socket buffering provides the existing overlap.
Keep frame-N timing/length snapshots separate from frame-N+1 parser counters. Main posts
release on success, decode failure and navigation/cancellation exits, including after
nested lv_timer_handler calls. An invalidated generation rejects progress but still
accepts its matching release; a different generation cannot release the held buffer.
No second JPEG buffer, no second connection, no per-frame heap churn.

## 5. Cancellation, cleanup and failure behavior

Main navigation, Stop, replacement refusal, WiFi epoch change and power-down may revoke
work. WiFi callbacks only publish/invalidate link generation; they never touch transports
or LVGL. Repeated GOT_IP while up does not cancel a healthy transfer. A new same-IP
association still invalidates it. Snapshot IDs prevent a late DNS/native completion from
restoring a cancelled screen or handing its result to another request.

Cancellation immediately stops UI adoption and admits no replacement until two facts
are true: worker has closed transport and stopped writing, and any main frame loan is
released. Worker performs all stop/close calls. No main-task cross-thread socket close,
forced task deletion, or timeout-based free. Terminal metadata carries separate cancel,
worker-closed and buffer-release timestamps so the logs can prove the ordering.

Main may navigate away immediately; cleanup remains busy in the background. A native
connect/TLS call may not observe cancellation until it returns. This is cooperative
cancellation, not a 100 ms close promise. Once it returns, validate epoch/deadline before
writing HTTP or publishing any success. Polling loops check cancellation every turn.

If worker does not close within 25 s of request dispatch, or a cancelled session fails
to release resources within 25 s of cancellation, main raises one MEDIA_FAULT record
and a visible 'Image connection stuck. Wait or restart.' notice. For Live, do not use
the entire feed's age as a 25 s fault timer; time each frame request or pending cleanup.
Retain ownership and exclusion until a matching late cleanup/release, then clear the
notice on main. No automatic retry storm or forced memory reclamation.

Add mediaShutdown() to both power-down functions before diagnosticsClose/radio teardown.
It invalidates and rejects new work without waiting. The existing finite power-down
sequence proceeds; retain worker-owned memory until reset/power-off, do not free live
resources because a logger-close wait elapsed. Best-effort cancellation records may be
absent after SD close; never claim a worker completion which was not observed. Power-down
must not later navigate back from its Sleeping/Shutdown message.

## 6. Protocol, deadlines and scheduling

Preserve GET paths, authentication token, Host/SNI, CA checking, Latest/Back semantics,
Live query height/quality, 200-only acceptance, positive Content-Length and JPEG caps.
No redirects, chunked decoding, compression or server retry is added. Installed
HTTPClient default redirect policy is disabled; the current still path already rejects
missing Content-Length. Keep-alive is for sequential Live frames; always close a still
transport before transferring its JPEG. Do not read beyond the declared body length or treat unsolicited trailing data as the
next image. A sequential Live response belongs to the next frame only after its GET
has been sent. On framing error close before reuse.

Bound requests to 1024 bytes, still headers to 2048 total bytes and Live headers to its
existing 512-byte capacity, all in PSRAM scratch. Validate complete CRLF header framing,
status line, decimal Content-Length without overflow, and reject ambiguous duplicate
lengths, transfer-encoding or unsupported content encoding. Fail explicitly on overflow
rather than truncating. No response line, request line, URL, host or token in diagnostics.
These explicit still-header limits are new and require the successful server bench case;
HTTPClient did not impose this same application header cap. Do not accept compressed or
chunked bodies merely because Content-Length also appears; do not let loose substring
matches in the old Live headerInt parser become the shared parser's contract.

Budgets (monotonic timestamps, separate media constants from MQTT):

- Still total network deadline: 15 s from worker request dispatch, covering DNS through
  last body byte; loading UI fallback 20 s from accepted action stays on main. A complete
  JPEG published before its network deadline is not discarded simply because main
  adoption/decode runs later. Cancelled/screen-left generation still cannot display it.
- DNS wait up to 15 s, TCP connect 5 s, TLS handshake 5 s. Each worker-controlled phase
  uses the smaller of its cap and remaining request time; never grants a fresh 15 s at
  each phase. Native calls are checked on return; a late success is discarded.
- Still/Live request write and no-progress headers/body bound: 8 s, additionally capped
  by the request deadline. Reset progress only on positive accepted/read bytes. Poll with
  bounded byte quotas (<=2048 bytes per turn), vTaskDelay(1) during active waits and at
  most 10 ms idle polling. No yield-only or busy-spin loop.
- Live preserves 15 s per HTTP response from request send and the existing 60 s feed
  duration from accepted start. DNS/TCP/TLS setup gets a separate 20 s overall ceiling
  (15 s bounded DNS plus remaining TCP/TLS allowance); no unbounded native hostname DNS.
  Stop feed on request/setup error as today. Do not introduce per-frame retries.
- Five-second connect/lifetime timeout setters for native transports; five-second TLS
  handshake setting for media. P005's ten-second TLS/MQTT allowance is MQTT-only.

Current still GET can overrun its nominal 15 s because main cannot check the timer while
inside it. Enforcing that existing nominal deadline is a deliberate behavior correction,
not a claim that every formerly late success will still display. Record deadline failures.
Native calls may overrun budgets before yielding; main responsiveness must remain intact.

On a healthy Live connection there is no repeated DNS/TLS. If the server closes between
frames, worker setup may run again under the same exclusion and bounded setup policy.
No change to decoder, blit, frame pacing or accepted large-download FPS exception.

## 7. Memory and diagnostics

Keep MQTT's worker and media worker separate: an ONLINE MQTT session continues serving
callbacks while media runs. Serializing media against *new MQTT TLS* does not erase
memory already retained by an ONLINE MQTT client. Check current largest internal block
>=20480 immediately before media TCP/TLS setup; sample internal-free/internal-largest
and DMA equivalents during setup and at boundaries, with >=20 ms busy sampling cadence.
Refuse on gate failure, do not lower the gate or fall back to insecure/extra clients.

Allocate worker stack and bounded protocol scratch in PSRAM once; use an internal static
TCB. Define compile-time mailbox size limits (command/result combined <=1024 bytes),
no large request/header arrays on worker stack. Record before/after worker allocation and
first handshake. Require worker stack margin >=2048; measured internal-largest >=20480
through retained cases. F003's near-gate media sample warrants measurement, not an assumed
margin. Stop and revise if gates fail; no automatic additional memory trade approved.

Preserve IMAGE_BEGIN/HTTP/END and LIVE_BEGIN/CONNECT/END identifiers and their meanings,
but remove main-thread network Span wrappers around worker dispatch. Add bounded
MEDIA_NET_END with generation/session/frame, request kind, valid phase mask, DNS/TCP/TLS,
write/headers/body/cleanup durations, expected/received bytes and literal failure reason.
Separate metadata records if worst-case formatting would reach the 456-byte field limit.
Capture original worker completion time; report adoption/cancellation separately.
Fresh TLS error only when the called API actually updates it. No fabricated TLS diagnosis.

Refactor service observation into independent main-owned measurement slots for MQTT
and media; keep existing diagmqtt API/record behavior as a compatibility wrapper. Do not
reuse its single active/id state, because MQTT DNS may overlap a media request and Live
has repeated frame requests. Main service hooks feed both slots without duplicate LVGL
calls. No worker touches breadcrumbs or service counters. Preserve existing MQTT tests.

For stills report network-wait SERVICE through worker terminal adoption separately from
main decode/display spans. For Live report startup/reconnect network-only SERVICE,
aggregate steady-feed render/service statistics in its final summary, not SD records for
every frame. Keep bounded failure/long-gap records and worker MEM records at setup/end.
If render overlaps network (Live prefetch), label that context and do not apply a pure
network <=100 ms claim to a combined rendering window. No reopening average IMU Hz.

## 8. Implementation and verification proposal

One coherent implementation increment, after Claude design review and JP approval:
common media owner/DNS helper, both adapters, shutdown/admission, service telemetry and
host checks. Keeping both adapters in one increment avoids an unsafe temporary TLS loan
between tasks. Claude reviews source before JP compiles. No automatic deployment to car.

Required host checks execute real policy/parser/ownership bodies where practical:

- All media transport calls confined to worker; LVGL/render/IMU on main. No direct
  borrowed secure client left in either adapter. DNS helper retains independent slots,
  late callbacks and ERR_MEM safety, with unchanged MQTT timing/behavior tests.
- Admission ahead of UI mutation, blocked MQTT/retrieval entry while cleanup/held fault,
  suppression of repeated refused pushes, same-IP flap cancellation, no GOT_IP renewal
  cancellation, idle ready result cannot display after navigation or shutdown.
- Still pointer adoption/discard exactly once; failure/cancel before and after READY;
  pending screen unload never frees worker storage. Pixel descriptor detaches before free.
- Live frame N immutable through prefetch N+1, no body overwrite until release; matching
  release accepted after invalidation, stale other-generation release rejected. Cover
  screen change during nested lv_timer_handler, decode failure, handover and repeated Stop.
- Framing: split status/headers/body, upper/lower header names, missing/zero/negative/
  overflow/duplicate length, non-200, partial EOF, unsupported transfer encoding, buffer
  limits and binary JPEG data. Partial writes and trickle cannot extend absolute deadline.
- Worker failure/cancel cleanup before reuse, 25 s held fault and matching late recovery;
  no main stop/free/wait. Power hook precedes diagnostics/radio close, no later UI return.
- Independent service slots, original completion stamps, phase validity and record
  maximum widths; no secrets. Preserve all existing suites and MQTT facade behavior.

Proposed essential hardware scope: at most four focused cases, one issued at a time,
only after code clearance. No general SD/retrieval retest matrix.

1. Latest then Back success from the bench server: correct image/orientation/history,
   spinner servicing, phase/MEM/SERVICE and continued MQTT/logging.
2. One pending still interrupted by hotspot loss/navigation, then recovery and a fresh
   still: touch remains usable, stale image never appears, cleanup precedes reuse.
3. One full direct Live cycle: correct video, prefetch/FPS summary, resource gates,
   stop/return and subsequent idle ownership. Reuse same-sitting pre-change Live evidence
   only if available; otherwise report throughput without claiming a paired FPS pass.
4. One normal motion-triggered still-to-Live handover: no duplicate echo fetch, no TLS
   overlap, safe return. Host tests cover forced edge races; do not manufacture an
   exhaustive physical cancellation matrix.

For a meaningful Live performance comparison, a short baseline before JP flashes may
be needed; this is a review decision, not an instruction to run it now. Do not reuse
old-day FPS as a paired baseline. Native compile and measurements remain JP's work.

## 9. Decisions for review and JP approval

Recommendations for approval with this design:

- One shared media network worker covering both still and Live, one implementation
  increment, preserving main-thread decode/render and documenting that residual gap.
- Bounded raw HTTP for the controlled server contract, with explicit still header cap,
  instead of partially migrating HTTPClient or lending clients across tasks.
- Refuse new media requests while another fetch/cleanup is active, rather than queueing
  replacements; accepted display/motion-handover behavior remains as specified.
- PSRAM worker/metadata allocations, unchanged 20480/2048 gates, four essential hardware
  cases maximum planned; revise only for failed evidence. No new UI/quality/pacing feature.

Claude should challenge protocol compatibility, buffer-release races, deadline origins,
shared DNS helper impacts, memory cost while MQTT is ONLINE, and the split between
network responsiveness and remaining decode stalls. Resolve blockers before JP approves
implementation. P005/P006 is accepted independently and remains the rollback checkpoint.
