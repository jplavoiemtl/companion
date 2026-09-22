# iPhone log retrieval - implementation spec

Status: **revision 6.** JP accepted increment 1 and approved increment 2 on September 21.
JP accepted increment 2 on September 22 after eleven issued hardware cases passed; see
[handoff](sd_iphone_log_download_increment2.md) and [bench evidence](sd_iphone_log_download_bench.md).
Acceptance includes the proposed deferral of battery-only entry refusal and entry during
the brief pending-handover gap to controlled hardware checks before car deployment.
These two checks are not hardware passes; host coverage is retained.
JP approved increment 3 implementation after `d3a8dfe`, including a persistent lazy
4096-byte PSRAM lifecycle worker with internal static TCB. JP accepted increment 3 on
September 22; hardware startup-failure injection is deferred to increment 7, due before
car deployment, with host coverage accepted for increment 3. JP explicitly approved
increment 4 and reconfirmed no token on his trusted hotspot for actual log retrieval.
JP accepted increment 4 after gate A passed and approved increment 5 on September 22.
Increment 5 uses the existing streamed archive implementation for timing gate B; no
firmware change or rebuild is required. Increment 6 and beyond remain unapproved.
Branch `iphone-log-retrieval`. Consolidates the settled behaviour from the
[Claude review](sd_iphone_log_download_review_claude.md), the
[Codex review](sd_iphone_log_download_review.md) and the
[case 1 result](sd_iphone_log_download_bench.md). Where those disagree, this document wins;
where it is silent, they remain the reference.
The [approved increment 3 design](sd_iphone_log_download_increment3_design.md) specifies
lifecycle, socket/cache ownership and hotspot-interface enforcement. See the
[increment 3 code handoff](sd_iphone_log_download_increment3.md) for validation and the
installed-component rejected-accept cleanup refinement reviewed by Claude. See the
[increment 4 handoff](sd_iphone_log_download_increment4.md) for archive transfers and validation.

Revision 5 recorded JP's implementation approval and replaces the superseded provisional
cross-task socket shutdown scheme with the approved HTTP-owned I/O/lifecycle worker design.
The first server bench case verifies PSRAM-stack lwIP; on a misbehaving start the agreed
response is an internal worker stack, not a PSRAM/lwIP investigation.

Revision 4 records two JP decisions of September 22: what counts as activity for the idle
deadline (section 4), and no capability token (section 12).

Revision 3 added the release-acknowledgement invariant Codex raised at `724da61`, marks
section 5's teardown mechanism provisional with its three unresolved problems, and moves
server lifecycle and descriptor ownership to a before-increment-3 deadline.

Revision 2 answered Codex's spec review at `97f8d30`: it adds the extraction boundary
(section 3), replaces the cancellation and teardown contract (section 5), settles the
admission and idle questions Codex found contradictory (section 4), and pins the response
and record contracts (sections 7 and 8). Decision deadlines for the remaining HTTP choices
are in section 12.

Nothing here is built or flashed by an assistant. JP performs all builds and flashes, one
bench case at a time.

## 1. Scope

Retrieve managed log files from the companion onto an iPhone over the existing Personal
Hotspot, using a plain HTTP server that runs only inside an explicit download mode.

**In scope:** listing managed files; downloading one file; a last-result view; mode entry
and exit; media and power admission; a USB command to drive it on the bench.

**Out of scope for version 1:** TLS, resume or range requests, uploads, deletion, arbitrary
paths, multi-file bundles, ZIP, AP fallback, mDNS, cloud relay, retention changes, and the
LVGL screen, which follows once the bench cases pass and before the car step.

**Unchanged:** the writer owns SD exclusively; queue, retention, rotation and USB limits;
writer placement and priority; network timeouts, retries and endpoint selection; the
20480-byte internal largest-block gate; zero normal queue drops.

## 2. Vocabulary

Three identifiers that this spec keeps strictly separate:

| Term | Meaning | Lifetime |
|------|---------|----------|
| **Managed file ID** | Identity of a log file: the `current` token, or an archive **generation** number | As long as that file exists |
| **Transfer ID** | Per-boot monotonic number for one retrieval attempt | One transfer |
| **Session generation** | Token invalidating stale work after cancellation | One reservation |

A route ID is a managed file ID, **never** a list-row index and never a transfer ID. A
stale archive link returns **not found**; it never silently selects a newer file.

## 3. Increment 1: the extraction boundary

The shared reader/session service is extracted from `diagnostics_usb.cpp` **before any HTTP
code exists**. HTTP is absent from increment 1.

### Ownership

| Shared service owns | USB adapter retains |
|---------------------|---------------------|
| Managed inventory and file IDs | Command parsing (`log list`, `log get ...`, `log abort`, `log status`) |
| The single reservation and session generation | Base64 encoding and the `@@BEGIN`/`@@D`/`@@END` wire framing |
| Writer-only reader open/close, `current` snapshot pause/resume | USB link grace and connection sampling |
| Prune and close integration (`diagnosticsUsbBeforePrune()` equivalence) | Control and error output, `@@ERR` delivery |
| Transfer bounds, stop reasons, progress accounting, CRC32 | Public `@@STATUS` / `[LOG USB ...]` status compatibility |

### Call contract

Each call states its caller and whether it may wait. **None of them introduces a new wait
on the network or on a consumer.** This is not a claim that they are instantaneous: the
existing SD operations behind them still take the time they always did, and **existing USB
progress accounting is preserved exactly**, not redefined by the extraction.

| Call | Caller | Waits? |
|------|--------|--------|
| Request acceptance (reserve) | main | No - immediate accept or busy |
| Transfer start | writer | No |
| Writer tick (produce bytes) | writer | No |
| Metadata ready (frozen size published) | writer publishes, any reader consumes | No |
| Buffer consumption acknowledgement | transport adapter | No |
| Cancellation publication | main | No |
| Terminal result | writer publishes | No |

Request **acceptance** and transfer **start** are distinct: acceptance reserves the session
on main; start happens on the writer's next turn.

### Behaviour preserved unchanged in increment 1

144-byte raw chunks inside the 240-byte wire envelope; framing and CRC32; timeout origins
and progress semantics; the 1 second USB-loss grace; pause then open ordering and
resume-before-end ordering; the 50 % queue abort; archive prune ordering; shutdown
no-output behaviour; offline status fallback. **HTTP may choose different chunk sizes
later; USB must not change here.**

Host tests keep their behavioural assertions. Their source locations are adapted to the new
layout - assertions are not weakened or removed to make extraction pass. Add shared-session
cancellation and reuse tests.

### First bench gate for increment 1

**One case, not the suite:** a normal `current.log` transfer over USB with `status`, CRC
verification and confirmed append continuation. Review that before the next case is given.
Queue-pressure, prune-conflict, cancel and close cases all remain required before the
extraction is called accepted.

## 4. Mode state model, admission and idle

States, owned on the main task, never persisted to NVS:

`OFF` → `STARTING` → `ACTIVE` → `STOPPING` → `OFF`

- **`OFF` is the only state that accepts a new entry.** A repeated "on" while not `OFF` is
  refused with a stated result; a repeated "off" while already `OFF` or `STOPPING` is
  idempotent and reports so. Neither is silently ignored.
- **`STARTING`, `ACTIVE` and `STOPPING` are all exclusive to media**, and remain so until
  transport release (section 5). Teardown never hands media back while the server still
  holds memory.
- **Hotspot loss leaves the mode `ACTIVE`** with a `link_down` substate; it does not
  traverse `STOPPING` → `OFF`. Sockets close, the server re-listens on `GOT_IP`, and
  **`GOT_IP` does not reset the idle deadline.**
- A reboot returns in `OFF`. The mode must not disturb `screenMem`'s 30 second debounce.
- **Startup failure rolls back to `OFF`**, releasing anything already acquired and recording
  the reason. A terminal logger failure while `ACTIVE` forces `STOPPING`.

### Entry conditions - all required

`vbusPresent`; `usbStatus().ready`; **`!usbStatus().closing`** (separate fields,
`sd_diagnostics.cpp:1099-1101`); `WiFi.status() == WL_CONNECTED`; `!imageFetcherIsBusy()`;
`!videoStreamActive()`; `!diagnosticsUsbBusy()`; and **no pending display timeout or motion
handover**.

**Decision taken (was open item 1): entry is REFUSED during a pending display or handover -
the mode never cancels the pending transition.** Refusing preserves existing media
behaviour and introduces no navigation side effects; a visible busy reason lets JP return to
the dashboard and retry. The cancellation alternative is removed from this spec.

That condition uses a **dedicated image-module predicate** - for example
`imageFetcherHasPendingDisplay()` reporting `imageDisplayTimeoutActive || motionTriggered`.
**Do not widen `imageFetcherIsBusy()`**: the MQTT retry-deferral callers at
`companion.ino:2497-2506` depend on its current meaning.

Each failed condition records its own reason and is visible to the operator.

### Exit conditions

| Trigger | Behaviour |
|---------|-----------|
| Explicit (USB command, later the screen button) | Normal path |
| USB power loss | Immediate exit, clean abort of any transfer |
| Idle backstop | **5 minutes** with no HTTP request and no panel touch |
| Shutdown / deep sleep | Takes precedence; **never waits for `OFF`** |
| Hotspot loss | Mode stays `ACTIVE` in `link_down`; idle backstop is the ceiling |

### The idle deadline, corrected

Revision 1 claimed a long transfer could not expire the deadline. **That was backwards**
and Codex caught it: because the deadline resets on request *arrival*, a single transfer
running longer than five minutes sees no new requests and can expire mid-transfer. Current
snapshots are capped at 120 s, but **archives have no overall cap**, so a slow but
progressing archive can reach five minutes.

**JP's rule is enforced unchanged, including mid-transfer: at expiry the transfer is
cleanly aborted and the mode exits.** No progress-based reset and no transfer exemption
without JP changing the decision.

**What counts as activity, decided September 22 (revision 4).** Reset on actions the user
took: loading the listing, loading the last-result view, a file transfer, or a panel touch.
**Do not** reset on incidental browser traffic: `/favicon.ico`, unknown paths and rejected
methods. This supersedes the earlier wording that any HTTP request resets the deadline. The
reason for the split is that refreshing the result page to see whether a download worked is
a deliberate act and should hold the mode open, while background requests are not evidence
that anyone is present - and if they counted, a browser tab left open could keep the mode
alive indefinitely, defeating the one job this deadline has. An expiry-during-archive-
transfer case is a required bench gate.

### Increment 2 STOPPING observation policy (September 21 review correction)

With the USB-only adapter, a still-busy reservation after 10 seconds in STOPPING emits
one RETRIEVAL_STUCK event and one serial warning, retaining a release_stuck status flag.
It does not free resources or end media exclusion. A late release completes exit;
otherwise the operator must reboot. Event persistence is best-effort through the existing
queue, not guaranteed with a stuck writer. This is an observation threshold, not an
extension of USB transfer limits or the power-close caller wait. It does not resolve the
provisional HTTP descriptor/lifecycle/error-recovery contract below.

## 5. Ownership, cancellation and teardown

SD is writer-owned. No HTTP path opens a file descriptor. One retrieval session exists
across USB and HTTP, and **a session is never tied to a TCP accept** - incidental requests
get a socket, not the reservation.

### Two independent acknowledgements

Cancellation completes in two parts that must not be conflated:

- **A - writer cancellation.** Stop producing, close the reader, resume appends for an
  ordinary cancellation (or continue the existing final drain and close on shutdown),
  publish the cleanup result. **A never waits for the network handler, or for any
  network-owned lock or buffer.**
- **B - transport release.** The handler stops using the buffer, request and socket, and
  acknowledges release. **Only after B may that storage be freed or reused, and only then
  may server teardown finish.**

Logging therefore resumes on A, independently of B. The transfer buffer and the reservation
are **retained until B**, and new retrieval is refused in the interim.

### Buffer and generation rules

A fixed, bounded transfer buffer with explicit ownership, a used-length and offset, and a
**session-generation token**. Cancellation invalidates the generation but **cannot free or
reuse bytes a blocked `send()` still references**.

**Invariant (Codex, `724da61`): an invalidated generation rejects further *progress* but
must still accept the matching *release* acknowledgement for the retained buffer.**
Otherwise cancellation would make release impossible and the buffer could never be freed -
the deadlock this whole section exists to prevent. Stale acknowledgements from a *different*
generation are ignored; the release from the cancelled generation is not. This is covered by
the increment 1 cancellation and reuse tests. Progress, cancellation and result cross tasks through a defined
synchronised mailbox or atomic snapshot; **HTTP events never directly mutate main-owned mode
fields.**

### Approved teardown execution context and socket ownership

JP approved the [increment 3 design revision 2](sd_iphone_log_download_increment3_design.md).
The old candidate in which main called shutdown(fd) is superseded. Synchronous HTTP owns
all client descriptors; cancellable send/receive overrides unwind on cancellation without
cross-task descriptor operations. A persistent lifecycle worker, with 4096-byte PSRAM
stack and internal static TCB, owns httpd_start/registration/stop. main and the SD writer
never wait for HTTP teardown. The default pending function remains unset for plain TCP.

Generation-tagged startup/cancellation, handler quiescence, independent writer cleanup,
transport release and cache cleanup precede reuse/OFF as specified in that design.
Missing completion retains resources and media exclusion with one recorded escalation
and a main-owned lv_layer_top notice; late complete teardown clears the notice, otherwise
operator reboot is required. A 10-second observation threshold is not a force-free timer.
No change to diagnosticsClose's bounded caller wait or to the two-acknowledgement rules.

### Relationship to `diagnosticsClose()`

`diagnosticsClose()` (`sd_diagnostics.cpp:1552-1559`) **bounds the caller's wait** at
`min(waitMs, 500)` ms and **returns `false`** if the writer has not finished. It does not
guarantee that any SD operation completes inside 500 ms, and it never did. Retrieval
**adds no new wait on that path** and does not change that guarantee. A stalled card still
produces a bounded caller return and a `false` result.

`send_wait_timeout` cannot help here: it is `uint16_t` in **seconds**
(`esp_http_server.h:200-201`), so no positive value expresses a sub-500 ms bound.

Existing transfer bounds carry over unchanged: `CURRENT_MS` 120 s overall for `current.log`,
`STALL_MS` 5 s no-progress abort, the 50 % queue-pressure abort (8 of 16 events), and reader
close before an archive is unlinked.

### Increment 4 implementation ownership refinement for review

The writer dispatches the existing shared reservation queue to USB or HttpArchive; HTTP
never consumes that queue or calls reader lifecycle methods. A locked mailbox copies
one 144-byte chunk; HTTP uses a private copy and posts cumulative accepted-byte counts.
The writer alone updates its reader offset/CRC and closes before prune/shutdown. Positive
body writes reset the body no-progress timer; headers/metadata do not count as progress.
The shared reader still applies the five-second guard from request acceptance until body
progress; both current limits remain unchanged. No current.log HTTP route in increment 4.

A terminal-writer exception is required for a late transport release: after the writer
has closed the reader and published Parked/Deleted, main's existing offline tick may
free retained reader buffers/release the reservation. It performs no SD work and cannot
overlap writer access. This extends the section 3 writer-only release rule only after
terminal ownership transfer, and is explicitly flagged for Claude's review. The writer
publishes close completion and never waits for HTTP before the close caller can return.
Lifecycle teardown waits off-main for handler and reservation release; missing release
retains exclusion/resources with the existing visible error policy.

## 6. Media admission

**One authoritative guard, ahead of every side effect.** Both still callers run
`imageBegin()` *before* `prepareForRequest()` (`image_fetcher.cpp:648`, `:693`), and
`imageBegin()` also marks the previous operation replaced, so a refusal after that point
leaves an unpaired `IMAGE_BEGIN`. Admission is checked **before any lifecycle mutation, UI
change, buffer allocation or pending-endpoint assignment.**

| Path | Site |
|------|------|
| Remote Latest over MQTT | `companion.ino:632` via `requestLatestImage()` |
| Latest button | `image_fetcher.cpp:658` via `requestLatestImage()` |
| Back button | `image_fetcher.cpp:688`, before `imageBegin()` |
| Live button | `video_stream.cpp:734` via `videoStreamStart()` |
| Motion handover | `image_fetcher.cpp:330` via `videoStreamStart()` |

`videoStreamStart()` is the **only** guard for the direct Live path, not a second
independent one - that path never calls `prepareForRequest()`.

A check inside `prepareForRequest()` is a **chokepoint assertion, not the authority**: if it
fires it records `path=late` and closes any lifecycle it finds open. Changing that function
to return `bool` is part of this, but `if (!prepareForRequest()) return;` alone is not a
sufficient contract. Preserve `motionTriggered` assignment after preparation, which clears
it, and the preparation-inclusive timing `imageBegin()` measures.

Refusals reuse existing vocabulary with a `download_mode` reason:
`IMAGE_REFUSED trigger=... reason=download_mode`,
`LIVE_REQUEST result=refused reason=download_mode`, and
`imageNotification("ignored_download_mode")`.

## 7. HTTP response contract

| Route | Response |
|-------|----------|
| `GET /` | Plain HTML listing, no JavaScript, from the **cached** inventory; includes the last-transfer result |
| `GET /f/<managed-file-id>` | One managed file |
| `GET /favicon.ico` | `204`, no body, no SD, **no reservation**, no effect on the last result |
| `HEAD` on any route | **`405` with `Allow: GET`**, no reservation, no SD |
| Any other method | `405` with `Allow: GET` |
| Unknown path | `404`, cheap, same properties as favicon |
| Route valid, session busy | `503` with a stated reason, no reservation attempt |
| Managed file absent | `404` - never a silently newer file |

`HEAD` returning `405` is a deliberate version-1 policy choice, **not implied by case 1**,
which could not observe request headers at all. An honest `200` reply to `HEAD` would have
to publish a frozen `current.log` size, which means taking a snapshot - the cost the cheap
path exists to avoid.

Download response headers:

```
Content-Type: application/octet-stream
Content-Disposition: attachment; filename="<boot>-<transferID>-<name>-<bytes>.log"
Content-Length: <frozen snapshot bytes>
Cache-Control: no-store
```

- `Content-Length` is mandatory - it is what made a truncated save visibly partial on the
  phone.
- **The snapshot is immutable once headers are sent.**
- A `Range` request is answered with a **full-body `200`** in version 1, and the arrival of
  `Range` or `If-Range` is recorded. Case 1 cannot show these are absent.
- **A post-header abort closes the connection without satisfying `Content-Length`.** Error
  text is never appended to a log body.
- Non-keep-alive by default; revisit only with measurements.

### Listing, cache and concurrency

Inventory requires writer SD work, so the listing is served from a **cached inventory
refreshed while idle**. During an active transfer, `/` serves the cached copy **marked
stale**, shows the busy state, and performs no SD access. The last-result view is **SD-free**
and may be a separate route; incidental requests never overwrite the stored result.

One response per TCP connection is enforced by the HTTP handler's failure return after
sending the response, which the installed component propagates to socket cleanup without
another response body. Same-connection pipelined requests are not served; a browser uses
a new connection. Listing/result pages are bounded non-chunked HTML from a 32768-byte
PSRAM buffer, released after successful server stop.

**Second requests on other connections while streaming are queued behind the streaming handler**, bounded by the
transfer's own limits, and the delayed reply is documented and tested. This is the explicit
version-1 choice: `esp_http_server` services requests from one task, so socket capacity
alone does not make handlers concurrent. If bounded interleaving is wanted later it needs a
different mechanism, decided before increment 4.

## 8. Records

Fields are assigned to the record that can actually carry them:

| Record | Fields |
|--------|--------|
| `HTTP_GET_BEGIN` | Transfer ID, managed file ID, started time. **No final size, no CRC, no result** |
| Metadata (at header emission) | Transfer ID, frozen size |
| `HTTP_GET_END` and last result | Matching transfer ID, expected bytes, bytes accepted by transport, CRC32 coverage, result |

- `HTTP_GET_BEGIN` precedes the `current.log` snapshot being frozen, so it cannot contain
  the final size - and a snapshot cannot contain its own size.
- **`HTTP_GET_END` is not always available:** it cannot appear in its own snapshot, and a
  shutdown or SD failure may prevent it persisting. Existing shutdown and end-record absence
  semantics are unchanged.
- **Progress means positive body bytes accepted by the transport** - not writer prefetch and
  not unrelated HTTP requests. Startup and header time is accounted separately from body
  progress.
- **CRC on failure covers exactly the reported prefix**, with partial-send and retry
  handling specified so it is counted once.
- Device **send completion is recorded separately from phone save completion**, and
  socket-accepted bytes are never presented as a verified save.
- `RETRIEVAL_MODE action=enter|exit trigger=... reason=...` for the mode itself. Guard
  refusals reuse `IMAGE_REFUSED`, `LIVE_REQUEST` and the image-notification vocabulary.
- A per-session capability in the URL, if used, is **never logged**.

## 9. Configuration

| Setting | Value | Reason |
|---------|-------|--------|
| `max_open_sockets` | 3 initially | Counts clients; three more are reserved internally. Measured tuning in increment 8 |
| `lru_purge_enable` | `false` (default) | A new connection must not evict an active transfer |
| `stack_size` / `task_caps` | 6144 internal, cleared at af9523f after gate A measured 696 bytes remaining with 4096 | Recheck HTTP margin and unchanged 20480-byte largest-block gate after JP rebuild |
| `core_id` | `tskNO_AFFINITY` | TCP/IP is pinned to core 0, writer on core 1; pinning is unevidenced |
| `max_req_hdr_len` / `max_uri_len` | 1024 / 512 defaults | Bounded request input |

## 10. Host checks

- `tools/tests/media_admission.test.cjs` - enumerate every call site of
  `prepareForRequest()` and `videoStreamStart()` across the production tree, excluding
  comments, asserting each is guarded or is itself a guard. **It proves nothing about
  runtime invariants**, so pair it with behavioural cases: a refusal leaves no pending
  endpoint, no UI or buffer mutation and no unpaired begin/end; after an allowed entry and
  exit, Latest, history-back and motion handover still work.
- Shared-session cancellation and reuse tests, added with increment 1.
- Existing checks are the USB regression gate: `usb_connection_guard`, `usb_logger_gate`,
  `sd_log_browser`, `network_diagnostics`.

## 11. Increments, one bench gate each

| # | Increment | First bench gate |
|---|-----------|------------------|
| 1 | Shared reader/session extraction; no HTTP | Normal `current.log` USB transfer: status, CRC, append continuation. Queue, prune, cancel and close cases follow before acceptance |
| 2 | Mode state machine and admission, USB command entry, no server | Entry refused on battery, during media, and during a pending handover; exit on USB loss; idle expiry; repeated on/off results |
| 3 | Server lifecycle, listing, favicon, last-result view; no transfer | Start/stop, address display, idle memory, repeated entry/exit; favicon takes no reservation; startup-failure rollback |
| 4 | Small immutable archive download | **Timing gate A** |
| 5 | Representative 2 MiB archive | **Timing gate B** |
| 6 | `current.log` snapshot | **Timing gate C** |
| 7 | Failure paths | Non-reading client, cancel, phone lock and background, hotspot loss, prune conflict, shutdown, power loss, re-entry, stale completion, deliberate truncated response, **idle expiry during an archive transfer** |
| 8 | Memory and contention | MQTT reconnect inside the mode with the server allocated; maximum admitted clients; repeated reconnect/close/re-entry; server task stack margin |
| 9 | Single-session competition | USB request during an HTTP transfer and the reverse |
| 10 | Server-off regression | Same-sitting Latest/Live/IMU, no new stalls or resets |
| 11 | LVGL entry screen, then the car | CAR build, accepted profile, blank FAT32, retrieval with the car running |

### Timing gates

**Gate A - first small immutable archive.** Exact bytes; request, start and end monotonic
times; time to first body byte; **maximum no-progress interval**; result and CRC32; queue
high-water and drops; internal free and largest block; writer and server stack margins. JP
confirms the actual Safari save and **exports that saved file for byte comparison**.

**Gate B - representative 2 MiB immutable archive**, separately. **Inherits every gate A
metric** and adds steady-state transfer behaviour and exported-file integrity. This is where
average throughput for the overall bound comes from.

**Gate C - `current.log` snapshot under ordinary logging.** Inherits gate A's metrics and
adds paused duration, queue-pressure early-abort behaviour, resume, and zero drops. **A fast
archive does not justify holding appends paused for 120 s.**

**Recorded separately in every gate**, because cancellation safety depends on them:
terminal no-progress interval, abort reason, cancellation publication time, reader close
time, append resumption time and transport release time.

**Successful transfers alone cannot justify stall tuning.** Controlled **slow-client** and
**queue-pressure** cases are required before `CURRENT_MS` or `STALL_MS` change. Both stay at
120 s and 5 s pending that data.

### Deferred usability proposal - finding logs by event time

Recorded September 22 at JP's request during increment 5. In the car, JP needs to find
which file contains a strange event without knowing its archive number. JP proposed
including the file's start date/time in its download filename.

Proposed scope for design review after increment 6 (current.log retrieval) passes, before
increment 11's final download screen and car deployment:

- Show each file's covered date/time range on the listing, alongside archive identity
  and size, so an event can be matched to a file. For current.log, distinguish the displayed
  range from the later frozen download snapshot.
- Include a trustworthy start date/time in the exported attachment filename while retaining
  archive identity. Preserve existing SD filenames and managed-file route identities.
- Label the timezone explicitly. If the clock was unsynchronized at file start, show the
  start as unavailable; do not present the first synchronized record as the file's actual
  start. Define behavior for clock corrections and files spanning boots during review.
- Review metadata storage, recovery and caching with Claude; avoid repeated full-file scans
  or adding SD reads to ordinary HTTP listing requests. Exact format and implementation
  remain open. Existing archives with missing metadata need an explicit fallback.

This is a recorded proposal, not implementation approval or a change to gate B. Revisit
with JP at the increment 6 acceptance checkpoint and obtain approval for a bounded follow-up.
Validate displayed ranges and exported names against log contents, including unsynchronized
starts, before relying on them in the car. Current testing continues unchanged.

## 12. Remaining decisions, with deadlines

| Decision | Deadline |
|----------|----------|
| **Server lifecycle teardown context and descriptor ownership** - the three problems in section 5 | Answered by the increment 3 design: cancellable transport overrides, no descriptor leaves HTTP execution, lifecycle worker off main |
| **Provisional `max_open_sockets`** to build increment 3 against | 3, with `lru_purge_enable` false; measured tuning completes in increment 8 |
| Whether bounded interleaving of a second request is wanted, replacing the queued-reply choice in section 7 | Before increment 4 |

**Decided September 22: no capability token.** Reconfirmed explicitly by JP for increment 4
when actual log contents become available: any device on his trusted hotspot can download
while the mode is active. Five minutes means inactivity expiry, reset by approved user
actions, not an absolute five-minute session lifetime.

 JP's reasoning, accepted: the server exists
only while download mode is active, the mode is off by default and needs USB power and a
deliberate entry, it closes itself after five minutes, and the hotspot is WPA-protected with
no other devices expected. A token's cost was never the comparison - it was delivery, which
without an entry screen forced a serial reveal, and this project commits console captures to
git, so manual redaction would have been the only control over the one secret in the design.

Dropping it also removes the serial `log mode url` exception, the redaction discipline, the
RNG-while-Wi-Fi-active dependency, invalidation on exit and the 403 path.

The structural protections stay and do the practical work: the server exists only during the
mode, managed-file IDs only with no arbitrary paths, no upload or delete, no permissive
CORS, `Cache-Control: no-store`.

**Correction, September 22:** an earlier revision of this paragraph said the server is
"bound to the hotspot interface". That is not achievable - `esp_http_server` has no
interface-binding option and listens on all interfaces at `server_port`. Codex identified
this. The equivalent restriction is operational: the device runs STA-only, and each
accepted connection's local address is checked against the current hotspot STA address
before it is served.

**Revisit when file bodies are served (increment 4 or later)**, because the exposure changes
from a listing of names and sizes to the logs themselves, and by then an entry screen may
make on-panel display or a QR code trivial. Also revisit if the hotspot password is shared
beyond JP. Logs carry no credentials - SSID, password and BSSID are already excluded - but
they do carry operational metadata: entrance-camera motion events, power and driving
patterns, network quality and screen usage.

Settled in revision 2 and no longer open: entry **refuses** during a pending display or
handover; the idle deadline is enforced mid-transfer with a clean abort; `HEAD` returns
`405`; the route ID is a managed file ID.
