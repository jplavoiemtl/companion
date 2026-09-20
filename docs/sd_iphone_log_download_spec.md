# iPhone log retrieval - implementation spec

Status: **revision 2, for Codex review and JP approval.** Not implementation approval.
Branch `iphone-log-retrieval`. Consolidates the settled behaviour from the
[Claude review](sd_iphone_log_download_review_claude.md), the
[Codex review](sd_iphone_log_download_review.md) and the
[case 1 result](sd_iphone_log_download_bench.md). Where those disagree, this document wins;
where it is silent, they remain the reference.

Revision 2 answers Codex's spec review at `97f8d30`: it adds the extraction boundary
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

Each call states its caller and whether it may wait. **None of them waits on the network.**

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
without JP changing the decision. Any HTTP request resets it, including the last-result
view and favicon, as does a panel touch. An expiry-during-archive-transfer case is a
required bench gate.

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
reuse bytes a blocked `send()` still references**. Acknowledgements carrying a stale
generation are ignored. Progress, cancellation and result cross tasks through a defined
synchronised mailbox or atomic snapshot; **HTTP events never directly mutate main-owned mode
fields.**

### Teardown execution context and socket interruption

- **Main publishes cancellation and interrupts the socket.** A plain flag cannot interrupt a
  blocked socket call, and `httpd_sess_trigger_close()` queues work onto the HTTP task,
  which cannot run ahead of a blocked synchronous handler. Main therefore calls
  `shutdown(fd, SHUT_RDWR)` directly on the active socket descriptor, which forces a blocked
  `send()` to return an error so the handler unwinds.
- **The fd is published by the handler** under the session generation while it is in use and
  cleared before it returns. Main only interrupts an fd whose generation is current, which
  bounds the exposure to descriptor reuse.
- **`httpd_stop()` is never called from the main loop's power-transition or close path.** It
  runs only once transport release (B) is acknowledged and no handler is executing, on a
  later loop iteration - main polls across iterations and never blocks.
- **If release never arrives**, the mode stays in `STOPPING`, media stays refused, the
  reservation and buffer stay held, and the condition is recorded and escalated. Device
  shutdown does not wait for it.

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

**Second requests while streaming are queued behind the streaming handler**, bounded by the
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
| `max_open_sockets` | 2-3, measured | Counts clients; three more are reserved internally. Case 1 does not disprove 2 |
| `lru_purge_enable` | `false` (default) | A new connection must not evict an active transfer |
| `stack_size` / `task_caps` | 4096 internal, measured like the writer's | An overflow reboots the device |
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

## 12. Remaining decisions, with deadlines

| Decision | Deadline |
|----------|----------|
| Whether a per-session capability appears in the URL | Before increment 3 |
| Whether bounded interleaving of a second request is wanted, replacing the queued-reply choice in section 7 | Before increment 4 |
| Final `max_open_sockets` value from measured occupancy | Before increment 8 completes |

Settled in revision 2 and no longer open: entry **refuses** during a pending display or
handover; the idle deadline is enforced mid-transfer with a clean abort; `HEAD` returns
`405`; the route ID is a managed file ID.
