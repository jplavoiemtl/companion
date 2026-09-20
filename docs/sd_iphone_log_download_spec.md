# iPhone log retrieval - implementation spec

Status: **draft for Codex review and JP approval.** Not implementation approval.
Branch `iphone-log-retrieval`. Consolidates the settled behaviour from the
[Claude review](sd_iphone_log_download_review_claude.md), the
[Codex review](sd_iphone_log_download_review.md) and the
[case 1 result](sd_iphone_log_download_bench.md). Where those disagree, this document wins;
where it is silent, they remain the reference.

Nothing here is built or flashed by an assistant. JP performs all builds and flashes, one
bench case at a time.

## 1. Scope

Retrieve managed log files from the companion onto an iPhone over the existing Personal
Hotspot, using a plain HTTP server that runs only inside an explicit download mode.

**In scope:** listing managed files; downloading one file; a last-result view; mode entry
and exit; media and power admission; a USB command to drive it on the bench.

**Out of scope for version 1:** TLS, resume or range requests, uploads, deletion, arbitrary
paths, multi-file bundles, ZIP, AP fallback, mDNS, cloud relay, retention changes, and the
LVGL screen (which follows once the bench cases pass, before the car step).

**Unchanged:** the writer owns SD exclusively; queue, retention, rotation and USB limits;
writer placement and priority; network timeouts, retries and endpoint selection; the
20480-byte internal largest-block gate; zero normal queue drops.

## 2. Mode state model

Four states, owned on the main task, never persisted to NVS:

`OFF` → `STARTING` → `ACTIVE` → `STOPPING` → `OFF`

- **`STARTING`** brings up the server. **`STOPPING`** tears it down and waits for resources
  to be released.
- **`STARTING`, `ACTIVE` and `STOPPING` are all exclusive to media.** Media is refused in
  every one of them. Teardown must not hand media back while the server still holds memory.
- A reboot returns in `OFF`. The mode must not disturb `screenMem`'s 30 second debounce.

### Entry conditions - all required

`vbusPresent`; logger ready by the existing `usbStatus().ready` definition;
`WiFi.status() == WL_CONNECTED`; `!imageFetcherIsBusy()`; `!videoStreamActive()`;
`!diagnosticsUsbBusy()`; **and no pending display-timeout or motion handover**
(`imageDisplayTimeoutActive` / `motionTriggered`).

That last condition exists because `imageFetcherIsBusy()` reports *not busy* between a
displayed motion still and its handover (`image_fetcher.cpp:591-597` versus `:227-230`).
Entering in that window would let the pending handover call `videoStreamStart()`, be
refused, and run `returnToPreviousScreen()` underneath a download session. Either refuse
entry, or cancel the pending transition on the main task before entering - not both
implicitly. **Do not widen `imageFetcherIsBusy()`**: the MQTT retry-deferral callers at
`companion.ino:2497-2506` use it for a different purpose.

Each failed condition records its own reason and is visible to the operator.

### Exit conditions

| Trigger | Behaviour |
|---------|-----------|
| Explicit (USB command, later the screen button) | Normal path |
| USB power loss | **Immediate exit, clean abort** of any transfer |
| Idle backstop | **5 minutes** with no HTTP request and no panel touch |
| Shutdown / deep sleep | Takes precedence over everything |
| Hotspot loss | **Mode stays open.** Close sockets, re-listen on `GOT_IP`; the idle backstop is the ceiling |

Every exit runs the existing abort path and records which trigger ended the session.

USB-power loss is detected in `updatePowerStatus()`, which runs on the main task.
`finishError()` is `static` inside `diagnostics_usb.cpp` and manipulates SD directly, so
**main raises a cancellation flag and the writer acts on it** - main never performs SD
cleanup itself.

## 3. Media admission

**One authoritative guard, ahead of every side effect.** Both still callers run
`imageBegin()` *before* `prepareForRequest()` (`image_fetcher.cpp:648`, `:693`), and
`imageBegin()` also marks the previous operation replaced. A refusal after that point
leaves an unpaired `IMAGE_BEGIN`. Admission is therefore checked **before any lifecycle
mutation, UI change, buffer allocation or pending-endpoint assignment.**

Refusal points, where the trigger is still known:

| Path | Site |
|------|------|
| Remote Latest over MQTT | `companion.ino:632` via `requestLatestImage()` |
| Latest button | `image_fetcher.cpp:658` via `requestLatestImage()` |
| Back button | `image_fetcher.cpp:688`, before `imageBegin()` |
| Live button | `video_stream.cpp:734` via `videoStreamStart()` |
| Motion handover | `image_fetcher.cpp:330` via `videoStreamStart()` |

`videoStreamStart()` is the **only** guard for the direct Live path, not a second
independent one - that path never calls `prepareForRequest()`.

**Chokepoint assertions, not authority.** A check inside `prepareForRequest()` exists to
catch a caller nobody has written yet. If it fires it records `path=late` and closes any
lifecycle it finds open. Changing `prepareForRequest()` to return `bool` is part of this,
but `if (!prepareForRequest()) return;` alone is **not** a sufficient contract. Preserve
`motionTriggered` assignment after preparation (preparation clears it) and the
preparation-inclusive timing `imageBegin()` measures.

Refusals reuse the existing vocabulary with a `download_mode` reason:
`IMAGE_REFUSED trigger=... reason=download_mode`,
`LIVE_REQUEST result=refused reason=download_mode`, and
`imageNotification("ignored_download_mode")` for the MQTT path.

## 4. Ownership, cancellation and shutdown

- **SD is writer-owned.** No HTTP path opens a file descriptor. The server task reads from
  a bounded handoff buffer the writer fills.
- **One retrieval session across USB and HTTP**, with an explicit busy response in both
  directions. A session is **never** tied to a TCP accept: incidental requests get a socket,
  not the reservation.
- **Cancellation is published by main and executed by the writer**, independently of the
  HTTP handler. Writer cleanup and append resumption must proceed whether or not the
  handler has noticed.
- **`send_wait_timeout` cannot bound shutdown.** It is `uint16_t` in **seconds**
  (`esp_http_server.h:200-201`), so no positive value fits the 500 ms budget in
  `diagnosticsClose()`. Closing the listening socket does not interrupt an accepted
  connection; `httpd_sess_trigger_close()` queues work onto the HTTP task; `httpd_stop()`
  waits for that task. **Never call `httpd_stop()` from the main loop's power-transition or
  close path.**
- Existing transfer bounds carry over unchanged: `CURRENT_MS` 120 s overall for
  `current.log`, `STALL_MS` 5 s no-progress abort, queue-pressure abort at 50 % (8 of 16
  events), and `diagnosticsUsbBeforePrune()` equivalence so a reader closes before its
  archive is unlinked.

## 5. HTTP response contract

Routes, all `GET`:

| Route | Response |
|-------|----------|
| `/` | Plain HTML listing, no JavaScript, generated from the same inventory the USB `log list` path uses. Includes the last-transfer result. |
| `/f/<id>` | One managed file. `id` maps through the inventory; **no path is ever built from request input.** Reuse `archiveName()`'s exact `archive-%08u.log` form. |
| `/favicon.ico` | `204`, no body, no SD, no reservation, no effect on the last result. |
| anything else | `404`, cheap, same properties as favicon. |

Download response headers:

```
Content-Type: application/octet-stream
Content-Disposition: attachment; filename="<boot>-<id>-<name>-<bytes>.log"
Content-Length: <exact snapshot bytes>
Cache-Control: no-store
```

- `Content-Length` is mandatory - it is what makes a truncated save visibly partial on the
  phone.
- The filename carries **boot, a per-boot monotonic transfer ID, the file and the expected
  byte count**. Boot plus size is not a unique download identity.
- **Method and range policy is explicit, not incidental.** `HEAD` is answered cheaply
  without a body. A `Range` request is answered with a **full-body `200`** in version 1;
  the firmware must not assume one never arrives. Record when either is seen - case 1 could
  not observe request headers, so their absence there proves nothing.
- **A post-header abort closes the connection without satisfying `Content-Length`.** Never
  append error text to a log body and complete the response.
- Non-keep-alive by default; revisit only with measurements.

The last-result view is **SD-free** and shows transfer ID, name, expected bytes, bytes the
transport accepted, CRC32 and result. Incidental requests never overwrite it. It reports
**device send completion**, which is explicitly not a phone save.

## 6. Configuration

| Setting | Value | Reason |
|---------|-------|--------|
| `max_open_sockets` | 2-3, measured | Counts clients; three more are reserved internally. Case 1 does not disprove 2 |
| `lru_purge_enable` | `false` (default) | A new connection must not evict an active transfer |
| `stack_size` / `task_caps` | 4096 internal, measured like the writer's | An overflow reboots the device |
| `core_id` | `tskNO_AFFINITY` | TCP/IP is pinned to core 0, writer on core 1; pinning is an unevidenced optimisation |
| `max_req_hdr_len` / `max_uri_len` | 1024 / 512 defaults | Bounded request input |

Bounded servicing of a second request while streaming is a **design requirement**;
`max_open_sockets` alone does not make handlers concurrent.

## 7. Records

- `RETRIEVAL_MODE action=enter|exit trigger=... reason=...`
- `HTTP_GET_BEGIN` / `HTTP_GET_END` beside the USB pair, carrying transfer ID, expected
  size, bytes accepted by the transport, CRC32 and result **atomically**. CRC is counted
  exactly once despite partial sends or retries.
- **`HTTP_GET_END` is not always available:** it cannot appear in its own snapshot of
  `current.log`, and a shutdown or SD failure may prevent it persisting.
- Guard refusals reuse `IMAGE_REFUSED`, `LIVE_REQUEST` and the image-notification
  vocabulary. No other new record types.
- A per-session capability in the displayed URL is optional; if used it is **never logged**.

## 8. Host checks

- `tools/tests/media_admission.test.cjs` - enumerate every call site of
  `prepareForRequest()` and `videoStreamStart()` across the production tree, excluding
  comments, and assert each is guarded or is itself a guard. **It proves nothing about
  runtime invariants**, so pair it with behavioural cases: a refusal leaves no pending
  endpoint, no UI or buffer mutation and no unpaired begin/end; after an allowed entry and
  exit, Latest, history-back and motion handover still work.
- Existing checks are the USB regression gate for the shared-reader extraction:
  `usb_connection_guard`, `usb_logger_gate`, `sd_log_browser`, `network_diagnostics`.

## 9. Increments, one bench gate each

| # | Increment | Bench gate |
|---|-----------|-----------|
| 1 | Shared reader/session service extracted from `diagnostics_usb.cpp`; no HTTP | Existing USB cases re-run; host checks pass; no behaviour change |
| 2 | Mode state machine and admission, USB command entry, no server | Entry refused on battery; refused during media and during a pending handover; exit on USB loss; 5-minute backstop; repeated entry/exit |
| 3 | Server lifecycle, listing, favicon, last-result view; no file transfer | Server start/stop, address display, idle memory, repeated entry/exit; favicon takes no reservation |
| 4 | Small immutable archive download | **Timing gate A**, below |
| 5 | Representative 2 MiB archive | **Timing gate B**, below |
| 6 | `current.log` snapshot | **Timing gate C**, below |
| 7 | Failure paths | Cancel, phone lock and background, hotspot loss, prune conflict, shutdown, USB power loss, deliberate truncated response |
| 8 | Memory and contention | MQTT reconnect inside the mode with the server allocated; maximum admitted clients; repeated reconnect/close/re-entry; server task stack margin |
| 9 | Single-session competition | USB request during an HTTP transfer and the reverse |
| 10 | Server-off regression | Same-sitting Latest/Live/IMU, no new stalls or resets |
| 11 | LVGL entry screen, then the car | CAR build, accepted profile, blank FAT32, retrieval with the car running |

### Firmware timing gates

**Gate A - first small immutable archive.** Record exact bytes, request/start/end monotonic
times, time to first body byte, **maximum no-progress interval**, result and CRC32, queue
high-water and drops, internal free and largest block, and writer and server stack margins.
JP confirms the actual Safari save, and exports that saved file for byte comparison.

**Gate B - representative 2 MiB immutable archive**, separately. Steady-state transfer
behaviour and exported-file integrity. This is where average throughput for the overall
bound comes from.

**Gate C - `current.log` snapshot under ordinary logging.** Paused duration, queue-pressure
early abort behaviour, resume, and zero drops matter as much as elapsed time. **Do not
conclude from a fast archive that a snapshot can safely hold appends paused for 120 s.**

Across all three: **record device send completion separately from phone save completion**,
and never present socket-accepted bytes as a verified phone save. `CURRENT_MS` 120 s and
`STALL_MS` 5 s stay unchanged pending this data - average throughput informs the overall
bound, while stall safety depends on the longest no-progress interval, queue occupancy and
cancellation latency.

## 10. Open items

1. Whether mode entry should **refuse** during a pending handover or **cancel** the pending
   transition. Spec allows either; the choice must be explicit before increment 2.
2. Whether a per-session capability appears in the URL.
3. Exact idle-backstop interaction with a transfer in flight: the backstop is defined on
   request arrival, so a single long transfer cannot expire it. Confirm that is intended.
