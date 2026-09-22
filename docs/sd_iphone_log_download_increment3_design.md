# Increment 3 design proposal - lifecycle and cached listing

Revision 2, September 22, 2026. Reviewed base: `d0483ce`, branch `iphone-log-retrieval`.
**For Claude review, then JP approval. No increment 3 implementation is approved.**
Increment 2 is accepted with its two explicitly deferred hardware admission checks.
The historical draft remains verbatim. This proposal does not silently amend the spec.

## Recommended decisions

1. Keep descriptors entirely on the synchronous HTTP task. Replace the provisional
   cross-task shutdown(fd) scheme with cancellable send/receive overrides.
2. A dedicated lifecycle worker calls httpd_start/register/stop. Main and the SD writer
   only exchange bounded commands and status; neither waits for server teardown.
3. Start with max_open_sockets=3, LRU purge off, one retrieval reservation across USB/HTTP.
4. JP decided no capability token (spec revision 4). Keep the interface and route
   restrictions below. Revisit access protection before file bodies in increment 4.
   Recommend a persistent 4096-byte PSRAM lifecycle-worker stack with internal TCB;
   the worker-stack choice is presented for JP approval below.
5. Implement only startup, cached listing, favicon, empty last-result view and shutdown.
   No HTTP file body, reader transport handoff or new transfer limits in increment 3.

## Evidence and limits

Installed headers under
`C:/Users/photo/AppData/Local/Arduino15/packages/esp32/tools/esp32s3-libs/3.3.11/`
identify **ESP-IDF 5.5.5**, not 5.5.1. The selected sketch profile is
amoled-1-8-core-3-3-11 with PSRAM enabled. Local esp_http_server.h exposes open_fn,
send/receive overrides and internal task_caps. Local lwipopts.h enables socket send and
receive timeouts; sdkconfig.h gives 16 sockets and HTTPD event-post timeout 2000 ms.

Matching upstream source reviewed (not proof of every compiled vendor patch):

- [IDF 5.5.5 lifecycle](https://github.com/espressif/esp-idf/blob/v5.5.5/components/esp_http_server/src/httpd_main.c):
  stop may wait on control capacity and then server termination. It must run off main.
- [IDF 5.5.5 sessions](https://github.com/espressif/esp-idf/blob/v5.5.5/components/esp_http_server/src/httpd_sess.c):
  open_fn can install transport hooks before parsing; synchronous session processing owns
  the socket, and the default session cleanup closes it.
- [IDF 5.5.5 transport](https://github.com/espressif/esp-idf/blob/v5.5.5/components/esp_http_server/src/httpd_txrx.c):
  response output uses the send override, with positive partial sends handled by the
  component's send loop. Header and body accounting must remain distinct.

The proposal's polling intervals are engineering targets, not measured completion
bounds. Network-stack scheduling and component event posting can still delay teardown.
Do not advertise a 500 ms HTTP stop guarantee. diagnosticsClose retains only its existing
bounded caller wait and false-on-timeout contract; no new HTTP wait is added there.

## Ownership and messages

| Owner | Exclusive work | Cross-task interface |
|---|---|---|
| Main | OFF/STARTING/ACTIVE/STOPPING, media admission, activity deadline, UI | Publish desired server generation/state; consume completion snapshot |
| Lifecycle worker | Start, route registration, rollback, stop; owns handle lifetime | Read desired state; publish generation-tagged start/stop/failure completion |
| HTTP task | Session descriptors, parser, synchronous handlers, transport callbacks | Read cancellation/ready snapshot; pin cached listing; publish bounded result/status |
| SD writer | SD operations, inventory refresh, future reader cleanup A | Publish immutable inventory; never wait on HTTP or worker |

Use a short critical section for small POD mailbox fields and cache ownership counts.
No socket API, allocation, SD call, formatting, task wait or logging inside it. Do not use
volatile as synchronization. ESP32 is 32-bit: protect 64-bit generations/timestamps too.
Commands use a desired-state mailbox plus notification, so a STOP cannot be lost behind a
full START queue. Completion is retained until main consumes it; no lossy event queue for
ownership acknowledgements. Diagnostic event persistence remains best effort.

Each accepted mode entry gets a nonzero monotonic lifecycle generation. Never wrap/reuse
it. Future file-session generation is separate. A stop is sticky for that lifecycle;
late start completion cannot reactivate it. Stop completion is acted on only for its
matching lifecycle. No raw httpd_handle_t or fd is published for main to operate on.

### Worker stack - recommendation for JP approval

| Choice | Benefit | Cost / consequence |
|---|---|---|
| Persistent 4096-byte internal stack | Simple lifetime, conventional placement | Retains 4 KiB of scarce internal RAM after first use, plus TCB |
| Per-entry internal task with self-deletion | Releases stack after task reclamation | Requires task-exit/reclamation synchronization before re-entry; publishing completion alone is insufficient |
| **Persistent 4096-byte PSRAM stack, internal static TCB (recommended)** | Simple lifetime without retaining 4 KiB internal | Retains 4 KiB PSRAM and an internal TCB; placement and stack margin must be measured |

Use the same explicit static-task allocation pattern as the existing SD writer,
normal low application priority, no affinity. Create lazily on first entry; allocation
failure refuses entry without an internal-stack fallback. Retain task/stack until reboot,
sleeping on notification while OFF. Installed CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM=1
and the existing working writer support this placement; do not reintroduce the withdrawn
claim that lwIP categorically forbids a PSRAM stack. This is a different task, so the
writer's measured stack margin does not prove its adequacy. No direct flash/NVS work is
added to it. Measure worker high-water mark, stack/TCB placement and internal memory.
HTTP's own stack remains 4096 internal bytes and is measured separately. Approval of
this revised design should explicitly include this recommended worker choice.

## Startup, cancellation and OFF

Main completes the existing admission checks and enters STARTING before posting START.
It starts the same five-minute deadline then. Worker allocates bounded HTTP/cache state,
starts the server and registers all routes/error handlers. Until main accepts matching
startup success, handlers refuse service without SD work. Only then does main publish
ready and enter ACTIVE. IP display occurs on main; a DHCP change updates the address,
not the mode deadline. Link loss retains ACTIVE/link_down as before.

Any exit first clears ready and sets cancellation under the mailbox lock, before posting
STOP. main's logRetrievalExit remains nonwaiting, including during STARTING. In-flight
startup finishes or fails on the worker; if cancellation won, the worker tears down any
created server and never reports an actionable ACTIVE completion. Partial registration
failure takes the same rollback path. OFF is published only after rollback/stop success
and USB reservation release; start failure with no acquired resources may return OFF.

A handler checks ready/cancelled and increments an active-handler count in one locked
step, so there is no check-then-enter gap. After STOP, no new handler is admitted. Each
admitted handler drops all cache pins and future buffer references before decrementing
that count. Worker waits off-main for admitted handlers to leave, then calls httpd_stop;
parser-only sockets need no application-buffer acknowledgement and are closed by the
server. open_fn and transport callbacks reject cancelled generations too. httpd_stop
completion, not a zero handler count alone, proves the server/context can be reclaimed.

No custom close_fn, async HTTP requests or external descriptor close/shutdown in this
increment. Session descriptors never escape HTTP execution. The server's normal cleanup
owns closure. This removes the proposed main-to-handler fd-reuse race by construction.
Worker only invokes the component lifecycle API; it never closes an individual client.

For later transfers: writer cancellation A proceeds independently even if the HTTP task
is stuck. Transport B says the handler no longer accesses retained bytes; matching B is
accepted after invalidation, different generations ignored. Writer alone performs final
shared-session release after A and B. main reaches OFF only after that release and server
stop. Increment 3 must not route HTTP callbacks directly into diagreader::view/progress/
release: those remain writer-only, and the cross-task transfer mailbox belongs to increment 4.

## Cancellable network I/O

### Pending-data check and per-session state

Leave pending_fn unset; do not install httpd_sess_set_pending_override. Contrary to the
review's suggested default-socket concern, the installed library's httpd_sess_pending
only calls a pending function if non-null, otherwise tests parser pending_len. It does
not call recv/ioctl on this path. Plain TCP has no extra TLS/decryption buffer to report.
Socket readability still comes from select. Buffered/readable data may dispatch after
cancellation, but the receive override and authoritative handler admission reject it;
no SD/UI/session mutation is allowed. Cancellation is not enforced by pending_fn.

Use a fixed three-slot internal SessionIo pool, one slot per client socket (not six:
HTTP's three control/listening sockets never use these callbacks). Bound each slot to
96 bytes with a compile-time size check. Store fd, lifecycle generation, unique connection
serial, allocation flag, receive/header/output start timestamps and explicit started/
header-complete flags. Do not index the pool by numeric fd. Only HTTP execution accesses
slots; callbacks read lifecycle cancellation through the synchronized mailbox.

open_fn acquires and zero-initializes a fresh slot on every accept, even if the numeric fd
was just reused. Attach it through httpd_sess_set_transport_ctx with a custom context-free
callback that clears/releases the slot, then install send/receive overrides. The component
owns context cleanup; close_fn stays unset. If setup fails before attachment, release the
slot locally; after attachment leave cleanup to the component exactly once. Pool exhausted,
missing context or fd/generation mismatch fails closed. No shared reader allocation is
involved. Internal sockets and unrelated MQTT sockets never acquire a slot.

First recv attempt starts its initial-wait clock; first positive recv starts the absolute
header clock. Receipt of additional bytes never resets it. At handler entry, verify the
header deadline once more and mark headers complete. First output attempt starts the
absolute output clock, shared across every header/body send. No per-send reset. A
connection serves only one request, so clocks reset only on a new open_fn. Terminal
cancellation/deadline returns HTTPD_SOCK_ERR_FAIL to close, not a parser-retry timeout;
EAGAIN retries remain inside the callback with the same deadlines. Explicit flags handle
timestamp zero correctly. Mode activity uses a separate generation-tagged mailbox.

Install both send and receive overrides from open_fn, before parsing requests. Use
per-call MSG_DONTWAIT (preserving other flags), not a globally nonblocking socket flag
that would change the component's select/parser assumptions. The installed lwIP sockets.h defines MSG_DONTWAIT; confirm its send/receive behavior
in the implementation review and bench, rather than inferring a hard deadline from it.

Each override checks cancellation first, attempts send/recv, returns any positive count
promptly, and on EAGAIN/EWOULDBLOCK waits at most a proposed 20 ms task interval before
rechecking. Yield between retries; never busy-spin. EINTR also rechecks cancellation and
deadlines. Zero receive is EOF; zero send for nonempty data is failure. Other errors map
to the documented HTTPD_SOCK_ERR codes. Do not return zero as a retry signal. No lock is
held while waiting. Callbacks never retain the supplied pointer after returning.

The polling interval is not STALL_MS: a 20 ms wait does not abort a healthy connection.
Use an absolute 5 s request-header budget starting with the first received byte, plus a
5 s no-progress wait for an initial receive attempt. With one response per connection,
that header clock needs no keep-alive reset. For increment 3, small response output gets
an absolute 5 s budget beginning with its first send, in addition to mode cancellation.
Reject request bodies for these GET/HEAD routes and close instead of draining arbitrary
bodies. Later file body output gets its approved transfer/current/stall limits separately;
header bytes and polling must never count as body progress.

Apply cancellation to all output, including cheap errors and favicon, and all input,
including incomplete headers. Otherwise a slow header can trap lifecycle stop before a
handler exists. Register no application HTTP event callback that blocks or accesses SD.
The component's own event-post waits remain a source of latency; benchmark rather than
claim a hard bound from the retry interval alone.

### Enforced close - verified before implementation approval

Send Connection: close, complete the response, release pins/application references, then
return ESP_FAIL deliberately from every response handler (including custom HTTP error
handlers). Record the actual send result separately: deliberate closure is not a failed
application response. On send failure also return ESP_FAIL; never append an error body.
TCP keepalive and SO_LINGER stay disabled. No explicit client shutdown/close is needed.

The installed bundle contains headers and libesp_http_server.a, not the component C
sources. Therefore verification used matching IDF 5.5.5 source **and the installed archive's
disassembly**, rather than claiming an unavailable local source file was inspected:

- httpd_uri propagates nonzero handler return as ESP_FAIL without a second response.
- httpd_req_new returns that result after request cleanup; httpd_sess_process propagates it.
- httpd_process_session invokes httpd_sess_delete on that failure; default deletion closes
  the socket and frees transport context. No custom close callback is installed.

References: [URI dispatch](https://github.com/espressif/esp-idf/blob/v5.5.5/components/esp_http_server/src/httpd_uri.c#L336),
[parser](https://github.com/espressif/esp-idf/blob/v5.5.5/components/esp_http_server/src/httpd_parse.c#L628),
[session cleanup](https://github.com/espressif/esp-idf/blob/v5.5.5/components/esp_http_server/src/httpd_sess.c#L339).
Installed archive SHA256:
`01258A9E813A8FDBCEDF285991264F07EF583CB194B63F642F7E80AC2A9A6E3F`.
Read-only verification used xtensa-esp32s3-elf-objdump -dr --disassemble=<function>.
In that archive: httpd_uri handler call at +0x95 returns through +0x98..0xa0;
httpd_req_new calls httpd_uri at +0x2d3 then takes cleanup/return;
httpd_sess_process branches on failure at +0x25;
httpd_process_session calls httpd_sess_delete at +0x4d.

A second request on the same connection, including pipelined bytes, is not served.
A second connection can be accepted/queued within the three-client budget and receives
its reply when the synchronous HTTP task is available. This specifies the spec's queued
second-request behavior precisely; it does not introduce concurrent handlers. Bench still
checks actual browser completion and single-response closure; source/binary inspection
is not phone-save evidence. Component warning text for deliberate handler failure may
occur; do not misclassify it as a failed file/result record.

## Stuck teardown and visible recovery

Keep STOPPING and media exclusion if handlers, shared release or server stop do not
complete. At 10 s after the first exit request, main latches release_stuck and emits one
RETRIEVAL_STUCK record/status with stage=handler|reader_release|server_stop and
recovery=await_completion_or_reboot. Do not rename a server-stop failure as a writer
failure. A concrete stop API failure is recorded immediately, retaining the handle and
exclusion; no concurrent retry or timeout-based free. Later successful completion clears
exclusion normally. For an unrecoverable returned error, recovery is operator reboot.

Add a main-owned persistent on-screen error notice for stuck teardown using a custom
LVGL object parented to lv_layer_top(), not the currently active screen or generated UI.
Main creates it once with a child label: download mode could not close; waiting, restart
if it persists. It survives screen loads, does not capture touches outside its small
notice area and invokes no network work. Main deletes it and nulls the pointer only on
a matching late successful teardown reaching OFF; clear the active release_stuck flag
then while retaining the historical diagnostic event. Status queries, repeated off/on,
link recovery and screen navigation do not clear it. A terminal failure needs reboot.
If UI allocation fails, retain serial/status/event escalation and retry UI creation at
most once per second while stuck; absence of a label never permits resource release.
This is needed now so failure is visible without a serial status command; the full entry
screen still belongs to increment 11. No automatic reboot and no wait added to the power/
deep-sleep path. Event loss must not erase the retained status or UI indication.

## Cached listing without a file transfer

Add a dedicated writer-owned inventory cache, not a synthetic USB list command and not
pointers into diagreader::view(). Reuse managed-name validation and the existing 256-entry
limit. Two bounded PSRAM arrays hold published/staging snapshots; HTTP pins the published
array only while copying/formatting a bounded response. Writer never overwrites a pinned
array; if staging is unavailable it skips that refresh rather than waiting.

Refresh requested on entry, after writer-observed rotation/prune, and at most every 5 s
while ACTIVE and no retrieval reservation/close is pending. Writer performs bounded
batches (at most eight directory entries per tick), services normal queued logging first,
and closes the directory on cancellation/close before acknowledging cache cleanup.
The filesystem calls themselves still have the existing SD stall limitation. A new USB
reservation preempts the cache scan before the USB adapter starts it; no append pause or
shared retrieval reservation is taken for listing. Stage a complete snapshot and publish
atomically; errors retain the prior snapshot with stale/error metadata. A first missing
snapshot displays 'inventory pending', not an invented empty card. Resource ownership
must also survive startup cancellation while an inventory refresh is in progress. Writer
publishes a generation-matched cache-quiescent acknowledgement after closing any scan.
Cache arrays can be freed only after that acknowledgement, zero cache pins and server
stop completion; main cannot report OFF earlier. If the SD writer has terminated before
accepting a refresh, no scan may start and the cancelled pending request is acknowledged
without SD access. If it terminates with owned scan state, its terminal cleanup must close
that state before publishing quiescence. Neither lifecycle worker nor HTTP closes it.

Page shows inventory timestamp, stale/busy state and 'no HTTP transfer yet'. Every listed
size is advisory, not a frozen current.log transfer size. Mark stale during retrieval,
after known storage mutations, refresh error, or cache age over 5 s. Generation is the
managed identity; stale archive links must never resolve to a newer archive. In increment
3 render entries as text without download links; /f/... returns 503 not_implemented and
never reserves or reads a file. All views are SD-free on the HTTP task.

### Listing output storage

Allocate one 32768-byte PSRAM response buffer during startup, explicit
MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT, with no internal fallback. Only the synchronous HTTP
task formats/uses it; no buffer on its 4096-byte stack and no per-request growth. Pin one
inventory snapshot during formatting, unpin before any network send. Limit the page to
256 managed entries at at most 112 formatted bytes each, plus at most 2048 bytes for
fixed markup/status: 30720 bytes, below capacity including NUL. Managed names are
fixed-format ASCII, sizes/IDs bounded decimal; never insert raw path/error/request text.
Every append checks remaining capacity, and overflow before headers yields a fixed 500
response rather than a truncated page. Build-time format-size assertions and boundary
host checks cover maximum values and count. Last-result view uses the same buffer,
showing no HTTP transfer yet in this increment.

Send a single non-chunked text/html response with its computed Content-Length using
httpd_resp_send. The 5 s output deadline covers headers and body together. PSRAM remains
pinned by server ownership until synchronous send returns, and is freed only after
handler quiescence and successful server stop. Component temporary headers/session/parser
allocations remain internal and must fit the unchanged 20480-byte largest-block gate;
PSRAM output allocation does not remove that measurement requirement.

### User activity - JP's revision 4 decision

An admitted GET / or GET /result posts one generation-tagged user-activity timestamp at
request admission; main applies max(current activity, timestamp) only for the still-active
matching generation. Panel touch also resets. Future accepted file-transfer arrival
resets it once; body progress does not repeatedly reset it, preserving idle expiry during
long transfers. Favicon, unknown paths, rejected methods, refused /f/... in increment 3,
startup/stopping refusals and background refresh do not reset it. No auto-refresh, polling
or prefetch script is included in the page. Process activity before main's expiry decision
using a synchronized snapshot; once STOPPING wins, late activity cannot revive the mode.
Loading the last-result view explicitly counts as activity and does not trigger SD work.

## Socket budget and JP's no-token decision

Use max_open_sockets=3, lru_purge_enable=false, default header/URI limits 1024/512,
HTTP port 80, default control port, one synchronous HTTP task. This provisions three
client sessions, not three file readers; second requests still queue behind a handler.
The installed socket ceiling is 16 and the component reserves three internal sockets.
Thus HTTP can account for six socket descriptors, leaving ten nominally for other users;
this is capacity arithmetic, not a claim that all ten are free or that PCB/RAM costs are
six times a fixed buffer. Measure MQTT/DNS/recovery and occupied-client cases in increment
8, including idle clients. Keep the unchanged 20480-byte largest internal block gate.
Do not derive total internal RAM solely from TCP send/window constants: count the two
stacks, component/parser/session allocations, pbufs, control resources and cache metadata.

JP has decided no capability token. Serve / and /result directly, with no RNG dependency,
auth query, 403 authorization path, credential display command or redaction exception.
Print the ordinary http://<STA-address>/ on successful entry and in mode status. All
responses use no-store. No upload/delete, arbitrary filesystem paths, external assets,
permissive CORS, TLS or remote-access feature. Managed IDs only when bodies are added.
Revisit access protection before increment 4 serves logs, as spec revision 4 requires.

Interface restriction: the installed httpd_config_t has no bind-interface field. Do not
claim a literal listener bind that this API cannot provide. On this STA-only build,
require STA mode with no SoftAP/other active data interface before startup; in open_fn
verify getsockname's local IPv4 address equals the current hotspot STA address before
installing context/overrides. Reject mismatches and link-down accepts. This restricts
application service to the hotspot interface, though the component listener uses ANY.
If another interface becomes enabled, request teardown rather than serve it. Reject
IPv6 in this version (numeric IPv4 URL). This explicit implementation interpretation of
'bound to the hotspot interface' requires review; no custom component patch is proposed.

## Review and validation gates

Before implementation approval, Claude should specifically challenge:

- Whether synchronous descriptor ownership plus cancellable parser/output actually
  removes external shutdown, including stop during incomplete headers and startup.
- Whether MSG_DONTWAIT/error mapping and enforced close match the installed component.
- Worker retention cost, generation/ack ordering, cancellation before startup completion,
  and visibility/recovery when stop returns error or never returns.
- Inventory scan preemption/close ownership, buffer pinning and impact on unchanged USB.
- PSRAM worker recommendation and the explicit interface restriction described above.

Implementation host checks must cover those races and allocation/registration failures,
late completion, stop twice, stop while STARTING, no SD/reservation for incidental paths,
HEAD/method routing, stale inventory, three-slot context reuse, bounded output formatting,
partial send/EAGAIN/EOF and cancellation in receive. Preserve all existing checks without
weakening assertions. Host simulations do not prove lwIP scheduling or memory margins.

**First hardware case after reviewed implementation and JP's build/flash:** one normal
start/list/favicon/stop cycle from iPhone Safari on the existing hotspot. Capture status
before entry, ACTIVE and address, listing matching USB managed inventory, favicon 204
checked with a laptop if Safari observation is insufficient, unchanged retrieval
reservation/last-result, OFF after explicit exit, and status/memory afterward.
No HTTP file download yet. JP receives only this case when the code is ready.
Repeated cycles, incomplete-header cancellation, startup-failure rollback and memory
checks follow separately; this document is not an instruction to run them now.

After Claude resolves objections, fold approved choices into spec sections 5, 7, 9 and 12,
then ask JP for increment 3 implementation approval. The two deferred increment 2 hardware
checks remain due before car deployment and are not closed by this design review.

## Revision 2 disposition

D1/D2: JP's revision 4 idle and no-token decisions applied throughout. D3: pending default
verified safe for plain TCP. D4: bounded session pool/reset/free rules specified. D5:
32 KiB PSRAM, full bounded non-chunked listing specified. D6: top-layer notice and matching
late-completion clear rule specified. D7: close path verified in matching source and the
installed binary before approval. D8: three worker choices presented; persistent PSRAM
is recommended and needs JP's approval. No firmware, build or flash changes.
