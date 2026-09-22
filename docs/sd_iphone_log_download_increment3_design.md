# Increment 3 design proposal - lifecycle and cached listing

September 22, 2026. Base: `71786fe`, branch `iphone-log-retrieval`.
**For Claude review, then JP approval. No increment 3 implementation is approved.**
Increment 2 is accepted with its two explicitly deferred hardware admission checks.
The historical draft remains verbatim. This proposal does not silently amend the spec.

## Recommended decisions

1. Keep descriptors entirely on the synchronous HTTP task. Replace the provisional
   cross-task shutdown(fd) scheme with cancellable send/receive overrides.
2. A dedicated lifecycle worker calls httpd_start/register/stop. Main and the SD writer
   only exchange bounded commands and status; neither waits for server teardown.
3. Start with max_open_sockets=3, LRU purge off, one retrieval reservation across USB/HTTP.
4. Recommend a random capability per mode entry, shared by the listing and future file
   links. This is a proposed product choice for JP, not an already confirmed decision.
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

Provisional worker allocation: one lazy-created persistent worker, 4096-byte internal
stack, normal low application priority, no affinity. Keep it dormant after OFF; this
retained RAM is explicit and must be included in server-off regression and memory gates.
Creation failure refuses entry cleanly. Avoid task deletion/recreation races and idle-task
reclamation assumptions. HTTP task remains 4096 internal bytes initially. Both stacks need
separate high-water observations; neither size is a measured adequacy claim.

## Startup, cancellation and OFF

Main completes the existing admission checks and enters STARTING before posting START.
It starts the same five-minute deadline then. Worker allocates bounded HTTP/cache state,
starts the server and registers all routes/error handlers. Until main accepts matching
startup success, handlers refuse service without SD work. Only then does main publish
ready and enter ACTIVE. IP display occurs on main; a DHCP change updates the address,
not the mode deadline or capability. Link loss retains ACTIVE/link_down as before.

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

Send Connection: close and enforce one response per socket via the HTTP task's close
path after response completion (handler failure return where supported by the component;
verify it closes without emitting another body). Do not rely on that header alone to
force closure. TCP keepalive and SO_LINGER remain disabled.

## Stuck teardown and visible recovery

Keep STOPPING and media exclusion if handlers, shared release or server stop do not
complete. At 10 s after the first exit request, main latches release_stuck and emits one
RETRIEVAL_STUCK record/status with stage=handler|reader_release|server_stop and
recovery=await_completion_or_reboot. Do not rename a server-stop failure as a writer
failure. A concrete stop API failure is recorded immediately, retaining the handle and
exclusion; no concurrent retry or timeout-based free. Later successful completion clears
exclusion normally. For an unrecoverable returned error, recovery is operator reboot.

Add a main-owned persistent on-screen error notice for stuck teardown using a custom
LVGL object, not generated UI edits; it states that log download mode could not close
and a restart is needed. It must not steal the entire display or invoke network work.
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
never reserves or reads a file. Favicon, status/result display, unknown paths and rejected
methods perform no SD work and do not trigger inventory refresh or reset idle time.

## Socket budget and capability proposal

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

Recommend 128 random bits per accepted mode entry, encoded as 32 hex characters using
the ESP RNG while Wi-Fi is active. Serve the listing at /?k=<capability>; future links
carry the same query parameter. Bad/missing capability gets a cheap 403 and no SD work;
GET /favicon.ico stays unauthenticated 204. Method policy remains 405/Allow: GET before
capability checks; unknown paths stay cheap 404. All responses use no-store; the listing
also uses Referrer-Policy: no-referrer, no external assets, no permissive CORS.
Invalidate on exit, retain across link loss, regenerate on new accepted entry.

Do not put the token in SD records, routine status, access/error logging or review logs.
For the USB-only entry UI in increment 3, add an explicit local 'log mode url' command
that reveals the address to JP; that deliberate credential display is the sole serial
exception and must be redacted from captures sent back. Later show it locally/QR on the
increment 11 UI. JP must approve this interaction. A capability does not encrypt HTTP or
protect against a peer observing traffic; it limits casual access by other hotspot
clients during explicit download mode. Do not add TLS or remote access in this increment.

## Review and validation gates

Before implementation approval, Claude should specifically challenge:

- Whether synchronous descriptor ownership plus cancellable parser/output actually
  removes external shutdown, including stop during incomplete headers and startup.
- Whether MSG_DONTWAIT/error mapping and enforced close match the installed component.
- Worker retention cost, generation/ack ordering, cancellation before startup completion,
  and visibility/recovery when stop returns error or never returns.
- Inventory scan preemption/close ownership, buffer pinning and impact on unchanged USB.
- Capability display exception and provisional socket budget as explicit choices for JP.

Implementation host checks must cover those races and allocation/registration failures,
late completion, stop twice, stop while STARTING, no SD/reservation for incidental paths,
HEAD/method routing, stale inventory, invalid capability, bounded output/error formatting,
partial send/EAGAIN/EOF and cancellation in receive. Preserve all existing checks without
weakening assertions. Host simulations do not prove lwIP scheduling or memory margins.

**First hardware case after reviewed implementation and JP's build/flash:** one normal
start/list/favicon/stop cycle from iPhone Safari on the existing hotspot. Capture status
before entry, ACTIVE and address, listing matching USB managed inventory, favicon 204
checked with a laptop if Safari observation is insufficient, unchanged retrieval
reservation/last-result, OFF after explicit exit, and status/memory afterward. Redact the
capability. No HTTP file download yet. JP receives only this case when the code is ready.
Repeated cycles, incomplete-header cancellation, startup-failure rollback and memory
checks follow separately; this document is not an instruction to run them now.

After Claude resolves objections, fold approved choices into spec sections 5, 7, 9 and 12,
then ask JP for increment 3 implementation approval. The two deferred increment 2 hardware
checks remain due before car deployment and are not closed by this design review.
