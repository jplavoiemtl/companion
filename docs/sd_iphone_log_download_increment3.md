# Increment 3 code handoff - server lifecycle and cached listing

September 22, 2026. Branch iphone-log-retrieval; implementation base d3a8dfe.
JP approved implementation, including the lazy persistent 4096-byte PSRAM worker stack
with internal static TCB. **Ready for Claude code review, not for JP's build/flash yet.**
Increment 4 is unapproved. Historical draft unchanged.

## Implemented behavior

- Main owns OFF/STARTING/ACTIVE/STOPPING. Enter publishes STARTING; only matching worker
  success activates. Exit publishes cancellation without waiting. Media exclusion remains
  until USB release plus HTTP stop/cache cleanup. Startup failure rolls back, stop failure
  retains resources. The worker is never deleted and never falls back silently to internal.
- HTTP descriptors stay on its synchronous task. MSG_DONTWAIT callbacks yield on temporary
  unavailability, keep absolute header/output deadlines and return terminal failure on
  cancellation. Three bounded per-client contexts reset on accept and release via the
  component's transport-context cleanup. Default pending_fn is left null.
- GET / lists managed files and advisory sizes; GET /result shows no HTTP transfer yet.
  GET /favicon.ico returns 204, /f/... returns 503, unknown paths 404, other parsed methods
  including HEAD return 405/Allow: GET. No HTTP file body is served, no auth token, no SD
  access or shared retrieval reservation from an HTTP handler. No-store/Connection: close.
- Listing and result GET admission reset user activity, as does touch. Incidental traffic,
  link recovery and background cache work do not. The expiry decision checks HTTP activity
  and publishes cancellation under one lock so a late request cannot revive STOPPING.
- Writer owns batched inventory scans and two bounded PSRAM arrays. It publishes immutable
  snapshots, preempts scans before USB reader startup, refreshes after rotation/prune and
  at most every five seconds. HTTP unpins after formatting into one 32 KiB PSRAM page,
  before network sends. Cleanup requires matching generation, writer quiescence and no pins.
- USB framing, progress, deadlines, CRC, queue-pressure abort and reader release logic are
  unchanged. The only adapter addition closes an inventory scan before starting a reader.
- Main displays a persistent top-layer stuck notice; matching completed teardown removes
  it and clears the active warning. Serial/status include stage/error, address, worker
  placement/high-water and HTTP high-water. Runtime memory adequacy remains unmeasured.

## Files and ownership boundaries

`src/diagnostics/diagnostics_http.{h,cpp}`: worker, server, callbacks, formatter, top-layer
notice (called only by main). `diagnostics_inventory.{h,cpp}`: writer-owned cache and
HTTP pins. `diagnostics_retrieval.cpp`: asynchronous mode integration. `sd_diagnostics.cpp`:
writer startup/tick/terminal cleanup and mutation notifications. `diagnostics_usb.cpp`:
one writer-only preemption hook. No generated UI or companion.ino edits.

## Installed-SDK finding requiring review attention

The installed esp32s3-libs/3.3.11 IDF 5.5.5 archive closes a rejected open_fn socket in
httpd_sess_new, then again in the inlined accept caller. Source and installed disassembly
were checked, with offsets/hash recorded in the design. Returning ESP_FAIL from open_fn
would expose a numeric-descriptor reuse race against another task.

Implementation installs fail-closed I/O hooks first and returns ESP_OK from open_fn even
on rejection, queueing component-owned httpd_sess_trigger_close instead. Missing context
causes callbacks to fail without socket I/O; inability to queue closure cancels the server.
This is for rejected accepts only, not interruption of a blocked handler. No library patch,
custom close_fn or main-task fd operation. Please scrutinize this refinement and callback
cleanup ordering, including allocation/setup errors and cancellation during registration.

## Validation

All nine Node host suites pass, **172 checks**:

| Suite | Checks |
|---|---:|
| HTTP lifecycle | 36 |
| Retrieval mode | 29 |
| Media admission | 15 |
| Reader/session | 21 |
| USB connection/pacing | 16 |
| USB logger gates | 12 |
| Browser | 19 |
| Network diagnostics | 16 |
| Operation diagnostics | 8 |

Host tests execute source bodies with mocked platform calls, including worker startup
allocation failures, registration rollback, cancellation before/during start, handler/cache
release waits, stop failure retaining resources, partial send/EAGAIN/EOF, absolute deadlines,
maximum 256-entry page and formatter overflow unpinning. Integration assertions cover
routes, stack placement request and writer hooks. Existing assertions were retained;
mock adaptations account for asynchronous startup and the new cache invalidation hook.
These are not a C++ compile, RTOS proof, lwIP timing measurement or hardware result.
`git diff --check` passes. No firmware build, flash, hardware test or measurement performed.

## Review focus / next gate

Review lifetime/lock ordering, SDK signatures, failure rollback, callback close semantics,
cache scan preemption/pins and terminal-writer races, idle activity ordering, and PSRAM
worker startup. All APIs were checked against installed headers; JP will report compile
errors after Claude clears the code. Do not weaken existing host checks to accommodate it.

The first future bench case remains one normal server start/list/favicon/stop cycle from
Safari, with status/memory and unchanged USB reservation. It also verifies the firmware's
first lwIP calls from a PSRAM stack: httpd_start creates listener/control sockets on the
worker. SD writer placement is not evidence for this network property. If start misbehaves,
move worker to an internal stack through review/rebuild rather than debug PSRAM/lwIP.
No bench instructions are issued now; Claude review precedes JP build/flash. companion.ino
is unchanged, so this increment does not require deleting its generated sketch; a clean
build remains fallback for added translation-unit discovery issues. The two increment 2
hardware admission deferrals remain due before car deployment.

## Claude review accepted for first build - September 22

Claude's review at d61e921 found no blocking defects and independently reran 172 checks.
No code change after review. Two non-blocking notes are deliberately deferred: rename
idleExpired to express its decide-and-cancel side effect, and reduce the lifecycle
worker's 100 ms idle polling. Neither changes the correctness of this build; avoid
inserting an unreviewed change before its first hardware case. Revisit naming/polling
with the next reviewed code increment, preserving immediate cancellation notifications.

JP may build/flash amoled-1-8-core-3-3-11 with logging and PSRAM writer enabled, hooks and
USB fixture disabled. companion.ino is unchanged; generated-sketch deletion not required.
Use a clean build only if new translation units are not discovered. Compile errors come
to Codex; no board test until successful build and flash.

### First single bench case issued: normal server start/list/favicon/stop

USB power, existing iPhone hotspot, dashboard idle. Connect the web console with
DTR=true/RTS=false and test switches off. Capture status, log status, log list, then
log mode on. Expect STARTING followed by ACTIVE and a numeric http:// address in LOG HTTP.
Send log mode status. On the hotspot-host iPhone open that address in Safari. If inventory
is pending, wait a few seconds then reload once; verify the file names match the USB list
and sizes are plausible (current size is advisory). Open Last result: no HTTP transfer yet.
No file download links are expected.

On a laptop connected to the same hotspot, check the same server's favicon with
`curl.exe --noproxy "*" --max-time 10 -i "http://<device-IP>/favicon.ico"`.
Expect HTTP 204 and no body. Capture its output. If laptop connectivity is unavailable,
report that limitation; Safari rendering alone does not verify the 204 gate. Return to
console: log status, log mode status (ACTIVE, no USB reservation), log mode off, wait
about two seconds, log mode status (OFF/server=off), status, log status. Download current
via the existing USB console (CRC OK), then log status again for append/unpaused state.

Send the full console, curl output, downloaded current.log and iPhone observations.
No throughput measurement or separate stress case. This also verifies the first lwIP
start on the PSRAM worker. If startup resets/stalls, stop and send the capture; the agreed
response is an internal worker stack through review/rebuild. A monitor-close restart
before the case must be identified separately from a restart on log mode on.
No gate result is recorded yet; increment 4 remains unapproved.

## First hardware attempt and address-family correction - September 22

[Bench evidence](sd_iphone_log_download_bench.md): PSRAM-worker startup and idle teardown
succeed on boot 106, but favicon curl resets without HTTP headers and http_min remains 0.
The initial application address check overlooked the installed dual-stack listener.
Correction in diagnostics_http.cpp accepts only the hotspot IPv4 address, either in
AF_INET or its exact AF_INET6 mapped representation, with a full sockaddr_storage buffer
and length checks. Native IPv6 remains excluded. Bounded per-entry admission counters and
last rejection reason make further diagnosis possible without per-request SD log traffic.

**177 host checks pass**, including 41 HTTP lifecycle checks (five added address cases).
No C++ build/flash or new hardware measurement. Claude quick review is required before JP
rebuilds and repeats the still-incomplete first case. The earlier focus on pasted command
formatting did not explain the verbose curl result; it should not be treated as user error.
Reference: [IDF 5.5.5 listener construction](https://github.com/espressif/esp-idf/blob/v5.5.5/components/esp_http_server/src/httpd_main.c#L327).
