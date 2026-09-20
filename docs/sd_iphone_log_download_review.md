# iPhone log retrieval - review against accepted Stage3

## September20 follow-up: review of Claude at4cbe974

Scope: iphone-log-retrieval branch, clean before review. Historical draft is now
tracked and remains unchanged. This addendum supersedes earlier statements here
that called it untracked or left architecture/power choices undecided. JP's
recorded decisions stand, including esp_http_server, USB-power-only admission,
immediate abort on detected power loss, hotspot-loss mode retention, five-minute
idle reset by HTTP request or touch, and no motion-based exit. No firmware/build/
flash changes. Findings below concern the proposed implementation, not deployed bugs.

### P1: shutdown proposal does not yet meet the cleanup deadline

Claude lines295-299: lowering send_wait_timeout or queuing session close is not
by itself a500ms guarantee. Installed esp_http_server.h:201-202 uses uint16_t
whole seconds. A positive value cannot express a sub500ms timeout. Listening
socket closure does not interrupt an accepted client socket. IDF5.5.5
httpd_sess_trigger_close queues work on the HTTP task; httpd_stop queues shutdown
and waits for the task to stop. A blocked synchronous handler delays both.
Default5s socket-call timeouts also do not implement the writer's progress-based
STALL_MS or overall CURRENT_MS.

Specify independent writer cancellation/resume, bounded handoff ownership and
safe accepted-socket interruption. Never call blocking httpd_stop from the main
loop's power transition/close path. Keep mode STARTING/STOPPING exclusive to media
until resources are released; do not expose normal media while teardown still
holds memory. Publish cancellation from main; execute SD cleanup only on writer.
The current static finishError directly closes/reopens SD, so power handling
cannot literally call it from updatePowerStatus. Demonstrate prompt main/UI and
writer cleanup with a client that stops reading, including power-off and exit.
A synchronous streaming handler also delays favicon/second-request handling;
choose bounded yielding/asynchronous servicing or document/test delayed replies.

### P1: reverse admission misses a pending motion handover

The five external trigger paths listed are complete in current production source.
But image_fetcher.cpp:591-597 marks HTTP_COMPLETE and requestInProgress=false
while imageDisplayTimeoutActive remains true. imageFetcherIsBusy():228-230 then
returns false even when motionTriggered will call videoStreamStart at:324-332.
A bench USB mode entry during that interval passes all proposed media checks.
The future Live guard prevents network activity, but the caller still executes
returnToPreviousScreen on refusal, changing UI during download mode. A regular
still also has an outstanding automatic return timer.

Choose a defined transition: reject entry while a motion handover/display return
is pending, or deliberately leave/cancel the media screen on the main task before
entry. Do not change imageFetcherIsBusy globally without reviewing MQTT retry
semantics that use it. Add a behavioral case: successful remote still, enter mode
before handover, verify mode/screen/network result and no surprise navigation.

### P2: bool prepareForRequest is safe only with lifecycle ordering fixed

There are two live callers, plus the static forward declaration at:143. Return
bool is not an ABI problem, and the normal still/Live flow need not change. But
both callers currently call imageBegin FIRST (:648/:693). A late refusal inside
prepareForRequest would leave a fresh IMAGE_BEGIN without its end; imageBegin
also terminates the previous image as replaced. Simply adding if(!prepare...)return
is therefore not a complete backstop contract.

Check admission before lifecycle mutation, or explicitly finish any lifecycle
created before a refusal. Preserve total timing including UI prepare and preserve
motionTriggered assignment AFTER preparation, which clears it. Direct Live does
not call prepareForRequest; a second duplicate guard inside videoStreamStart is
not independent protection. Put one authoritative guard before all side effects.
A source call-site check should exclude comments and cover the whole production
tree, including requestImage dispatch; it cannot prove these runtime invariants.
Add allowed/refused behavior checks: no pending endpoint/UI/buffer mutation, no
unpaired begin/end, normal latest/history and motion handover still work.

### P2: internal-memory estimate and benchmark provenance need correction

Installed sdkconfig values5744/5760/1436 and16 sockets are correct; the inference
that unset SPIRAM_TRY_ALLOCATE_WIFI_LWIP means all lwIP buffers are internal is not.
lwipopts.h:1708-1713 maps the unset branch to ordinary malloc/calloc. This bundle
sets SPIRAM_USE_MALLOC and ALWAYSINTERNAL4096; Arduino esp32-hal-psram.c:103-104
enables external-memory malloc when its conditional applies. Inspect allocation
size/caps; do not infer placement solely from the unset preference flag.

TCP send/window values are capacity limits, not a complete reserved-memory
account. Three fully occupied send budgets alone are17232 bytes before4096
stack, receive/pbuf/netconn/PCB/mailbox/HTTP state, headers and handoff buffers.
That arithmetic is not an internal-heap prediction either: allocation is dynamic.
10-16KB may describe one workload but is not justified as a bound. max_open_sockets
counts clients; HTTPD uses three additional infrastructure sockets. Small
individual allocations can fragment the contiguous block; none exceeding20480
is no assurance that20480 remains free.

Claude lines355-359 mix baselines:95688/57332 are from logging-OFF console at
10:11:13, and heap_min_boot is a historical minimum, not current idle free.
Latest accepted logging-on evidence records writer margin3096, not a lower bound
of3144. Keep the MQTT-reconnect gate, measuring server idle, active/slow transfer,
maximum admitted clients and repeated reconnect/close/re-entry. Measure actual
internal free/largest and server stack margin instead of accepting the estimate.

### P2: separate task is not crash containment

External-stack/task-create flags and the internal-TCB requirement are confirmed;
I find no blanket socket prohibition justified by the inspected configuration.
That is not blanket certification of every cache-off path. A separate HTTP task
is still a good way to keep socket stack use and blocking out of the writer.
But Claude:279 says that if the server task dies logging survives. The installed
configuration enables stack-canary checking; IDF5.5.5's Xtensa overflow hook calls
esp_system_abort. A server stack overflow can stop/reset the whole device. Claim
scheduling/stack separation, not process-like fault isolation; test stack margin.

### P2: four-layer verification needs snapshot identity and honest results

JP's size-only iPhone check and laptop CRC gate remain accepted. Add transfer ID
(or boot+monotonic request number) to the filename, end record and last-result
snapshot: boot+size is not a unique download ID. Preserve expected size, actual
bytes accepted by transport, CRC coverage and result together atomically. Count
CRC exactly once despite partial sends/retries; distinguish successful send from
browser save. HTTP_GET_END cannot be in its own current snapshot and may not be
persisted on shutdown/SD failure, so remove the word 'always' for its availability.

A second curl request for current.log is a DIFFERENT snapshot, changed at least
by retrieval records. Compare each curl body to its own transfer's expected
size/CRC and the corresponding physical-card PREFIX after safe close; full-file
comparison is appropriate for immutable archives. Last-result refresh must be
SD-free and must not overwrite the download result with favicon/index requests.
Test exact-byte visibility in iOS Files: a rounded display is not proof of size
equality. Export one actual Safari-saved archive to the laptop for byte comparison
at the bench; this tests the iPhone save path without adding client crypto.

### Power findings and additional missing gates

allowSleep assignments/normal inactivity guard and disabled TEST_POWER checks
match the source (apart from the harmless global initialization). USB presence
closes the normal inactivity case; no battery keep-awake feature is needed.
'Immediate' power loss means after the cached PMIC observation: background polling
is200ms and I2C lock/read or synchronous work can delay it. Likewise Wi-Fi event
callbacks/HTTP requests must post to the main owner rather than mutating a plain
main-task flag/timer from other tasks. Model flag/timer publication explicitly.

Keep five minutes as decided. HTTP/touch resets do not detect reading a static
page without touching it. Do not promise that case cannot expire; do not add
automatic refresh that defeats the idle policy. Define idle expiry during a
slow archive transfer and hotspot downtime; GOT_IP alone must not silently reset
the ceiling. Reject entry when closing, not only usbStatus.ready (closing is a
separate field), and unwind partial server-start failures.

Additional one-at-a-time gates: partial-header/slow-reading clients; sockets
occupied by idle keep-alive clients with LRU disabled; accept failure/resource
exhaustion with MQTT reconnect; STOPPING/re-entry and stale completion after
hotspot loss; pending motion handover; guard refusal lifecycle; five-minute
expiry with/without requests/touch and across hotspot recovery; actual iPhone
saved-byte comparison and same-boot repeated filenames. Hotspot loss should
cancel promptly, not wait to accumulate8 events; prove queue-pressure abort
separately. Preserve the selected keep-mode-open/GOT_IP behavior.

### Defaults checked and references

Installed esp32s3-libs/3.3.11 reports IDF5.5.5. HTTPD defaults verified:4096-byte
internal/8-bit stack, unpinned,7 client sockets, LRU=false, send/recv5s,
header limit1024 and URI512. These facts support the component choice; they
do not discharge application cancellation, admission or memory gates.

Local references: sdkconfig; include/esp_http_server/include/esp_http_server.h;
include/freertos/esp_additions/include/freertos/idf_additions.h:259-266;
include/lwip/port/include/lwipopts.h:1708-1713, all under the installed
Arduino15/packages/esp32/tools/esp32s3-libs/3.3.11 bundle. Arduino3.3.11
cores/esp32/esp32-hal-psram.c:103-104 supplies the malloc integration.
Upstream implementation reference (matching IDF version, not a rebuilt binary):
- https://github.com/espressif/esp-idf/blob/v5.5.5/components/esp_http_server/src/httpd_main.c
- https://github.com/espressif/esp-idf/blob/v5.5.5/components/esp_http_server/src/httpd_sess.c
- https://github.com/espressif/esp-idf/blob/v5.5.5/components/freertos/FreeRTOS-Kernel/portable/xtensa/port.c

No tests/build/flash performed: review only. Historical draft and Claude review
are unchanged. The findings refine the settled architecture, not reopen it.

Earlier review below is historical where superseded.

Date: September20,2026. Reviewer: Codex. Status: design review, not implementation.
Original Claude draft: [sd_iphone_log_download_plan.md](sd_iphone_log_download_plan.md).
That untracked draft is unchanged. Stage3 is accepted; the original reference
to a future stage after Stage1B is stale. Wireless retrieval is now a prerequisite
to Stage4 car evidence collection, developed and accepted on the bench first.

## Agreed direction from JP

Use the companion bench unit for development and simulation. Add an iPhone
log-download feature, fully bench-test the agreed scope, then install the accepted
firmware and a blank FAT32 SD card in the car unit. Real trip/incident logs will
be retrieved on the phone. The previous USB-after-trip procedure is superseded.
No firmware edits or build/flash are authorized by this review alone; design
and Claude feedback come first. JP still performs all builds/flashes.

## Recommended corrections to the draft

1. Treat iPhone Safari access to a hotspot client as an empirical prerequisite,
   not a proven consequence of being on the same network. Test on JP's actual
   phone/iOS. Laptop HTTP validates that path at that time; companion firmware
   must still pass its own test. Serve only a dedicated scratch directory with
   a harmless sample; do not expose the repository with a default directory server.
   Any firewall rule should be narrow/temporary; do not disable the firewall.
2. Preserve SD single ownership. diagnostics_usb.cpp currently owns file listing,
   reading, CRC, snapshot and abort state inside a USB-specific state machine.
   Its hooks in sd_diagnostics.cpp include USB-specific event names. Reuse the
   proven semantics by extracting a small shared reader/session service, with
   USB regression checks; do not invoke the USB parser from HTTP or open SD
   handles from the network/main task. One retrieval session total across USB
   and HTTP; an explicit busy response handles competition.
3. Separate SD progress from network blocking. Recommend evaluating IDF
   esp_http_server with one bounded transfer and a bounded buffer handoff to
   the writer, since a synchronous main-loop send risks touch/IMU/MQTT delay.
   This is a recommendation pending memory measurement, not a settled choice.
   A server task itself costs internal memory; plain HTTP does not make that
   cost negligible. Do not move socket sends into the SD writer. Define buffer
   ownership, cancellation, request generation IDs, late completion handling,
   and task stop ordering before implementing. No whole-file ESP32 allocation.
4. current.log must be a fixed snapshot. Preserve flush/close, bounded read,
   reopen/drain, queue-pressure abort at50%, current limit120s, stall bound5s
   unless separately justified. Every cancellation, network loss, send failure,
   allocation failure and timeout must release the reader and restore logging.
   HTTP socket blocking must not delay writer cleanup or the500ms close budget.
   Abort reader before pruning its archive; shutdown takes precedence over HTTP.
5. Define media admission in BOTH directions, including MQTT remote images and
   motion handover. Refusing downloads only when media is already active leaves
   the reverse race open. Recommended first version: explicit download mode,
   refuse entry during media; refuse new media while mode is active, visibly
   explain busy and log the decision. Keep normal network recovery and logging.
   JP explicitly selected this dedicated download mode; cover all media entry paths.
6. Define power behavior: mode idle timeout, active transfer cap, user cancel,
   USB-loss grace, inactivity sleep, physical power-off and Wi-Fi loss. Do not
   promise indefinite keep-awake or silently override normal shutdown. Clarify
   whether parked retrieval occurs with vehicle power present or on battery.
7. Content-Length detects incomplete HTTP bodies but TCP alone is not an
   application-level file verification claim. Use binary HTTP and attachment
   filenames, cache-control:no-store, and test saving/sharing from Files on the
   actual iPhone. Keep independently verified size/CRC or hash as a bench gate.
   If the UI says verified, design a matching snapshot checksum protocol and
   client check; do not equate socket-send completion with saved-to-Files success.
   Do not assume a streaming CRC can be placed in headers before it is known.
   Post-header aborts close an incomplete response; never append error text to
   a log and present it as successful. Defer resume/range until semantics exist.
8. Start with list plus one-file download. Single current+recent bundle is a
   desirable follow-up, but Safari multiple downloads/Files behavior must be
   measured. Do not assume desktop sequential-save behavior transfers unchanged.
   Avoid ZIP-on-ESP32 as an unmeasured memory/CPU addition to version1.
9. Local HTTP is plausible on a trusted hotspot, but password protection does
   not authorize every other connected client and redacted logs still disclose
   operational metadata. Define intended access: physical enable, short lifetime,
   managed-file IDs only, no arbitrary paths/uploads/deletes, no permissive CORS.
   Consider a per-session capability in the displayed URL; never log it. That
   limits accidental access, not confidentiality against a hostile local network.
10. No automatic AP fallback, new Wi-Fi policy, mDNS dependency, cloud relay or
    retention changes in the first version unless JP explicitly selects them.
    JP selected hotspot-only version1; offline fallback is out of scope. A hotspot
    failure must recover before retrieval, or logs wait for later access. Numeric
    address display is a reasonable starting point.

## Bench gates, executed one case at a time after design review

- Actual iPhone hotspot-to-laptop reachability and harmless file save/share.
- Companion server start/stop and address; idle costs and repeated entry/exit.
- Managed file list and small immutable archive download verified byte-for-byte.
- current snapshot and continued logging; larger files and slow receiver.
- Cancel, Safari background/lock, Wi-Fi loss, reconnect and repeat download.
- Queue-pressure abort, prune/rotation conflict, SD errors, shutdown and power
  transitions; cleanup without drops or stuck paused logger.
- Remote/local media admission, network reconnect/TLS during download mode,
  simultaneous USB request rejection and existing USB behavior regression.
- Repeated sessions: internal memory/stack recovery, no accumulating resources.
  Retain20480-byte TLS largest-block gate and zero normal queue drops.
- Server off: same-sitting Latest/Live/IMU regression, no new stalls/resets;
  accepted IMU-rate investigation and writer placement remain closed topics.
- Before car deployment, confirm CAR build configuration, accepted profile and
  hardware match, fresh FAT32 startup/creation, and successful iPhone retrieval
  from the car unit while parked. A bench pass alone does not verify installation.

## JP decisions and remaining details

JP uses an iPhone13 Pro already working with the companion for MQTT. He selected
an explicit download mode that refuses local and remote Latest/Live while
logging and normal connection recovery continue. Version1 uses only the existing
Personal Hotspot; no disconnected-hotspot fallback. These are confirmed choices.
Exact iOS version is not yet supplied; record it with the first reachability test.
Working MQTT does not prove phone-browser-to-companion-server reachability.
Settle power availability and preferred screen entry/file-sharing workflow during
UI design, without blocking initial network proof on cosmetic decisions.

## Claude review handoff

Please review this document and your original draft against the current
sd-diagnostics branch, accepted Stage3, diagnostics_usb.cpp/.h,
sd_diagnostics.cpp/.h and diagnostics_operation.cpp. Do not implement yet.
Prioritize single-writer safety, bounded transfer cleanup and power handling,
USB regression risk, memory/task costs, bidirectional media arbitration,
iPhone hotspot/Files assumptions and file-integrity semantics. Identify concrete
blockers, preferable simpler designs and missing bench gates. Distinguish
verified source behavior from platform assumptions. Preserve JP's one-case-at-
a-time bench workflow and no assistant builds/flashes. Optional tail stays deferred.
This is a handoff prepared for JP; no Claude review has yet been performed.

## External reference checks

Apple documents Safari downloads in Files, but this does not establish hotspot
client reachability or the exact behavior of our HTTP response on JP's iPhone:
https://support.apple.com/en-ke/102440

Espressif documents server task configuration, socket limits and send/receive
timeouts; check the installed IDF5.5.x headers before fixing implementation values:
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/protocols/esp_http_server.html
