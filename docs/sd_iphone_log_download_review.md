# iPhone log retrieval - review against accepted Stage3

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
