# Codex review: SD log retrieval over USB

Date: 2026-09-15. Review only; no firmware changes or implementation approval.

Reviewed [USB retrieval plan](sd_usb_log_retrieval_plan.md), its
[input brief](ESP32_SD_Log_USB_Investigation.md), and the existing
[SD diagnostics plan](sd_diagnostics_plan.md).

## Recommendation

Keep Option A: the existing USB serial port and a small Web Serial page. Correct the
pacing, delivery detection, current-file handling, pruning and shutdown rules before
folding this proposal into the main SD diagnostics plan. The connection experiment and
Stage 1B bench gate remain necessary; source inspection does not establish bench results.

## 1. Current-build facts

Checked the local `sketch.yaml`, generated build options, firmware, installed Arduino
ESP32 core 3.1.3, and generated and installed SDK configurations. The two SDK
configurations have identical hashes (see evidence appendix).

| ID | Claim | Outcome and evidence |
|----|-------|----------------------|
| F1 | Hardware USB Serial/JTAG, not TinyUSB | Agree. Profile selects `USBMode=hwcdc`, `CDCOnBoot=cdc`; firmware defines `HWCDC USBSerial`. No USB mode change is needed for A. |
| F2 | Baud rate ignored | Refine. `HWCDC::begin(baud)` does not use its argument. Use 115200 for compatibility; “any baud rate” unnecessarily assumes Windows accepts every value. |
| F3 | One write protects an entire line | Refine. The TX mutex protects against other callers of `HWCDC::write()`. Low-level console and panic output may bypass that mutex. |
| F4 | Other writers wait 100 ms, then lose their print | Agree with scope. Mutex acquisition failure returns zero. This is not a universal 100 ms bound on a complete write. |
| F5 | A 256-byte buffer and a space check make writes nonblocking | Disagree with the consequence. The default is 256 bytes. `availableForWrite()` itself takes the mutex with a timeout; another writer can consume space between that check and the next write. |
| F6 | Host stalls always produce short writes and dropped remainders | Refine substantially. Disconnected writes use `flushTXBuffer()`, which can discard queued bytes while `write()` returns the requested length. A full return value does not establish delivery. |
| F7 | FatFs file locking is disabled | Agree: `CONFIG_FATFS_FS_LOCK=0`. Enabling it would reject prohibited duplicate opens, not make concurrent append and read handles safe. |
| F8 | Two CDC ports and MSC require TinyUSB | Agree for this board's existing USB connection. SDK flags and board menu confirm capability; these interfaces are not active in the current build. |

## 2. Alternative USB options

**B1 — Two CDC ports: retain rejection, soften the explanation.** Core 3.1.3 already
supports `USBCDC(0)` and `USBCDC(1)` with descriptors. A custom composite USB stack is
not required. Nevertheless, migrating the existing console, testing uploads and losing
the current hardware JTAG path makes this larger than A.

Say a broken firmware USB stack *may require* manual bootloader recovery. Distinguish
application TinyUSB output from ROM USB output rather than claiming all early and panic
USB output is impossible. Espressif's ROM USB console is also distinct from TinyUSB;
its capabilities should not be attributed directly to this application's TinyUSB port.
See [Espressif USB console documentation](https://docs.espressif.com/projects/esp-idf/en/v5.1/esp32s3/api-guides/usb-otg-console.html).

**C1 — Mass storage: retain rejection, acknowledge read-only MSC.** Core provides
`USBMSC::isWritable(false)`, which removes the Windows-write concern. Exposing a changing
FAT volume remains inconsistent even to a read-only host. Freezing card changes or
providing a snapshot still complicates ongoing logging. “Highest corruption risk”
applies primarily to writable shared access, not every MSC implementation.

## 3. Design corrections before integration

**D1 — Commands.** Good scope. The existing buffer holds 23 characters plus its
terminator, and the proposed commands fit. Validate generation range and trailing garbage.
Reconcile `log info` with the main plan's `log status`. Specify busy handling for listing
and tail, as well as a second download.

**D2 — Framing.** Keep base64, sequence numbers and a single write containing both leading
and trailing newline. Define protocol version, exact CRC variant, maximum line length
and strict decoding. Give transfers an identifier so delayed replies cannot contaminate
retries. Browser reads can split anywhere; a read result is not necessarily a line.

**D3 — Pacing.** Retain the space check as a best-effort optimization. Remove “never
waits” and “debug prints are never delayed or dropped.” Bound work per writer turn and
measure contention. Avoid changing the global USB timeout merely to make the claim true.

**D4 — Browser disappearance.** USB buffering and successful writes cannot reliably prove
that the page is still consuming data. Add a small periodic application acknowledgment
with a deadline, or explicitly weaken the five-second detection claim. Do not add
per-line acknowledgments. Account for the existing serial parser waiting behind blocking
main-loop work when choosing and testing that deadline.

**D5 — Writer ownership, pruning and shutdown.** Keeping all SD access in the writer is
correct. Protect an archive being downloaded from pruning. If retention needs that file
removed, abort and close its reader first. Shutdown must preempt transfers and restore
logging or complete the existing close sequence without waiting to send a serial error.

**D6 — Downloading current.log.** Pausing appends alone leaves the duplicate-handle issue
unresolved. Flush and close the append handle, open for reading, then close and reopen
for append on every completion and abort path. Handle reopen failure explicitly.
FatFs prohibits duplicate opens involving write access; see
[FatFs f_open documentation](https://elm-chan.org/fsw/ff/doc/open.html).

**D7 — Queue safety.** Define the abort threshold with reserved headroom. Check it between
chunks and while USB is stalled. A high-water abort should prevent event loss, not excuse
it. Record transfer completion after appends resume. Account for the longer power-loss
exposure while events remain queued instead of being written and flushed.

**D8 — Version 1 scope.** Defer “current + newest 3” until single-file retrieval passes.
Bound browser console history. Replace “costs nothing in the car” with “inactive, with
bounded memory and polling overhead.”

## 4. Stage 1B test refinements

- **T1 — Integrity:** compare downloaded `current.log` with the captured-length prefix
  of the later card-reader copy. Resumed logging changes the complete file.
- **T2 — Faults:** add forced browser-read stall, corrupted or missing protocol data,
  rotation and pruning during retrieval, SD read failure, shutdown during retrieval,
  and repeated-download memory and handle checks.
- **T3 — Resources:** preserve same-session heap, largest-block and writer-stack checks.
- **T4 — UI:** require no additional stalls beyond existing MQTT blocking. “Touch
  responsive during reconnect” contradicts the accepted firmware behavior.
- **T5 — Throughput:** measure decoded-file throughput separately from serial throughput.
  At 100,000 serial bytes per second, framing puts a 2 MiB transfer nearer 30 seconds
  than the base64-only estimate of 28 seconds. Actual throughput is unmeasured.

Keep the existing integrity, normal debug output, disconnect and abort, queue-pressure,
and return-to-VS-Code tests. Apply the refinements above rather than replacing those tests.

## 5. Answers to the four questions

1. **Pause or rotate current.log?** Pause first, with D6 and D7 safeguards. This preserves
   size-only rotation and avoids pruning history through repeated small downloads.
2. **Whole-file or per-line CRC?** Whole-file CRC-32, byte count and sequence numbers are
   sufficient for version 1. Specify CRC-32/ISO-HDLC and require a valid END record.
   No per-line checksum is needed.
3. **Where should tail live?** Keep optional tail in Stage 3, convenient from VS Code,
   sharing the same writer and bounded output machinery.
4. **Does HWCDC change the space-check approach?** Yes. The mutex race and disconnected
   write behavior invalidate the claimed nonblocking and delivery guarantees. See F5,
   F6, D3 and D4.

## 6. Windows control lines and local browser page

**W1 — Reset behavior.** USB Serial/JTAG supports control-line reset sequences; DTR and
RTS are not harmless placeholders. Current Chromium Windows source enables DTR and,
without hardware flow control, RTS during port configuration. `setSignals()` runs after
opening, so it cannot prevent transitions already made by the driver.

The exact reset outcome on the owner's installed browser, Windows driver and board is
**unverified**. Keep Step 0. Do not promise that adjusting signals necessarily eliminates
resets. A page cannot compare pre-open boot information on its first connection unless
it was recorded beforehand; Step 0's existing `status` comparison handles that.

Sources: [Chromium Windows serial implementation](https://raw.githubusercontent.com/chromium/chromium/main/services/device/serial/serial_io_handler_win.cc),
[Espressif reset implementation](https://raw.githubusercontent.com/espressif/esptool/master/esptool/reset.py),
[Chrome signal handling](https://developer.chrome.com/docs/capabilities/serial#handle-signals).
These upstream sources do not establish which Chromium revision is installed on this PC.

**W2 — Opening the page from disk.** This is a reasonable starting point. Secure-context
rules treat `file:` URLs as potentially trustworthy, although browsers may impose
restrictions. Actual Web Serial permission and operation from disk on this PC remain
**unverified**. Check `isSecureContext`, `navigator.serial`, and a user-initiated
connection. Retain localhost as fallback.

Sources: [Secure Contexts specification](https://w3c.github.io/webappsec-secure-contexts/#is-origin-trustworthy),
[Chrome Web Serial guide](https://developer.chrome.com/docs/capabilities/serial).

## Evidence appendix

- Profile: `sketch.yaml`; generated profile: `build/build_amoled-1-8/build.options.json`.
- Firmware: `companion.ino` defines and starts `USBSerial`;
  `src/net/net_module.cpp` contains `benchCommand[24]` and `netBenchLoop()`.
- Installed core root:
  `%LOCALAPPDATA%/Arduino15/internal/esp32_esp32_3.1.3_e149c3cd368ed269`.
- Core sources: `cores/esp32/HWCDC.cpp`, particularly `flushTXBuffer()`,
  `availableForWrite()` around line 406, and `write()` around line 418;
  `USBCDC.cpp`, `USBCDC.h`, `USBMSC.h`, and `boards.txt`.
- Installed SDK root:
  `%LOCALAPPDATA%/Arduino15/internal/esp32_esp32-arduino-libs_idf-release_v5.3-489d7a2b-v1_80ffc9027a/esp32s3`.
- Its `sdkconfig` and `build/build_amoled-1-8/sdkconfig` share SHA-256:
  `0B02347898DFEF469DF8D2AC3220B1CA2CA1501C55D0E87E4A646E8F962B2D6E`.
- Relevant flags: `CONFIG_FATFS_FS_LOCK=0`, `CONFIG_TINYUSB_CDC_ENABLED=y`,
  `CONFIG_TINYUSB_CDC_MAX_PORTS=2`, `CONFIG_TINYUSB_MSC_ENABLED=y`,
  `CONFIG_ESP_CONSOLE_UART_DEFAULT=y`,
  `CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG=y`, and `CONFIG_FREERTOS_HZ=1000`.

## Integration decision

Ready to fold after the corrections above, particularly USB pacing and liveness, safe
current-file handling, pruning and shutdown priority, and the revised bench gate.
Implementation remains subject to the owner's staged-plan approval and bench acceptance.
