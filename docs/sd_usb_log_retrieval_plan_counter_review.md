# Counter-review: SD log retrieval over USB

Date: 2026-09-15. Reviewer: Claude Code. Review only; no firmware or plan changes.

Reviewed [Codex's review](sd_usb_log_retrieval_plan_codex_review.md) of the
[USB retrieval plan](sd_usb_log_retrieval_plan.md), with the
[input brief](ESP32_SD_Log_USB_Investigation.md) and the
[SD diagnostics plan](sd_diagnostics_plan.md) for context. Findings were checked against
the firmware and the pinned Arduino ESP32 core 3.1.3 sources
(`%LOCALAPPDATA%/Arduino15/internal/esp32_esp32_3.1.3_e149c3cd368ed269`).

## Summary

Codex's review is mostly correct. Two findings are real defects in the plan:
- The `availableForWrite()` space check is not race-free (F5).
- A write can report success while its data is discarded (F6).

A few recommendations are more than version 1 needs. The main one is periodic application
acknowledgments (D4): the USB driver already provides the liveness signal. With the nine
changes at the end of this document, the proposal is ready to fold into the main plan.

## Per-ID verdicts

| ID | Verdict | Evidence or reason |
|----|---------|--------------------|
| F1 | Agree | |
| F2 | Agree | `USBSerial.begin(115200)` at `companion.ino:2160`; the page uses 115200. |
| F3 | Agree, low impact | ESP-IDF error logging (`CONFIG_LOG_DEFAULT_LEVEL=1`, secondary console on USB Serial/JTAG) and panic output bypass the HWCDC mutex. They are rare, and the CRC catches any damage. |
| F4 | Agree | |
| F5 | Agree | `availableForWrite()` releases the mutex before `write()` takes it, so another task can fill the ring buffer in between. The effect is a short wait while the host drains the buffer, not a failure. Remove "never waits". |
| F6 | **Agree, most important** | When not connected, `write()` calls `flushTXBuffer()` and still returns the full `size` (`HWCDC.cpp:418-488`). Only the first stalled write returns short and sets `connected = false`; later writes report success while discarding. "Abort on short write" alone would miss most of a lost transfer. |
| F7 | Agree | |
| F8 | Agree | |
| B1 | Agree to soften | The rejection stands; the wording was stronger than the evidence. |
| C1 | Agree | Read-only MSC removes Windows writes, but Windows still caches its view of a FAT volume that the logger keeps changing, so logging would have to freeze. Rejection stands. |
| D1 | Agree, simplified | Drop `log info` and extend the main plan's `log status`. During a transfer, accept only `log status` and `log abort`; other `log` commands return `busy`. |
| D2 | Refine, partly overbuilt | Add protocol version in `@@BEGIN`, CRC-32/ISO-HDLC (standard zlib CRC-32), maximum line length, strict base64, and page-side line buffering. No transfer ID; see "Overbuilt: transfer ID". |
| D3 | Agree | |
| D4 | **Disagree for version 1** | See "Overbuilt: periodic acknowledgments". |
| D5 | Agree, simplified | Pruning happens only at rotation. If it selects the archive being downloaded, abort the download first, then prune. Shutdown aborts a transfer immediately and sends nothing. |
| D6 | Agree | FatFs forbids a second handle on a file open for writing, whatever `CONFIG_FATFS_FS_LOCK` says. Close the append handle, read, then reopen for append on every exit path. Reopen failure follows the existing rule: a write failure disables logging for that boot. |
| D7 | Agree, as one rule | Abort when the queue reaches 50% full, checked before each line. The other half is headroom, so no events are lost. |
| D8 | Partly agree | Bound console history; replace "costs nothing" with "inactive, negligible cost". Keep "current + newest 3": it is a page-side loop with no firmware cost. Test single-file downloads first. |
| T1 | **Agree** | Compare downloaded `current.log` with the first N bytes of the later card-reader copy, because logging continues after the download. |
| T2 | Partly overbuilt | Keep the forced read stall, a dropped or damaged line (page-side test switch), pruning during a download (existing small test limits), and repeated-download memory and handle checks. Drop the two tests listed under "Overbuilt: fault tests". |
| T3 | Agree | |
| T4 | Agree | Touch already pauses during blocking MQTT attempts. Reword to "no additional stalls beyond existing MQTT blocking". |
| T5 | Agree | About 206 wire bytes per 144 file bytes, so 2 MiB is about 3.0 MB, roughly 30 s at 100 KB/s. |
| Q1-Q3 | Agree | Pause `current.log` rather than rotate; whole-file CRC with no per-line checksum; `log tail` stays in Stage 3. |
| Q4 | Agree | |
| W1 | Agree | `setSignals()` runs after the driver has opened the port, so the page cannot prevent the first DTR/RTS change. Remove "adjust until no reset". Step 0 decides whether a reset happens. |
| W2 | Agree | Check `isSecureContext` and `navigator.serial`; keep `localhost` as fallback. |

## Overbuilt: periodic acknowledgments (D4)

The firmware only needs to notice a vanished page, so it can stop the download and resume
logging. The USB driver already signals this: when the page stops reading or closes the
port, the host stops collecting data. A write then stalls for 100 ms and sets
`connected = false`, which `USBSerial.isConnected()` exposes.

Version 1 rule: **before each line, abort if `USBSerial.isConnected()` is false.**

Two backstops cover the remaining cases:
- **Queue rule (D7):** the download aborts before the pause can cost events.
- **Maximum transfer time,** proposed 120 s: a page that keeps reading but ignores the data
  holds the pause no longer than that.

A page that receives data and discards it just sees a failed CRC, which is harmless.
Acknowledgments would add code and a deadline that must tolerate the serial parser
blocking behind MQTT attempts of 5 s or more, inviting false aborts.

Unverified: that a closed or stalled page produces `connected = false` on Windows. The
forced-stall bench test covers it.

## Overbuilt: transfer ID (D2)

The writer runs one transfer at a time. Two page rules prevent old data from mixing into a
retry:
- After `log abort`, wait for `@@ERR reason=aborted` before sending a new `log get`.
- Ignore `@@D` lines received before a `@@BEGIN`.

## Overbuilt: fault tests (T2)

- **Shutdown during a download:** cannot happen in normal use. With USB power present,
  `allowSleep` is false, and `TEST_POWER` is commented out (`secrets.h:9`). Keep the rule
  that shutdown aborts a transfer; skip the bench test.
- **SD read-failure injection:** treat a read error as an abort; no extra test hook.

## Required changes before folding into the main plan

1. **Pacing and liveness:**
   - Check `USBSerial.isConnected()` before every line; abort on false or on a short write.
   - Keep `availableForWrite()` as a best-effort check only.
   - Add a maximum transfer time (proposed 120 s).
   - No application acknowledgments.
   - Remove the claims that the sender "never waits" and that debug prints are "never
     delayed".
2. **Commands:** merge `log info` into `log status`. Only `log status` and `log abort` are
   accepted during a transfer. Validate the generation number and reject trailing text.
3. **Line format:**
   - Add a protocol version in `@@BEGIN`.
   - Name CRC-32/ISO-HDLC.
   - Set a maximum line length.
   - Require strict base64 decoding and line buffering on the page.
   - Add the abort-confirmation rule. No transfer ID.
4. **current.log:** flush, close the append handle, read the recorded size, then reopen for
   append on every exit path. Reopen failure follows the existing write-failure rule.
5. **Queue, pruning and shutdown:** abort at 50% queue, checked before each line. Abort before
   pruning the archive being downloaded. Shutdown aborts immediately without sending.
6. **DTR/RTS:** state that the page cannot prevent the first change when the port opens;
   Step 0 decides whether it resets the board.
7. **Options B and C:** soften the wording as Codex suggests (manual recovery "may be
   required", ROM console versus the application's TinyUSB port, read-only MSC). Both
   rejections stay.
8. **Stage 1B tests:**
   - Compare `current.log` with the first N bytes of the card-reader copy.
   - Add the forced read stall, a dropped or damaged line, pruning during a download, and
     repeated-download leak checks.
   - Reword the touch test and use the 30 s estimate.
   - Test single-file downloads before "current + newest 3".
9. **Wording:** replace "costs nothing" with "inactive, negligible cost"; bound console
   history.
