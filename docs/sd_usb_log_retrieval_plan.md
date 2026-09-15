# SD log retrieval over USB

> **Merged proposal:** the current content now lives in
> [sd_diagnostics_plan.md](sd_diagnostics_plan.md), including the five-second no-progress
> timeout and battery-equipped USB-unplug test guidance. Use that main plan for implementation.
> The text below is the historical standalone proposal; the review files remain unchanged.

Status: reviewed proposal, updated from the owner-accepted
[counter-review](sd_usb_log_retrieval_plan_counter_review.md) of
[Codex's review](sd_usb_log_retrieval_plan_codex_review.md). No firmware or tool changes
are implemented. The original standalone proposal is retained below for review history.

Input: [ESP32_SD_Log_USB_Investigation.md](ESP32_SD_Log_USB_Investigation.md).

## Goal and recommendation

List and download SD logs through the existing USB cable without removing the card.
Wi-Fi, MQTT, display and sensors keep running. The iPhone hotspot is unchanged.

**Option A: reuse the existing USB serial port with a small text protocol and a
single-file Web Serial page.** Close the VS Code Serial Monitor, connect the page to the
same COM port, download, disconnect, then reopen the monitor.

Keep version 1 simple and reusable. There is no companion network server or network
transfer. Options B and C remain rejected for this project.

## What the current build uses

Checked against `sketch.yaml`, firmware, core 3.1.3 and the generated `sdkconfig`.
The generated SDK configuration matches the installed configuration.

| Fact | Evidence | Consequence |
|------|----------|-------------|
| Hardware USB Serial/JTAG is active | `USBMode=hwcdc`, `CDCOnBoot=cdc`; firmware defines `HWCDC USBSerial` | A needs no USB configuration change. |
| HWCDC ignores the baud argument | `HWCDC::begin(baud)` does not use it | Open the page at 115200 for compatibility. |
| A write holds the TX mutex for its buffer | `HWCDC.cpp:418-488` | One call protects a protocol line against other HWCDC writes. Low-level console and panic output can bypass this mutex. |
| Mutex acquisition has a 100 ms timeout | `tx_timeout_ms=100`; failure returns zero | Keep writes short. This is not a total time limit for an entire write. |
| The default TX ring buffer is 256 bytes | `setTxBufferSize(256)`; `availableForWrite()` around line 406 | The space check is best effort. It can wait for the mutex and does not reserve space for the next write. |
| Disconnected writes may report full size despite discarded bytes | `write()` calls `flushTXBuffer()` when disconnected | Check connection state and short writes. Verify the received size, sequence and CRC on the page. |
| FatFs file locking is disabled | `CONFIG_FATFS_FS_LOCK=0` | Close the append handle before opening current.log for reading. Enabling locking would not permit duplicate opens involving write access. |
| Dual CDC and MSC are compiled capabilities | `CONFIG_TINYUSB_CDC_MAX_PORTS=2`, `CONFIG_TINYUSB_MSC_ENABLED=y`; board menu requires USB-OTG for MSC | B and C require TinyUSB on the existing USB connection. They are not active now. |

## Options reviewed

### Option A: shared COM port (recommended)

- Preserve USB mode, board profile and upload workflow.
- Extend the existing serial parser. The planned SD writer owns all card access.
- Continue normal debug output in a browser console panel.
- Keep commands usable from a terminal or a later Python script.

Windows gives the COM port to one application at a time. Opening or closing it may
reset the board; Step 0 checks this before logger implementation.

Throughput is unmeasured. About 206 wire bytes carry 144 file bytes, so a 2 MiB file
uses about 3.0 MB on the wire. At 100,000 wire bytes per second, allow roughly 30 seconds.
Download current and recent archives rather than all 30 by default.

### Option B: two CDC ports (rejected)

Core 3.1.3 already provides `USBCDC(0)` and `USBCDC(1)` with descriptors. A custom
composite USB stack is not needed. It would let the VS Code monitor stay open, but:

- Switching from HWCDC to TinyUSB replaces the current USB Serial/JTAG connection.
  The two controllers share the internal PHY on this connection.
- Existing `HWCDC USBSerial` declarations must migrate to the application CDC type.
- A broken application USB stack may require BOOT and RESET for upload recovery.
- The application's TinyUSB console starts with that stack. Early and panic output
  cannot be relied on through it. ROM USB console behavior is a separate facility.
- Hardware JTAG over this same connection is lost while TinyUSB uses the internal PHY.

The extra migration and validation are not justified for version 1.

### Option C: USB mass storage (rejected)

MSC needs TinyUSB and its migration work. Writable MSC also needs exclusive ownership:
flush, close and unmount the firmware filesystem before Windows access, then remount
after eject. Windows writes and cached writes make an incorrect handoff a corruption risk.

Core provides `USBMSC::isWritable(false)`. Read-only MSC prevents Windows writes, but
Windows still caches a FAT view that firmware logging would change. Logging would need
to freeze for the session, or a separate snapshot would be needed. Neither is simpler
than A for this version.

## Proposed design

### Commands

Use the existing `netBenchLoop()` parser: its 24-byte buffer holds 23 command characters
and a terminator. The commands fit. They do not modify or delete log contents.

| Command | Reply |
|---------|-------|
| `log status` | One `@@STATUS` line with boot, uptime, logger state, current size, newest archive, drops, card size, free space and file count. Extend the main plan's status snapshot. |
| `log list` | One `@@FILE` line per managed file, then `@@LIST_END`. |
| `log get current` | Download `/logs/current.log`. |
| `log get <generation>` | Download `archive-NNNNNNNN.log`; for example `log get 124`. |
| `log abort` | Stop the transfer and reply `@@ERR reason=aborted` after cleanup. Also confirm when already idle. |

Only regular files named `current.log` or matching `^archive-[0-9]{8}\.log$` inside
`/logs/` are eligible. Accept decimal generations from 0 through 99999999 and format
exactly eight digits. Reject overflow, signs, arbitrary paths and trailing text.
A valid number without a matching file returns `not_found`.

One transfer runs at a time. During it, accept only `log status` and `log abort` among
log commands; all other log commands return `@@ERR reason=busy`. Existing `off`, `on`
and `status` retain their behavior. Optional `log tail` stays in Stage 3 and uses the
same writer and bounded output machinery.

### Line format

Protocol records start with `@@`. Other text goes to the console panel. The following
example is illustrative; the base64 payloads are abbreviated.

```text
@@STATUS boot=214 up_ms=812345 logger=ready current_size=183422 newest=124 drops=0 card_mib=30436 free_mib=30102 files=12
@@FILE name=current.log size=183422
@@FILE name=archive-00000124.log size=2096980
@@LIST_END count=12
@@BEGIN version=1 name=archive-00000124.log size=2096980
@@D 1 <base64 payload>
@@D 2 <base64 payload>
@@END name=archive-00000124.log bytes=2096980 lines=14563 crc32=1A2B3C4D
@@ERR cmd=get reason=not_found
@@ERR reason=aborted
```

- Use ASCII protocol lines, maximum **240 bytes including the leading and trailing LF**.
  Send each complete line with one `write()` call. The leading LF separates a partial
  debug print; the page ignores empty lines.
- Each `@@D` contains up to **144 file bytes**, encoded as at most 192 base64 characters.
  Sequence numbers begin at 1 and must be consecutive. A full line is about 206 wire bytes.
- Buffer incoming browser reads until LF. Reads can split lines or contain several lines.
  Enforce the line limit for protocol parsing and bounded storage for ordinary text.
- Require strict standard base64: valid alphabet and padding, no embedded whitespace,
  and decoded length within the chunk limit. Reject malformed data.
- Use **CRC-32/ISO-HDLC**, the standard zlib CRC-32, over decoded file bytes in order.
  `@@END` contains eight hexadecimal CRC digits, byte count and data-line count.
  Validate these against `@@BEGIN` size and the received bytes before reporting success.
- Require a supported BEGIN version and a matching END filename. Missing END or any
  validation mismatch fails the download. Version 1 has no offset resume or per-line CRC.
- There is no transfer ID. Ignore data lines outside a BEGIN-to-END transfer. After
  `log abort`, wait for `@@ERR reason=aborted` before sending another get. If confirmation
  cannot be obtained, close the connection instead of issuing a retry into that session.

### Pacing and transfer bounds

Before every transfer line, and on each retry while waiting for space:

1. Abort if shutdown is requested or `USBSerial.isConnected()` is false.
2. Abort at **50% event-queue occupancy**, leaving half the queue as headroom for logging.
3. Abort when elapsed transfer time reaches **120 seconds**, measured from accepting get.
4. Check `availableForWrite()`. If the complete line does not fit, yield and retry later.
5. Write once when space appears available. Abort if the return value is short.

The space check does not reserve capacity. A concurrent print can cause a wait, so bound
work per writer turn and measure interference. Keep the existing global USB timeout.
Do not use periodic application acknowledgments.

A connection check reports the driver's state, not browser receipt. In core 3.1.3,
`isCDC_Connected()` returns true while its `connected` flag stays set and USB is plugged.
A sender waiting for free space may never enter the write timeout that clears that flag.
The independent 120-second check therefore remains active even during space waits.

### Writer ownership and aborts

The serial parser validates and posts requests. It never accesses the card. Status uses
an existing thread-safe snapshot. The writer reads bounded chunks between logging batches;
logging and close requests take priority. Control replies also use bounded output.

On abort, close the download handle and restore appending if it was paused. Do this
before attempting an error reply. Use `logger_busy` for queue pressure, `timeout` for
120 seconds, `stalled` for a short write, and `read_failed` for a read error or premature
EOF. A lost connection stops output; error delivery must not hold logging paused.

Pruning occurs at rotation. If it selects the archive being downloaded, abort and close
that reader before pruning. No open download handle survives removal of its archive.

Shutdown aborts the transfer immediately without sending a reply. Complete the main
plan's bounded close hook; do not wait for USB output. Existing in-flight storage work
still follows that close deadline.

Record `USB_GET_BEGIN` and `USB_GET_END` with filename, bytes, duration and result.
For current.log, write BEGIN before the snapshot and END after appends resume. Shutdown
uses the existing close records rather than delaying close to send or enqueue replies.

With no host, retrieval is inactive with negligible cost: bounded state and existing
parser polling remain. No download work runs automatically.

### Downloading current.log

Pause appends rather than force rotation. Repeated downloads should not create small
archives and shorten retention.

1. Flush and close the append handle. Record the file size and open a read handle.
2. Send exactly that size. New events remain queued while appends are paused.
3. On success or any abort, close the reader and reopen for append, then drain the queue.
   An initial read-open failure takes this same cleanup path.
4. If reopening for append fails, disable logging for that boot under the existing
   write-failure rule. Never truncate the file as recovery.

Apply the 50% queue rule throughout the pause, including space waits. Bench acceptance
requires no event loss on the high-water abort. The pause increases the amount of recent
history exposed to power loss until queued events have been written and flushed.

### Web Serial page

One file: `tools/sd_log_browser.html`, with no build step or install.

- Connect and Disconnect buttons. Open at 115200, then request `log status` and `log list`.
- A file table, per-file Download, progress, and verified or failed result.
- Keep **Download current + newest 3** as a page-side loop of sequential gets. Validate
  each result before continuing. Test single-file retrieval before this convenience loop.
- Show non-protocol output in a console panel with a fixed history cap; discard oldest
  display entries when full. Do not retain unlimited partial lines or console text.
- Save normal browser downloads as `<boot>-<name>`, such as `214-archive-00000124.log`.
- On cancellation, send abort and follow the confirmation rule. On transport loss,
  invalidate any incomplete download and release browser stream locks and the port.

Check `isSecureContext` and `navigator.serial` before offering Connect. Request port
permission from a user click. Start with a disk-opened page in desktop Chrome or Edge;
use localhost if that environment does not permit it.

DTR and RTS settings take effect after the driver opens the port. The page cannot
prevent the first driver control-line change. Record the settings used in Step 0.
Compare boot and uptime with a prior observation when one exists; the first connection
has no automatic pre-open reference.

## Implementation steps and tests

### Step 0: connection check with existing firmware

Build only the page console: connect, show text and send a typed command. No firmware
change. The existing `status` command reports uptime.

1. Request status in VS Code and retain its uptime, then close the monitor.
2. Connect the page and request status. Record whether uptime restarted or boot output
   appeared. Record the browser version and DTR/RTS settings.
3. Disconnect, reopen VS Code and request status. Record any reset on that transition.
4. Repeat five times, including during Live. Check disk-page permissions as part of this.

Step 0 determines whether this setup resets the board. If it does, review the observed
behavior with the owner before proceeding; changing signals after open may not fix it.

### Stage 1B: USB retrieval, after Stage 1 acceptance

This stage uses the accepted Stage 1 writer, storage layout and rotation. It comes before
Stage 2 network hooks. The owner compiles and flashes in VS Code using the main plan's
build precautions.

Run gate tests one at a time:

1. **Integrity:** compare an archive byte for byte with its card-reader copy. For current,
   compare the download with the first N bytes of the later copy, where N is BEGIN size.
   The remaining file may contain later records. Check size and CRC in both cases.
2. **Throughput:** measure decoded-file bytes per second and wire throughput separately
   for a 2 MiB archive. After single-file tests pass, time current plus newest three.
3. **Non-interference:** download during Live and MQTT `off` and `on` cycles. Meet Stage 1's
   same-session performance and memory limits. Allow no additional UI stalls beyond
   existing MQTT blocking, no new watchdog resets and no queue drops. Check internal heap,
   largest internal block and writer-stack margin.
4. **Debug and validation:** verify ordinary prints remain visible during retrieval.
   Use a page-side test switch to drop or damage one data line. Validation must reject
   the file, and a later retry must succeed.
5. **Abort and liveness:** close the page, unplug USB, send abort, and deliberately stop
   browser reads. Observe driver connection state and verify recovery through connection,
   queue or 120-second limits. No wait for TX space may bypass the deadline. Confirm
   appending resumes and the next download succeeds. Verify abort confirmation ordering.
6. **Current pause:** generate events while retrieving current.log. Exercise the 50% queue
   abort and verify zero dropped events, resumed appends and an error reply when connected.
7. **Pruning:** use the existing small rotation and retention test limits. Make pruning
   select the archive being downloaded. Confirm its reader is closed before deletion,
   the transfer fails cleanly and logging continues.
8. **Repeated downloads:** repeat successful and aborted transfers. Check heap, largest
   block, writer stack and file handles for leaks or accumulating resource loss.
9. **Round trip:** disconnect the page and reopen VS Code. Confirm output and status,
   and check for resets against the Step 0 observations.

Keep the shutdown-abort and read-error handling rules, but add no dedicated
shutdown-during-download bench test or SD read-failure injection hook in version 1.

## Limits and unverified points

- Windows behavior for a closed or stalled page, including when HWCDC detects it, needs
  the forced-stall test. Successful writes alone do not establish browser delivery.
- Reset behavior and disk-page permission depend on the installed browser and driver;
  Step 0 verifies this PC and board.
- Throughput, queue headroom and serial contention require same-session measurements.
  Base64 and CRC protect file validation, not retention of every ordinary debug print.
- The maximum transfer timer is checked between bounded operations. It cannot cancel an
  in-flight filesystem call. Existing MQTT blocking can delay parsing serial commands.
- A paused current.log retains unwritten events in RAM. Power loss can lose those events.

## Review resolution

| Decision | Resolution |
|----------|------------|
| 1. Pacing and liveness | Use connection checks, short-write abort, best-effort space checks and 120 seconds; no application acknowledgments. Keep deadline checks active during space waits. |
| 2. Commands | Extend log status; validate generation and trailing text; only status and abort among log commands while transferring. |
| 3. Protocol | Version 1, 240-byte lines, strict base64, CRC-32/ISO-HDLC, buffered parsing and abort confirmation; no transfer ID or per-line CRC. |
| 4. Current file | Pause with close, read and reopen for append on every exit; reopen failure disables logging for that boot. |
| 5. Queue and ownership | Abort at 50% queue; close the selected reader before pruning; shutdown aborts without serial output. |
| 6. DTR and RTS | Post-open settings cannot prevent the driver's initial change; Step 0 records actual resets. |
| 7. Alternatives | Keep B and C rejected; acknowledge built-in dual CDC, possible manual recovery, distinct ROM output and read-only MSC. |
| 8. Bench gate | Add prefix comparison, read stall, damaged-line detection, pruning and leak checks; retain same-session limits and omit the two rejected fault tests. |
| 9. Scope and wording | Bound console history, describe inactive overhead, retain current plus newest three after single-file tests; optional tail stays in Stage 3. |
