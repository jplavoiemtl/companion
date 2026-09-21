# iPhone log retrieval - bench cases and results

Running bench record for the wireless retrieval feature, one case at a time.
Design: [Codex review](sd_iphone_log_download_review.md) and
[Claude review](sd_iphone_log_download_review_claude.md). Historical draft:
[sd_iphone_log_download_plan.md](sd_iphone_log_download_plan.md).
Branch `iphone-log-retrieval`. Results are appended newest first as cases complete.

Status: design review closed on September 20, 2026 with six corrections accepted.
**Case 1 passed on September 20, 2026**, with transfer timing deliberately deferred. No
wireless firmware exists for this feature yet. The USB-only extraction is now implemented
and reviewed by Claude at `0ba72e5`; its host checks and issued first bench gate are recorded in the
[increment 1 handoff](sd_iphone_log_download_increment1.md). The first normal-current USB extraction gate passed on September 21; the extraction
as a whole remains pending its remaining regression gates.

---

## Increment 1 USB gate 5 - 2026-09-21, 10:30-10:33 - PASS

Orderly shutdown and restart, boots 98 -> 99, fixture-enabled build. JP reports the test
ran fine. Evidence: attachment `281557af-8003-4e4e-b338-fbd7ae3d18b0/Pasted text.txt`,
`C:/Users/photo/Downloads/98-current.log` and `99-current.log`.

- Before: 78888 bytes, browser CRC OK, local CRC32 **6AF52D0E**. After: 86435 bytes,
  browser CRC OK, local CRC32 **FF9C81B8**. The first **78888 bytes match exactly**.
- USB removal at host 10:31:06.494 is expected for this test. Persisted boot 98 records:
  POWER_DECISION action=shutdown moving=0 usb=0 idle_ms=88833 at 10:31:37.471;
  SESSION_END reason=shutdown pending=0 at 10:31:37.531. Boot 99 records reset=power_on,
  context=append. This establishes orderly close and append preservation, not an exact
  diagnosticsClose latency: clocks and record intervals are not a caller-wait measurement.
- Post-restart CRC transfer succeeds; later current_size=87730 (1295 beyond snapshot).
  Same file generation 21/newest 20, archives=7; no rotation. Final ready, active=0,
  paused=0, result=ok, queue=0/16, drops=0, error=none, no USB losses or retained failure.
  Wi-Fi/MQTT connected. Internal largest block 51188 > 20480, internal minimum 94412,
  writer stack minimum 2984. Startup sd_max_us=159198 is reported, not treated as a
  transfer stall; slow write count=0.

All five issued increment 1 bench cases now pass. This close case is idle shutdown,
not in-flight close or deep-sleep repetition; host tests cover session shutdown and
previous accepted platform evidence remains applicable. Full acceptance belongs to JP.

### Next single case issued: restore normal build and verify configuration

Codex restored only the temporary fixture define from 1 to its tracked default 0;
DIAG_ENABLED=1, DIAG_TEST_HOOKS=0 and DIAG_WRITER_STACK_PSRAM=1 remain unchanged.
JP builds/flashes amoled-1-8-core-3-3-11 (no generated-sketch deletion). Connect Chrome
with explicit DTR=true/RTS=false and all test switches off. Send status/log status,
download current once (CRC OK), wait 70 seconds, then send both status commands again.
Send saved console and current.log. Verify hooks=0, fixture/test/gate status lines absent,
ready, no drops/errors, USB idle/unpaused, CRC success and append growth. This is the
normal-configuration handoff check, not a repeat of the fixture suite. After reviewing it,
present increment 1 for explicit acceptance and ask for increment 2 approval separately
or together. No increment 2 implementation has begun.

---

## Increment 1 USB gate 4 - 2026-09-21, 09:40-09:43 - PASS

Selected synthetic archive pruning, same boot 97/fixture-enabled build. JP reports normal
operation. Evidence: attachment `3496d409-128d-4ea3-9f04-89cfa38a2bb2/Pasted text.txt`
and `C:/Users/photo/Downloads/97-current (1).log`.

- Fixture creation completed with archive=00000022, bytes=2097152, result=ok. Two
  earlier list requests returned unavailable while creation was active; this is the
  documented guard, not a logger failure. After creation, list count was 9/newest=22.
- `log test prune 22` armed and fired once; result=pruned, outcome=pruned. Device
  returned pruned. Saved current records show archive-00000022.log completion with
  bytes=1584, duration_ms=61, result=pruned, followed by USB_PRUNE_TEST synthetic=true.
- After removal: count=8, newest=20, archives=7, pruned=1 (was 0), fixture result=
  pruned_by_test. These counts return to the pre-fixture inventory; the capture does
  not include individual FILE lines for a byte-by-byte inventory comparison.
- USB active=1 in the refresh snapshots corresponds to the concurrent listing. Later
  standalone status confirms active=0, paused=0, result=pruned before the retry.
- Normal current retry: **67600 bytes, CRC OK**, 0.68 s. Local size matches, computed
  CRC32 **F33D63CB**. Browser CRC OK verifies device-END comparison; numeric CRC is local.
  Later size=68902; ready, active=0, paused=0, result=ok, zero drops/errors, same boot,
  no new stall/reset, no USB link losses. Largest block=51188, stack minimum=2984.

This passes the production removal-helper regression for an actively read disposable
archive; it does not repeat retention threshold selection. Close coverage remains pending.
No firmware edit/build/flash by Codex. JP's fixture=1 change stays local and uncommitted.

### Next single case issued: orderly shutdown close and restart

Same build, no flash. Keep hotspot available and Live stopped. Save status/log status
and a fresh CRC-checked current.log as the pre-shutdown reference. Unplug USB from the
battery-equipped unit, leave stationary and untouched, and allow up to two minutes for
automatic Shutdown/screen-off. Do not force it with the power button; report a failure
to shut down. After power-off, reconnect USB and Chrome with explicit DTR=true/RTS=false.
Retain both console segments. Send status/log status, download current (CRC OK), wait
70 seconds, then send both status commands again. Send both saved files and the console,
plus observed shutdown behavior. Compare prefix preservation, prior SESSION_END
reason=shutdown pending=0, new boot append, later growth and zero drops/errors.
This is orderly idle shutdown/restart, not a measurement of an in-flight network or USB
cancellation deadline. Existing host tests separately cover shutdown during a session.
After reviewing close evidence, restore the normal fixture=0 build before final handoff;
no increment 2 work without explicit approval.

---

## Increment 1 USB gate 3 - 2026-09-21, 09:37-09:39 - PASS

Queue-pressure abort and retry, boot 97, fixture=1/hooks=0/PSRAM writer=1.
JP reports the test ran fine. Evidence: attachment
`7450c206-1d9e-47d3-b307-6c15820bc770/Pasted text.txt` and
`C:/Users/photo/Downloads/97-current.log`.

- `log test queue` armed; transfer fired once, added=8, queued_at_test=8/16,
  result=injected. Device returned logger_busy, browser reported the expected error.
- Saved retry contains all **eight USB_QUEUE_TEST records**, boot 97 seq=38..45,
  followed by seq=46 `USB_GET_END bytes=1584 duration_ms=54 result=logger_busy`.
  Thus the actual queued evidence was saved after abort, not merely reported injected.
- Retry without rearming: **61227 bytes, CRC OK**, 0.61 s. Local file size matches;
  independently calculated CRC32 **23E3FFE5**. Numeric CRC is local; browser CRC OK
  establishes the device-END comparison.
- Final gate retains fired=1, added=8, queued_at_test=8/16, result=injected,
  outcome=logger_busy; armed/active=none. USB active=0, paused=0, result=ok, queue=0/16.
  Logger ready, drops=0, error=none, no new stall/reset in capture. Queue high=9 is
  compatible with the completion record after the half-full trigger, not an overflow.
- current_size grows beyond the snapshot to 62520; same generation 21/newest 20,
  rotations=0. Largest internal block 51188 exceeds 20480; internal minimum 91928;
  stack minimum 3176 -> 2984 in this fixture build. No USB link losses.
- JP omitted the intermediate status between abort and retry. The persisted abort
  record, all eight records, successful retry and retained gate outcome substantiate
  cleanup/reuse; no intermediate status observation is claimed.

No firmware changes by Codex. JP's local DIAG_USB_TEST_FIXTURE=1 edit remains uncommitted
and untouched for the next case; tracked default stays zero. Prune and close regression
gates remain pending, and increment 2 remains unapproved.

### Next single case issued: prune an actively read disposable archive

No rebuild/flash; same fixture-enabled boot/build. Explicit DTR=true/RTS=false,
all browser test switches off; no Live. Capture status/log status. Send `log test file`,
wait for result=ok and note its new archive number N. Refresh files and preserve the
list. Send `log test prune N` using only that newly created synthetic archive number,
then download that archive. Expect Device: pruned and no saved partial file.
Refresh files and capture status/log status: only the fixture disappears, pruned rises
by one, result=pruned and outcome=pruned, logger ready and drops=0. Stop on mismatch.
Download current.log normally (CRC OK), then status/log status. Send full saved console
and that current.log. This exercises the production reader-close-before-unlink path;
it does not retest retention threshold selection. No separate fixture deletion is needed.

---

## Increment 1 USB gate 2 - 2026-09-21, 09:29-09:32 - PASS

Cancellation and reuse, same boot 95 and normal build. JP reports the test ran fine.
Evidence: attachment `cf8ccfce-6e10-4863-a073-1abd773a8e1b/Pasted text.txt` and
`C:/Users/photo/Downloads/95-current (1).log`.

- Deliberately damaged browser data caused `log abort` at 09:30:11.269; device replied
  `@@ERR reason=aborted` at .281 and browser reported abort confirmed. The 12 ms is
  host-observed command/response timing, not an isolated writer close measurement.
- Following status: active=0, paused=0, bytes=1296, result=aborted, queue=0/16.
  Retry snapshot contains `USB_GET_END bytes=1296 duration_ms=56 result=aborted`, seq=64.
- Normal retry: **41702 bytes, CRC OK**, 0.43 s. Local file has exactly 41702 bytes;
  independently computed CRC32 **E936A9DC**. Browser CRC OK establishes device-END
  comparison; the numeric CRC is calculated locally, not exposed in the capture.
- Post-retry current_size=41862 at 09:30:56, then 43139 at 09:32:07/14: **1277 bytes
  growth**. Same generation 21/newest archive 20, no rotation. Final active=0, paused=0,
  result=ok, ready, drops=0, error=none; no unexpected reset/stall in the capture.
- Internal largest block remains 51188 bytes, internal minimum 91952, writer stack
  minimum 2920. USB losses=0 for both new transfers; Wi-Fi/MQTT remain connected.

This proves cancellation cleanup and subsequent reservation reuse on hardware, not delayed
stale acknowledgements (those remain host-test coverage). Queue, prune and close gates
remain pending. Increment 2 is unapproved.

### Next single case issued: queue-pressure abort and normal retry

Use the existing fixture build: JP sets `DIAG_USB_TEST_FIXTURE=1` in
`src/diagnostics/diagnostics_config.h`, keeping DIAG_TEST_HOOKS=0, DIAG_ENABLED=1 and
DIAG_WRITER_STACK_PSRAM=1, then builds/flashes amoled-1-8-core-3-3-11. No generated-sketch
deletion is needed. This is existing bench instrumentation, not new firmware logic.
The checked-in default remains zero; restore it after fixture regression work.

Chrome: explicit DTR=true/RTS=false, all browser test switches off, no Live, Wi-Fi/MQTT
connected. Capture status and log status, send `log test queue` (expect armed=queue),
download current once (expect logger_busy and no partial save), then status/log status.
Require fired=1, result=injected, outcome=logger_busy, queued_at_test=8/16, added>0,
zero drops, and USB idle/unpaused. Stop on any mismatch. Download current again without
rearming (CRC OK), then status/log status. Send full saved console and successful file
so injected records and the terminal event can be checked. No later case issued yet.

---

## Increment 1 USB gate 1 - 2026-09-21, 09:23-09:25 - PASS

JP reports the test ran fine and the file downloaded. Checkout at review: `41dd46d`,
firmware implementation `7976818`, reviewed by Claude at `0ba72e5`.
Evidence: console attachment `bd3d1894-af58-4bd2-8264-ae1f7f796181/Pasted text.txt`
and local download `C:/Users/photo/Downloads/95-current.log` (09:23:31).

- Browser reports **32598 bytes, CRC OK**, 0.37 s, decoded 87865 B/s. Independent local
  file inspection confirms 32598 bytes and computes CRC32 **59978670**. The capture hides
  raw BEGIN/D/END frames, so that numeric CRC is a local calculation; browser CRC OK is
  the evidence for its device-END comparison. The snapshot ends with `USB_GET_BEGIN`,
  as expected; its own completion record cannot be in the frozen snapshot.
- Same **boot 95**, current generation 21, newest archive 20 throughout; rotations=0.
  Before transfer current_size=32438. At 09:23:45/50, post-transfer size=33890;
  at 09:24:52 size=34898; at 09:24:57 size=35032. Later growth **1142 bytes** confirms
  append continuation with no rotation ambiguity. The first post-transfer status was
  about 15 seconds after completion; the later samples still establish the gate.
- Post-transfer and final USB state: active=0, paused=0, bytes=32598, result=ok;
  queue=0/16, drops=0, failure valid=0. Logger ready, error=none, errno=0; writes 35 -> 45.
  No reset or new stall is visible in this capture. Wi-Fi and MQTT remain connected.
- One transient USB connection observation: losses=1, max_loss_ms=3, pending=0;
  recovered within the unchanged 1000 ms grace, with successful CRC completion.
- Internal largest block **51188 bytes**, above the **20480-byte** gate; internal minimum
  91952 bytes. Writer PSRAM stack minimum moved 3208 -> **2920 bytes** and then remained
  there in the later sample. This is one transfer, not a repeated-resource stability test.

The issued case requested DTR=true/RTS=false; opening is outside this cleared capture,
so those settings are procedural, not independently visible here. No build transcript
was supplied; running behavior is hardware evidence, not a compilation-log review.
No firmware changes, assistant builds or flashes in this result review. Queue-pressure,
prune-conflict, cancellation and close regression gates remain pending, one case at a
time. Increment 2 remains unapproved.

---

## Case 1 result - 2026-09-20, 18:21-18:55 - PASS, timing deferred

**Revised September 20 after Codex's evidence review (`6c3148e`).** The first write-up
stated several inferences as observations. Findings are now split into what was observed
and what was concluded from it. Corrections are marked; nothing in the pass verdict
changed, but four conclusions were withdrawn or narrowed.

**Rig.** Windows PC on the `iphone-jp` hotspot at `172.20.10.13/28`, phone at
`172.20.10.1`, Ethernet left connected throughout. Python 3.13.9
`python -m http.server 8000 --bind 172.20.10.13`, serving a scratch directory, never the
repository. The companion was powered and connected to the same hotspot for the whole
session. iPhone 13 Pro running **iOS 26.6.2**.

**Evidence available.** Server access log, `netstat` socket snapshots, a continuous ping
log, and JP's reports of what the phone displayed. **No request-header capture, no packet
trace and no Files screenshots.** Request headers were never recorded, which bounds several
findings below. Nothing here is packet-level proof.

**Verdict.** The listing loaded in Safari and files saved to the Files app at sizes
consistent with what was served. The case passes on its stated criterion.

### Observed

| # | Observation | Evidence |
|---|-------------|----------|
| O1 | Safari on the **hotspot-host phone** reached a **Wi-Fi client** of that hotspot and loaded the listing. | `172.20.10.1 - - [18:24:05] "GET / HTTP/1.1" 200` |
| O2 | One `GET /favicon.ico` accompanied the first listing load; the later listing load at 18:27:01 was not followed by another. | `"GET /favicon.ico HTTP/1.1" 404` at 18:24:05 |
| O3 | The same 262,144 bytes served as `application/octet-stream` produced a save prompt; served as `text/plain` it rendered inline as text. | `sample.log` versus `sample.txt`, identical bytes, differing `Content-Type` |
| O4 | Two distinct phone source ports appeared in `TIME_WAIT` after a download, on two occasions. | `:51950`/`:51952`, later `:51955`/`:51956` |
| O5 | Every logged request line used `GET`. No `HEAD` appeared. | Server access log, all entries |
| O6 | Files displayed `sample.bin` as **262 KB** and `big.bin` as **2.1 MB**, consistent with 262,144 and 2,097,152 bytes served. | JP's reports from the Files app |
| O7 | One interrupted transfer displayed on the phone as **"713 KB of 2.1 MB"**. | `big.log`, interrupted by a link loss |
| O8 | A 2 MiB body was served without server error, and a local fetch of the same file returned all 2,097,152 bytes. | Access log; local `Invoke-WebRequest` |

### Concluded, with its strength

- **C1 (from O1), strong.** The network path the feature depends on exists: the phone can
  reach an HTTP server running on a device attached to its own hotspot. **Not tested:**
  traffic between two tethered clients, and - the case that actually ships - reaching the
  **ESP32's** server rather than a laptop's.
- **C2 (from O2), moderate.** Safari requests a favicon. **Withdrawn: "once per session".**
  One non-repeat does not establish a caching rule. The firmware requirement does not
  depend on the frequency: **routing must acquire the reader reservation only for requests
  that need it.** A favicon may consume a socket; it must not take the session, touch SD or
  overwrite the last-transfer result. Under JP's rule it still counts as an HTTP request
  for the five-minute idle reset.
- **C3 (from O3), strong for MIME, untested for disposition.** Serving a log as
  `text/plain` puts it on screen instead of into Files, so the firmware sends
  `application/octet-stream`. **`Content-Disposition` was never exercised** - the Python
  server does not send it - so the attachment header is a **design decision**, taken
  because the filename carries the expected size and transfer ID, not a tested result.
  This evidence does **not** show that `text/plain` plus an attachment header would fail
  to download.
- **C4 (from O4), withdrawn as stated.** `TIME_WAIT` is a post-close state; two ports prove
  two **recently closed** connections, not two concurrent sockets, nor a purpose, nor "two
  per download". Two sequential requests preceded each snapshot and fully account for it.
  **The earlier claim that `max_open_sockets = 2` "would have been wrong" is withdrawn** -
  that setting counts clients and permits two, and nothing observed disproves it. Keep the
  2-3 proposal as a prudent start, measure live connection occupancy on firmware, and never
  tie a retrieval session to a TCP accept.
- **C5 (from O5), split.** *No `HEAD`* is supported for the observed request lines.
  ***No `Range`* is not established and the claim is withdrawn.** Verified independently:
  Python 3.13.9's `http/server.py` contains no `Range`, `Accept-Ranges` or `If-Range`
  handling at all, and `log_request` is called by `send_response()` and records only the
  request line and status. A `Range` request would therefore have been ignored, answered
  `200` with the full body, and left no trace. The honest statement: **observed downloads
  completed against a server with no resume support, so version 1 keeps full-body `200`
  behaviour** - not that the client never asks. Capture real `Range`/`If-Range` headers in
  firmware evidence.
- **C6 (from O6, O8), narrowed.** A 2 MiB transfer is feasible over this path. The phone
  sizes are **rounded Files displays, not byte-exact verification**; the local fetch
  validates the laptop's path, not the saved iPhone copy. Byte-exact confirmation still
  requires the exported-Safari-file gate. *Unit correction:* 262,144 bytes is **256 KiB**
  and 524,288 is **512 KiB**; the first write-up said 262 KiB and 524 KiB.
- **C7 (from O7), narrowed.** One interruption was visibly partial against `Content-Length`.
  This does **not** establish that every truncation is detected, nor what object Files
  finally retained. The deliberate truncated-response gate and the Safari-export comparison
  remain necessary.

Also noted: a logged `200` is written when the response starts, so **server status is not
evidence of a completed body**. Exact saved size and content are the stronger evidence.

### What this requires of the firmware

- **`application/octet-stream` with `Content-Disposition: attachment`** (C3).
- **SD-free, reservation-free handling of incidental requests** such as favicon (C2), with
  bounded servicing of a second request while a transfer streams - `max_open_sockets` alone
  does not make handlers concurrent.
- **Spare socket capacity above a single client** as a starting configuration, to be
  measured rather than assumed (C4).
- **Full-body `200` for version 1**, with an explicit, cheap and deliberate policy for
  `HEAD` and for a `Range` request that a future iOS may send (C5).
- `Content-Length` on every response, which is what made O7 visible.

### Two procedural findings

- **A Files "error" is not necessarily a failed download.** `big.bin` showed
  "Your device couldn't connect to the server" yet appeared at full size; Files was failing
  to *preview* an unrecognised binary, which it labels a "MacBinary archive". JP identified
  this. An earlier reading of it as a transfer failure was wrong and briefly motivated a
  `Range` hypothesis that C5 now shows the evidence cannot settle either way.
  **Confirm by size, not by whether the file opens** - and prefer an exact size to a
  rounded display.
- **The hotspot was absent from a PC scan while the companion stayed associated.** Treat
  this as a rig observation on this phone and date, not a universal rule about iOS.

### Timing: deliberately deferred

The 2 MiB duration was not obtained. Three attempts were interrupted by the PC losing the
hotspot (`ConnectionAbortedError 10053`, 12 ping timeouts, `netsh wlan` reporting
`disconnected`), and finally the PC could see the SSID but could not associate.

**Correction to the first write-up, which called this "the test rig".** The cause was not
established. The companion staying associated shows only that there was no total outage
affecting all clients; it does not identify what dropped the PC, and the SSID being absent
from a scan is hotspot-side behaviour. What can be said is narrower: **none of it involves
firmware, which does not exist yet**, and it is not a measurement of the feature.

Deferral stands. The number governing the 120 second `current.log` limit and the 5 second
no-progress abort is **ESP32-to-phone** behaviour, and average throughput alone would not
set either bound - the overall limit follows throughput, while stall safety depends on the
longest no-progress interval, queue occupancy and cancellation latency. The ESP32 is the
actual server but is **not yet proven to be the sole bottleneck**. For reference only, a
local fetch of the same 2 MiB file ran at roughly 1.2 MB/s. The firmware timing gates are
specified in [the implementation spec](sd_iphone_log_download_spec.md).

### Limits of this result

A laptop's Python server is not `esp_http_server`: response construction, timeouts, socket
limits and concurrency are all untested here. Findings are Safari's behaviour on one phone
running iOS 26.6.2, on one date, over plain HTTP with no TLS, with a single client, and
**without request-header capture**. A future iOS can change C2 through C5. The firmware
must pass its own reachability, response-contract and transfer cases.

---

## Case 1 - iPhone reachability proof (no firmware)

**Why this runs first.** Every later case assumes Safari on the iPhone can open a page
served by a device sitting on that same iPhone's Personal Hotspot. Nothing in the project
proves that yet: the companion talks *out* to MQTT and HTTPS, which is the opposite
direction. If this fails, the whole feature needs a different transport, and finding that
out now costs an evening instead of a firmware cycle.

It also answers two design questions cheaply, before any code is written:

- **Content type.** Does Safari render a `.log` inline instead of offering a download? That
  decides whether `Content-Disposition: attachment` and
  `Content-Type: application/octet-stream` are mandatory or merely tidy.
- **Link speed.** How long does 2 MiB actually take over the hotspot? That is the number
  behind the 120 second `current.log` limit and the 5 second stall bound.

**Nothing on the companion is involved.** Do not flash anything. Leave the companion
powered and connected to the hotspot as usual, so the subnet carries its normal client.

### Before you start

Record the iPhone's iOS version: **Settings > General > About > Software Version**. It goes
in the result and is needed context for every Safari behaviour observed later.

### Steps

1. **Turn on Personal Hotspot** on the iPhone 13 Pro. Connect the Windows laptop to it over
   Wi-Fi (not USB tethering - the companion uses Wi-Fi and that is the path under test).

   **Foreground Settings > Personal Hotspot only to establish or re-establish the
   association**, then switch to Safari to run the case. The two cannot be open at once on
   the same phone, so do not treat "keep that screen open" as a standing instruction. If the
   laptop later cannot rejoin, return to that screen, reconnect, and switch back.

2. **Create a scratch folder with three test files.** Use the session scratch directory, not
   the repository - a default directory server exposes everything below it.

   ```powershell
   $d = "$env:TEMP\hotspot_test"
   New-Item -ItemType Directory -Force $d | Out-Null
   # 256 KiB text file, roughly a real current.log
   $line = ("A" * 127) + "`n"
   Set-Content -Path "$d\sample.log" -Value ($line * 2048) -NoNewline -Encoding ascii
   # 256 KiB binary file
   [byte[]]$b = 1..262144 | ForEach-Object { $_ % 256 }
   [System.IO.File]::WriteAllBytes("$d\sample.bin", $b)
   # 2 MiB binary file, the archive size
   [System.IO.File]::WriteAllBytes("$d\big.bin", (New-Object byte[] 2097152))
   Get-ChildItem $d | Select-Object Name, Length
   ```

   Note the three sizes it prints. Those are the numbers you compare against on the phone.

3. **Find the laptop's hotspot address.** `ipconfig` - look for the wireless adapter
   connected to the iPhone. The address is normally `172.20.10.x`, with the phone itself at
   `172.20.10.1`. Confirm the link with `ping 172.20.10.1`.

4. **Serve the folder, bound to that address only:**

   ```powershell
   cd $env:TEMP\hotspot_test
   python -m http.server 8000 --bind 172.20.10.X
   ```

   Use your actual address. Binding it keeps the server off every other interface.

5. **Windows Firewall will prompt.** Allow the profile Windows actually assigned to the
   hotspot adapter - check it first with `Get-NetConnectionProfile`, since a hotspot is
   commonly classified **Public** and a Private-only rule would then never apply. Keep the
   rule narrow (one port, the hotspot address, the hotspot subnet). Do not disable the
   firewall. Remove the rule when the case is finished - step 9.

6. **On the iPhone, open Safari** and go to `http://172.20.10.X:8000/`. The directory
   listing should appear.

7. **Tap each file in turn and record what happens.** This is the actual data of the case:

   - `sample.log` - does Safari **display it inline** as text, or offer a download? This is
     the content-type question.
   - `sample.bin` - expect a download prompt. Save it to Files.
   - `big.bin` - start it and **time it**. While it runs, **lock the phone** for about ten
     seconds, unlock, and see whether the download resumed, continued or failed. Then repeat
     the download and **background Safari** (swipe to home) for about ten seconds instead.

8. **Check the saved sizes** in the Files app (long-press a file > Info, or the list view's
   size column) against the three numbers from step 2.

9. **Stop the server** with Ctrl+C and remove the firewall rule:
   `Windows Security > Firewall & network protection > Allow an app through firewall`, find
   the Python entry created in step 5 and remove it.

### What to send back

- iOS version.
- The laptop's hotspot IP, and whether `ping 172.20.10.1` worked.
- Per file: inline or download, and the size shown in Files against the expected size.
- `big.bin` duration in seconds, plus what happened on screen lock and on backgrounding.
- Anything unexpected: prompts, warnings, stalls, or a listing that loaded slowly.

### Pass criteria

The case passes if the directory listing loads in Safari and `sample.bin` saves to Files at
exactly the expected size. Everything else on the list is observation that shapes the
design rather than a pass condition.

### If the page does not load

Check the firewall rule and the `--bind` address **before** concluding that the hotspot
isolates its clients. A `ping 172.20.10.1` that works while the browser fails points at the
laptop, not the network. Only after both are ruled out is client isolation the explanation -
and that would be a genuine finding worth stopping on, because it would rule out the whole
local-HTTP approach.

### Limits of this case

A laptop's Python server is not the companion's `esp_http_server`, and passing here does not
certify the firmware's own responses, timeouts or socket limits. It establishes the network
path and Safari's handling of the two content types on this specific iOS version, on this
date. The firmware still has to pass its own reachability case later.
