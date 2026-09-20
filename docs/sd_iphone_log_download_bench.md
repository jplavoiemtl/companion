# iPhone log retrieval - bench cases and results

Running bench record for the wireless retrieval feature, one case at a time.
Design: [Codex review](sd_iphone_log_download_review.md) and
[Claude review](sd_iphone_log_download_review_claude.md). Historical draft:
[sd_iphone_log_download_plan.md](sd_iphone_log_download_plan.md).
Branch `iphone-log-retrieval`. Results are appended newest first as cases complete.

Status: design review closed on September 20, 2026 with six corrections accepted.
**Case 1 passed on September 20, 2026**, with transfer timing deliberately deferred. No
firmware exists for this feature yet.

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
