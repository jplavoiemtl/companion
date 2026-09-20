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

**Rig.** Windows PC on the `iphone-jp` hotspot at `172.20.10.13/28`, phone at
`172.20.10.1`, Ethernet left connected throughout. Python 3.13.9
`python -m http.server 8000 --bind 172.20.10.13`, serving a scratch directory, never the
repository. The companion was powered and connected to the same hotspot for the whole
session. iPhone 13 Pro running **iOS 26.6.2**.

**Verdict.** The listing loaded in Safari and `sample.bin` saved to Files at exactly
262,144 bytes, so the case passes on its stated criterion. `big.bin` also saved in full at
2,097,152 bytes. Seven substantive findings, two of which are firmware requirements.

### Findings

| # | Finding | Evidence |
|---|---------|----------|
| 1 | **Hotspot client-to-client reachability works.** This is the premise of the whole feature and was previously an assumption. | `172.20.10.1 - - [18:24:05] "GET / HTTP/1.1" 200` |
| 2 | **Safari requests `/favicon.ico`**, once per session rather than per page load - the second listing load did not repeat it. | `"GET /favicon.ico HTTP/1.1" 404` at 18:24:05; absent at 18:27:01 |
| 3 | **Content type decides inline versus download.** Identical 262,144 bytes: served as `application/octet-stream` Safari offered a download; served as `text/plain` Safari rendered it inline as text. | `sample.log` versus `sample.txt`, same bytes, different `Content-Type` |
| 4 | **Safari opens two TCP connections per download.** A speculative or preconnect socket accompanies the real one. | `172.20.10.1:51950` and `:51952` both in `TIME_WAIT` for one GET; repeated at `:51955`/`:51956` |
| 5 | **No `Range` and no `HEAD` requests at any size tested** - 262 KiB, 524 KiB or 2 MiB. Resumability is not required by the client. | Every transfer is a single `GET ... 200` |
| 6 | **A 2 MiB transfer completes**, which is the archive size. | `big.bin` verified at 2.1 MB in Files; local fetch returned all 2,097,152 bytes |
| 7 | **A truncated transfer is visibly partial on the phone**, shown as "713 KB of 2.1 MB" against `Content-Length`. | `big.log` interrupted by a link drop |

### What this requires of the firmware

- **Serve `application/octet-stream` with `Content-Disposition: attachment`**, never
  `text/plain`. Finding 3 makes this mandatory rather than cosmetic: `text/plain` is the
  intuitive choice for a log file and would put a quarter-megabyte of text on screen
  instead of into Files.
- **Answer `/favicon.ico` cheaply** - a 204 with no body, no SD access, and without
  consuming the single retrieval session. Finding 2 means the very first page load would
  otherwise spend the session before the user reaches a download.
- **Budget at least two client sockets for one Safari session** (finding 4), on top of the
  three `esp_http_server` reserves internally. `max_open_sockets = 2` would have looked
  defensible and been wrong.
- **No `Range` support needed for version 1** (finding 5). Deferring resume remains correct.
- Finding 7 supports the verification scheme: `Content-Length` plus the expected size in
  the filename makes truncation detectable on the phone with no tooling.

### Two procedural findings, not firmware findings

- **A Files "error" is not necessarily a failed download.** `big.bin` showed
  "Your device couldn't connect to the server" yet was present at full size; Files was
  failing to *preview* an unrecognised binary, which it labels a "MacBinary archive". JP
  identified this; an earlier reading of it as a transfer failure was wrong and briefly
  sent the investigation toward a `Range` hypothesis that finding 5 disproves.
  **Always confirm by size in Files > Info, never by whether the file opens.**
- **The iPhone stops advertising the hotspot when it locks or leaves the Personal Hotspot
  screen.** Already-associated clients stay connected - the companion never dropped - but
  the PC could not rejoin, and the SSID was absent from a scan. Keep the Personal Hotspot
  screen open on the phone for the whole of any bench case.

### Timing: deliberately deferred

The 2 MiB duration was not obtained. Three attempts were interrupted by the PC losing the
hotspot (`ConnectionAbortedError 10053`, 12 ping timeouts, `netsh wlan` reporting
`disconnected`), and at the end the PC could see the SSID but could not associate at all.
The companion stayed connected throughout every one of those events, so **none of it is
evidence about the hotspot or the feature** - it is the test rig.

Deferring costs nothing real: the number that governs the 120 second `current.log` limit
and the 5 second stall bound is **ESP32-to-phone** throughput, where the board is the
binding constraint, not a laptop's Wi-Fi. That measurement belongs to the first firmware
transfer case. For reference only, a local fetch of the same 2 MiB file ran at roughly
1.2 MB/s.

### Outstanding

- Whether an inbound firewall rule was required could not be established: no rule for port
  8000 was found and no allow-dialog was observed, yet the phone connected. Anyone
  reproducing this should expect to allow the **Public** profile, since a hotspot is
  normally classified Public and a Private-only rule would not apply.

### Limits of this result

A laptop's Python server is not `esp_http_server`: response construction, timeouts, socket
limits and concurrency are all untested here. The findings are Safari's behaviour on one
phone running iOS 26.6.2, on one date, over plain HTTP with no TLS, with a single client.
A future iOS update can change any of findings 2 to 5 - particularly the favicon request,
the socket count and the absence of `Range` - so re-record the version whenever these are
re-tested. The firmware must still pass its own reachability and transfer cases.

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

5. **Windows Firewall will prompt.** Allow **Private networks only**. Do not disable the
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
