# iPhone log retrieval - bench cases and results

Running bench record for the wireless retrieval feature, one case at a time.
Design: [Codex review](sd_iphone_log_download_review.md) and
[Claude review](sd_iphone_log_download_review_claude.md). Historical draft:
[sd_iphone_log_download_plan.md](sd_iphone_log_download_plan.md).
Branch `iphone-log-retrieval`. Results are appended newest first as cases complete.

Status: design review closed on September 20, 2026 with six corrections accepted.
Case 1 is prepared and not yet run. No firmware exists for this feature yet.

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
