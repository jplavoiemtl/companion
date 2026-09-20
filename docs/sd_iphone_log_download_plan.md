# iPhone log download over the hotspot

Status: early proposal, not reviewed. No firmware changes. Builds on
[sd_diagnostics_plan.md](sd_diagnostics_plan.md); if accepted, it becomes a stage after
Stage 1B.

## Goal

Retrieve SD logs from the **car module** without removing the card. Parked in the car,
open a web page on the iPhone and download the log files to the Files app.

The companion is already connected to the iPhone's Personal Hotspot, so the iPhone and
the companion share a local network. No cellular data, home server or extra hardware is
needed.

## Proposal

A small **plain HTTP server on the companion, turned on only when needed.**

1. Turn on the log server from the companion screen (for example a long press on an
   existing element, added in `companion.ino`; SquareLine files stay untouched).
2. The screen shows the address, for example `http://172.20.10.3`.
3. Open that address in Safari on the iPhone. The page lists `current.log` and the
   archives with their sizes.
4. Tap a file to download it. Safari saves it to the Files app.
5. The server turns itself off after about 10 minutes without requests, or from the screen.

## Why this is the simple option

- **Local only:** traffic stays between the iPhone and the companion.
- **No TLS:** plain HTTP needs little memory, and avoids the one-TLS-session limit in
  internal heap.
- **Reuses Stage 1B:** the managed file list, writer-owned reads, and the `current.log`
  close and reopen already exist by then. Only the transport changes, from USB serial to
  HTTP.
- **Reliable delivery:** TCP delivers the bytes intact, so no base64 or line format is
  needed. A normal HTTP download with `Content-Length` is enough.
- **Acceptable security:** the hotspot is password-protected, the server runs only on
  demand, and the logs contain no secrets under the main plan's rules.

## Design points to settle later

- **When downloads are allowed:** refuse while Live or a still image fetch is active, to
  protect memory and the feed.
- **Server choice:** Arduino `WebServer` from the main loop, or ESP-IDF `esp_http_server`
  in its own task. Either way, SD reads stay owned by the writer task.
- **Address display:** show the numeric IP on screen. `.local` names may not resolve over
  a hotspot.
- **Scope:** read-only. List and download managed log files only; no delete, no
  arbitrary paths, no upload.
- **Measurements:** memory with the server on and during a download, using the existing
  probes. Normal operation must be unchanged while the server is off.

## Validation test with a laptop (no firmware)

Before any firmware work, confirm that the iPhone can open a web page served by a
device on its own hotspot:

1. Turn on Personal Hotspot on the iPhone and connect a laptop to it.
2. On the laptop, open a terminal in a folder containing a small test file and run
   `python -m http.server 8000`.
3. Find the laptop's hotspot IP address (usually `172.20.10.x`).
4. On the iPhone, open `http://172.20.10.x:8000` in Safari.
5. Download the test file and check that it appears in the Files app.

If the page loads and the download works, the network path for this feature is
validated. If it doesn't load, check the laptop's firewall before concluding that the
hotspot blocks it.
