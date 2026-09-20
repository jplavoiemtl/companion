# iPhone log retrieval - Claude review of the design

Date: September 20, 2026. Reviewer: Claude. Status: design review before implementation.
Inputs: [original draft](sd_iphone_log_download_plan.md) (untracked, unchanged),
[Codex review](sd_iphone_log_download_review.md), and the accepted `sd-diagnostics`
branch at `9cf1bf7` (Stages 1, 1B, 2, 3 accepted; normal logging restored on
`amoled-1-8-core-3-3-11`). No firmware, build or flash changes accompany this review.
Source read for this review: `src/diagnostics/diagnostics_usb.cpp/.h`,
`src/diagnostics/sd_diagnostics.cpp/.h`, `src/diagnostics/diagnostics_config.h`,
`companion.ino`, `src/image/image_fetcher.cpp/.h`, `src/video/video_stream.cpp/.h`.

## Verdict

The direction is sound and the scope JP confirmed - iPhone 13 Pro, existing Personal
Hotspot, dedicated download mode that refuses local and remote Latest/Live while logging
and connection recovery continue - is the right version 1. I agree with the substance of
all ten Codex corrections.

This review originally raised two blockers. **JP resolved the first on September 20 by
deciding that retrieval is allowed only while USB power is present, with the car running.**
That decision is verified against the source below and it removes the blocker rather than
working around it. **Blocker 2 - "refuse media" has five entry points in the current code,
not the two the drafts describe - now carries a concrete proposal** rather than only a
warning: one predicate, three refusals, two backstops and one host check. The remaining
undecided item is where the socket lives, and one no-firmware check settles it. Everything
else is detail that the bench gates below can handle.

## Blocker 1, resolved: retrieval only while USB power is present

JP's decision: log retrieval is permitted only when the unit is on USB power, which in the
car means the engine running. Verified against the source, this closes the whole power
question:

- `allowSleep` is set in exactly three places: `initBattery()` at startup
  (`companion.ino:2116`, `allowSleep = !vbusPresent`), and the two transition branches in
  `updatePowerStatus()` - false on USB connect (`:1448`), true on USB loss (`:1455`).
- The inactivity branch at `companion.ino:2539` is guarded by `&& allowSleep`. With USB
  power continuously present, `allowSleep` is false and that branch never runs. No
  shutdown, no deep sleep, regardless of how long JP spends on the phone.
- The `TEST_POWER` variant at `:2571` has no `allowSleep` guard, but it is commented out in
  `secrets.h`, `secrets_private.h` and the example header. Production builds take the
  guarded path. Worth a line in the handoff so nobody enables it during this work.
- A second benefit: the setup-time shutdown at `companion.ino:954` fires when Wi-Fi fails
  **on battery**. On USB power the code falls back to local-only mode and keeps retrying.
  So requiring USB power also removes the "hotspot not up at boot kills the unit before you
  can reach it" case.

**The bench now reproduces the car's power state**, which was the actual substance of the
blocker. A bench unit on USB power and a running car are the same condition for this
feature. Nothing else in the original Blocker 1 - keep-awake caps, deep-sleep wake
handling, a battery-powered bench gate - needs to be built.

Rules that follow from the decision, much cheaper than the keep-awake machinery they
replace:

1. **Refuse mode entry when `!vbusPresent`**, with a visible reason and a logged record.
2. **Exit the mode on USB power loss.** `updatePowerStatus()` already detects the
   transition; the mode aborts any in-flight transfer through the existing `finishError()`
   path, resumes appends, and exits with a recorded reason. Without this, a USB dropout
   re-arms `allowSleep` and the unit begins its 60 second countdown while the mode believes
   it is still open. **JP decided on September 20: exit immediately with a clean abort,
   rather than finishing the file in flight.** JP also confirms the car is electric, so
   there is no cranking dropout to design around.
3. **Keep an idle backstop.** See the exit-conditions section below - motion was proposed
   for this role and withdrawn.

## What the mode's automatic exit is actually for

JP asked what a 10 minute idle timeout is for once the mode is entered and left explicitly.
It is a fair challenge, and the original justification for it is gone: with explicit entry,
required USB power, and exit on power loss, the timer is not protecting the battery and it
is not the thing that ends a normal session. Stated plainly, an idle timer only covers one
case - the mode is left on and nobody exits it.

That case is worth covering, but not because of the clock. The real harm is that **while
the mode is active, Latest and Live are refused.** If JP drives off with the mode still on,
the companion silently stops showing the camera on a motion push for the whole trip, which
is a functional loss with no visible cause. Secondary: the server keeps its internal
allocations, narrowing the margin for MQTT TLS reconnects, and stays reachable to anything
on the hotspot.

### Motion was proposed as that exit and is withdrawn

An earlier version of this section recommended exiting on the stationary-to-moving
transition. JP questioned it, and reading `updateMotionState()`
(`src/imu/imu_module.cpp:439`) shows the objection is correct:

- Motion is detected from the **change** in accelerometer and gyroscope magnitude between
  20 ms intervals, not from sustained velocity:
  `motionDetectedNow = (accelChange > ACCEL_MOTION_THRESHOLD) || (gyroChange > GYRO_MOTION_THRESHOLD)`.
- `ACCEL_MOTION_THRESHOLD` is **0.04 m/s²**, roughly 0.004 g. That is sensitive enough that
  closing a door, someone getting in, or reaching over to tap the panel can trip it - and
  entering the mode by touching the screen is itself a candidate.
- Once tripped, `g_isCurrentlyMoving` latches for at least `MOTION_TIMEOUT` (30 s) of
  stillness, so a single bump and a real departure look the same for 30 seconds. Requiring
  "moving for N seconds" does not separate them without new detection logic.

So the failure direction is a **false positive that aborts a download while JP is still
using it**, which is worse than the problem it was meant to solve. Withdrawn for version 1.
If the idle window ever proves annoying in practice, motion can be revisited with a
sustained-motion qualifier - counting detection intervals over a window rather than reading
the latched flag - but that is new logic, and it should be justified by an observed
annoyance rather than added speculatively.

### What ends the mode instead

- **Explicit exit** - USB command on the bench, button on the screen in the car. This is
  the normal path and JP's own design; nothing automatic needs to be clever.
- **USB power loss** - covers the car being switched off with the mode open.
- **Idle backstop** - the only automatic exit left, and now genuinely load-bearing rather
  than decorative. Reset it on any HTTP request **and** on any panel touch, so it cannot
  fire while JP is reading the file list without tapping anything.
- **Shutdown**, which already takes precedence over everything.

The worst remaining case is driving away with the mode on: the car stays powered, so USB
loss will not close it, and Latest/Live stay refused until the backstop expires. That
bounds the harm at the timeout value, which is why the value matters more than it did when
motion was also in the set. **JP chose five minutes on September 20**, reasoning that a
session should not last that long anyway because normal operation of the unit is suspended
throughout. Every exit uses the existing abort path and records its reason, so the log
always says which one ended the session.

Historical detail, retained because it explains why the rules above exist: on battery,
`INACTIVITY_TIMEOUT = 60000` (`companion.ino:162`) plus `USB_GRACE_PERIOD = 30000` (`:189`)
drive `goToShutdown()` when stationary or `goToDeepSleep()` when moving, and only
`activity_event_handler` (`:1712`) resets `lastActivityTime` - a touch handler, which a
user looking at their phone never triggers.

## Blocker 2: media admission has five entry points, not two

**In plain terms.** Download mode has to switch the camera off: no stills, no live feed,
in either direction, for as long as the mode is open. The obvious place to enforce that is
`requestLatestImage()`, because that is the function the drafts think of as "fetch an
image". The catch is that the camera can start in five different ways, and only some of
them pass through that function. A single check there looks complete and is not.

| Path | Site | Blocked by a check in `requestLatestImage()`? |
|------|------|-----------------------------------------------|
| Remote Latest over MQTT | `companion.ino:632` | Yes |
| Latest button | `image_fetcher.cpp:658` | Yes |
| Motion still-to-Live handover | `image_fetcher.cpp:330` - `videoStreamStart("motion_handover")` | Indirectly - it can only run after a still that the check already refused |
| Live button | `image_fetcher.cpp:667` - `videoStreamStart()` | **No** |
| **Back button** | `image_fetcher.cpp:688` - `prepareForRequest()` and `pendingEndpoint = "back"` | **No** |

Two real holes remain. The **Live button** calls `videoStreamStart()` directly, and the
**Back button** - browsing older images - calls `prepareForRequest()` directly. Neither
goes anywhere near `requestLatestImage()`. Press either one while a log is streaming to the
phone and the unit starts a TLS fetch that competes for internal memory and for the shared
TLS client, in a mode that is supposed to be quiet.

**The fix is smaller than the problem sounds.** There are two real chokepoints, not five:
`prepareForRequest()` for every still fetch and `videoStreamStart()` for every live feed.
A check in each covers all five paths, and it keeps covering them when the LVGL screen
arrives later and adds its own buttons - which is the reason to put the guard at the
chokepoints rather than at the convenience function sitting on top of them.

### Proposed fix

One predicate, three refusal points, two backstops, one test. In detail:

**1. One owner of the mode state.** The retrieval module exposes
`bool logRetrievalActive()` - a plain main-task-readable flag, in the same spirit as
`videoStreamActive()` and `imageFetcherIsBusy()`, which the media code already consults.
No new locking: the mode is entered and left on the main task, and every caller below runs
there too.

**2. Three refusals at the points that start work**, where the trigger is still known and
the diagnostics are useful:

- `requestLatestImage()` (`image_fetcher.cpp:614`) already opens with a
  `videoStreamActive()` refusal block. Add a `logRetrievalActive()` clause beside it,
  recording `IMAGE_REFUSED trigger=latest reason=download_mode` for button presses and
  `imageNotification("ignored_download_mode")` for the MQTT path, so a remote push is
  accounted for exactly like the existing `ignored_live` / `ignored_echo` cases.
- `buttonBack_event_handler()` (`image_fetcher.cpp:688`) - refuse before `imageBegin()` and
  `prepareForRequest()`, recording `IMAGE_REFUSED trigger=history_back reason=download_mode`.
- `videoStreamStart()` (`video_stream.cpp:734`) - add the clause next to the existing
  Wi-Fi-offline refusal, which already returns false and records
  `LIVE_REQUEST result=refused reason=...`. This single point covers both the Live button
  and the motion handover, because both reach the feed through it.

**3. Two backstops at the chokepoints**, for the caller nobody has written yet. Inside
`prepareForRequest()` and at the top of `videoStreamStart()`, refuse and record with a
`path=late` marker if the mode is somehow active. `prepareForRequest()` is `static void`
with two real callers today, so this means changing it to return `bool` and having both
callers honour it. That is a small, mechanical change and it is what makes the later LVGL
screen safe by construction rather than by review.

**4. The reverse direction, in one place.** Mode entry is refused unless all of:
`vbusPresent`, logger ready (the existing `usbStatus().ready` definition),
`WiFi.status() == WL_CONNECTED`, `!imageFetcherIsBusy()`, `!videoStreamActive()`, and
`!diagnosticsUsbBusy()`. Each refusal records its own reason so a failed entry explains
itself on the console and in the log. This closes the race Codex identified in point 5:
refusing media while the mode is open is only half of it.

**5. One host check.** Add `tools/tests/media_admission.test.cjs` in the style of the
existing source-contract checks: enumerate every call site of `prepareForRequest()` and
`videoStreamStart()` across `companion.ino`, `image_fetcher.cpp` and `video_stream.cpp`,
and assert each is either guarded or is itself a guard. That converts "did we remember all
five paths?" from a code-reading exercise into a check that fails when a sixth path
appears. It is also the test that protects this work when the LVGL screen is added.

**Records to add**: `RETRIEVAL_MODE action=enter|exit trigger=... reason=...` for the mode
itself; everything else reuses `IMAGE_REFUSED`, `LIVE_REQUEST` and the image-notification
vocabulary, so no new record types are needed for the guards.

The reverse direction matters equally, and Codex is right that refusing downloads only
when media is already active leaves the race open. Concretely: mode entry must be refused
while `imageFetcherIsBusy() || videoStreamActive()`, and both must be refused while the
mode is active, with the refusal visible on screen and recorded with the reason. The
existing vocabulary already fits - `IMAGE_REFUSED`, `imageNotification("ignored_*")`,
`LIVE_REQUEST result=refused reason=...` - so add a `download_mode` reason rather than a
new record type.

## Mode entry: USB command for the bench, LVGL screen afterwards

JP's plan is to enter the mode with a USB command during development and add a dedicated
LVGL screen once the system is proven. That is the right order and it fits the existing
code with almost no new surface:

- `diagnosticsCommand()` (`sd_diagnostics.cpp:1439`) already owns the `log ` prefix and
  falls through to a usage line, and `diagnosticsUsbCommand()` already returns false for
  unrecognised `log ` commands so the outer parser can handle them. A pair such as
  `log net on` / `log net off`, plus the mode's state folded into the existing
  `log status` output, needs no new command plumbing.
- **Write mode entry and exit as one function from the start**, with the USB command and
  the future screen as two triggers calling it. The guard conditions - USB power, media
  idle, logger ready, Wi-Fi associated - live in that one place, so the screen inherits a
  path that has already passed the bench gates instead of opening a second one.
- The mode must not be persisted. It is not a screen preference, `screenMem` should not
  see it, and a reboot must come back in normal mode.

One consequence worth planning for: during bench testing the USB console is attached, so
USB retrieval and HTTP retrieval are live at the same time. That is not a nuisance, it is
the single-session competition case - a USB `log get` while an HTTP transfer is in flight,
and the reverse - and it is easier to exercise now than after the screen exists. It also
means the console gives full `@@STATUS`/`[LOG ...]` evidence throughout every HTTP case,
so the first cases produce better records than they would from the screen.

The car test, however, needs a trigger the car can reach. Either the LVGL screen lands
before the car case, or the car case is run with a laptop attached for the USB command -
which defeats the purpose. Plan on the screen being a prerequisite for the car step, not
for the bench steps.

## Architecture: where the socket lives is not yet decided by evidence

Codex recommends `esp_http_server` with a bounded buffer handoff to the writer, pending
memory measurement. That is a defensible choice, but there is a second option that fits
this codebase's proven shape more closely, and the evidence to choose between them does
not exist yet.

**Option A - `esp_http_server` task plus a bounded PSRAM handoff.** Sockets never touch
the writer; the writer fills a ring buffer and the server task drains it. Costs: a new
task with an internal stack, cross-task cancellation, request generation IDs, late
completion handling and stop ordering - all of which Codex correctly lists as things to
define before writing code. Every one of those is a new failure mode that the accepted
USB path does not have.

**Option B - a non-blocking listening socket polled by the writer.** This mirrors the
existing USB state machine exactly: `Phase`, `stopReason()`, `finishError()`,
`closeReaderAndResume()` are unchanged, and `sendLine()`'s "check space, return 0, retry
next turn" becomes `send()` with `MSG_DONTWAIT` returning `EWOULDBLOCK`. No new task, no
handoff buffer, no second owner of the transfer state, and cancellation/cleanup semantics
come free because they are the ones already accepted at every bench gate. The HTTP subset
needed is small: `GET` with two path forms, one response header block.

Two measured facts decide this, and both are cheap to obtain:

1. **Writer stack.** Current writer stack high-water margin is **3144-3640 bytes** of
   `WRITER_STACK` (boots 86/87, repeated across sessions). lwIP socket calls consume
   caller stack. Option B without raising the stack is not safe on that margin. The stack
   is PSRAM (`DIAG_WRITER_STACK_PSRAM=1`), so raising it to 12-16 KB is nearly free in
   internal RAM terms - but see (2).
2. **Whether a PSRAM-stacked task may call lwIP at all** under the installed IDF 5.5.x.
   I have not verified this and will not assume it. Check the installed headers and
   Kconfig for the external-stack restrictions before choosing Option B. If lwIP calls
   from an external-memory stack are restricted or undocumented, Option A wins by
   default and the question is closed.

My recommendation: check (2) first, since it can settle the matter without any firmware
change. If Option B is permitted, prototype it - it is materially less new machinery. If
not, take Option A and set `SO_SNDTIMEO` to a small value (~200 ms) so a blocked send
cannot outlast the 500 ms close budget in `diagnosticsClose()`.

Independent of the choice: **SD reads stay writer-owned**, no HTTP path opens a file
descriptor, and the existing `reserved` single-session flag becomes one session across
USB and HTTP with an explicit busy response in both directions. Codex's point 2 is right
that this means extracting a shared reader/session service from `diagnostics_usb.cpp`,
and that the extraction carries USB regression risk. The regression gate should be the
existing host checks (`node tools/tests/usb_connection_guard.test.cjs`,
`usb_logger_gate.test.cjs`, `sd_log_browser.test.cjs`, `network_diagnostics.test.cjs`)
plus a re-run of the accepted USB bench cases, not a code reading.

## Reused invariants that must survive unchanged

These are current, verified source behavior, not proposals:

- `diagnosticsUsbInit({usbStatus, usbBegin, closeFile, usbResume, usbEnd})`
  (`sd_diagnostics.cpp:1301`): the "pause" hook **is** `closeFile()` - flush plus close of
  the append fd. `usbResume()` reopens `O_WRONLY | O_APPEND` and never creates or
  truncates. A `current.log` snapshot over HTTP must use this same pair.
- `Event` is 512 bytes and `QUEUE_COUNT = 8192 / sizeof(Event) = 16`. The queue-pressure
  abort in `stopReason()` fires at `queued * 2 >= capacity`, i.e. **8 events**. While
  appends are paused there is no drain, so a stalled `current.log` transfer aborts after
  8 queued records. On a parked idle device that is roughly one HEALTH/NET_HEALTH pair per
  interval - comfortable. During hotspot flapping it is not: boot 86 recorded alternating
  disconnect reasons about every 7.245 seconds with suppression active. A hotspot drop
  during a download will abort the transfer, which is correct, and must be a named bench
  case rather than a surprise.
- `CURRENT_MS = 120000` and `STALL_MS = 5000` carry over. The 5 second stall bound is what
  makes Safari backgrounding or screen lock visible as a clean abort; keep it and test it
  deliberately.
- `diagnosticsUsbBeforePrune()` closes the reader before an archive is unlinked
  (`pruneArchive()` calls it first). An HTTP reader needs the same call on the same path -
  the shared session service should own it so there is one implementation.
- `diagnosticsUsbStop()` runs from the writer's `closing()` branch before the close drain,
  and again after the loop exits. The HTTP session must be torn down at the same point,
  with shutdown taking precedence over any in-flight response.
- `archiveName()` accepts exactly `archive-%08u.log`, 20 characters. Reuse it verbatim for
  URL parsing. No path is ever built from user input; names come from the inventory.
- `usbStatus().ready` already excludes test/fixture activity. Mode entry should use the
  same readiness definition rather than a new one.

## Memory: the binding case is MQTT reconnect during download mode, not the download

Measured internal largest-block minima on the accepted build: normal **57332**,
`image_https` **31732**, `live_tls` **31732**, full `live` **28660**, against the retained
**20480** gate. Idle internal free is ~95688 (probe `heap_min_boot`).

Download mode refuses media, so the Live and image windows are not the worst case for this
feature. The worst case is a **TLS reconnect while the HTTP server holds its allocations**:
MQTT reconnects on a 15 second interval (`net_module.cpp:23`) and mbedTLS needs a
contiguous ~16 KB buffer. That single combination is the memory gate, and it must be
measured with a new probe window covering mode-active idle, mode-active transfer, and a
forced MQTT reconnect inside the mode.

A correction to my own draft: "no TLS, so plain HTTP avoids the one-TLS-session limit" is
loosely stated. MQTT holds `secureClient` and the image/video pair share
`imageFetcherSecureClient()`. What plain HTTP avoids is adding a *third* mbedTLS
allocation - a real saving, but the server task stack, socket buffers and lwIP PCBs are
still internal RAM, and none of it is measured. Codex is right that a server task is not
free; my draft implied it was.

## File integrity: what HTTP can and cannot claim

My draft said TCP delivers the bytes intact so no framing or CRC is needed. That is half
right and the wrong half matters. TCP protects against corruption in transit. It does not
protect against a truncated body from a mid-transfer abort, and Safari may still write a
partial file into Files - a short log that looks exactly like a complete one. The USB path
never had this problem because `@@END` carries `bytes` and `crc32` and the browser page
verifies before saving.

JP asked for a recommendation rather than options. This is it - four layers, none of which
requires JavaScript, client-side crypto, or anything that can fail differently on iOS:

1. **The expected size goes in the served filename.**
   `Content-Disposition: attachment; filename="87-current-262208.log"` for the current file
   (boot number, so repeat downloads do not collide in Files) and the existing
   `archive-00000012.log` name with its size appended for archives. The size is known from
   `fstat` on the snapshot before headers are sent, so this costs nothing. **The Files app
   shows a file's size, so a truncated save is visible on the phone itself**, with no tools
   and no computer - the one check JP can actually perform in the car.
2. **The device computes CRC32 while streaming** - `updateCrc()` is already in the reader
   path - and records `bytes` and `crc32` in an `HTTP_GET_END` record beside the existing
   `USB_GET_END`. The device's own log therefore always states what it believes it sent.
3. **The index page shows the last transfer's result** after a refresh: name, bytes,
   crc32, result. That turns "did it work?" into something JP can answer from Safari
   without a second device.
4. **The bench gate does the real byte-for-byte proof with a laptop**, not the phone:
   `curl` the same file over the same hotspot, compare size and CRC32 against the device's
   reported values and against the card contents. Once that passes, the iPhone path only
   has to demonstrate that it saves a file of the same size.

Do not put a streaming CRC in a response header - it is not known when headers are sent.
Do not claim "verified" in the page text: for the iPhone, version 1 verification is
size-only, and the page should say exactly that.

Never append an error string to a log body and complete the response. On a post-header
abort, close the connection without satisfying `Content-Length`. Defer range and resume
until the semantics are defined; if Safari sends a `Range` request for these files, answer
200 with the full body and test what Safari does with it.

## Safari and hotspot assumptions: what to test rather than assume

Codex point 1 is right that reachability is empirical. Additions:

- `Content-Type: application/octet-stream` and `Content-Disposition: attachment`, or
  Safari will render a `.log` inline instead of offering a download. Also
  `Cache-Control: no-store`.
- Safari will request `/favicon.ico`. The server must answer it cheaply (204) without
  touching SD and without taking the session, or the first page load consumes the
  single-session reservation.
- iOS Personal Hotspot uses a 172.20.10.0/28 subnet with the phone at 172.20.10.1. The
  numeric address display in the draft is right; `.local` is not.
- Concurrency: keep `max_open_sockets` small, but be careful with LRU purge - it must not
  close the socket carrying an active transfer.
- Record the exact iOS version with the first reachability result, as Codex asks.

Correction to my draft's laptop validation step: run `python -m http.server` in a scratch
directory containing one harmless file, never in the repository, and prefer a narrow,
temporary firewall rule over disabling the firewall. Codex's point 1 stands as written.

## Where my original draft is now stale or wrong

- "Builds on `sd_diagnostics_plan.md`; if accepted, it becomes a stage after Stage 1B" -
  stale. Stages 1 through 3 are accepted; this is now a prerequisite to Stage 4 field work.
- "Only the transport changes, from USB serial to HTTP" - understated. Framing, base64,
  `WIRE`/`CHUNK` sizing, stall handling and abort reporting are fused into one state
  machine in `diagnostics_usb.cpp`. Separating transport from session is a real refactor
  with regression risk, which is Codex's point 2.
- "Reliable delivery ... no base64 or line format is needed" - see the integrity section.
- "The server turns itself off after about 10 minutes without requests" - would have been
  unreachable in the car on battery, where the unit powers off at 60 seconds. JP's
  USB-power decision restores it as a workable value; see the resolved Blocker 1.
- "Acceptable security: the hotspot is password-protected" - too broad. The narrower true
  statement: physical enable, short lifetime, managed-file IDs only, no arbitrary paths,
  no upload, no delete, no permissive CORS. Codex's per-session capability in the displayed
  URL is a reasonable addition; it must never be logged.

## Where I differ from, or would sharpen, the Codex review

- **Point 3.** Agreed on the constraints, but I would not settle on `esp_http_server`
  before checking the PSRAM-stack/lwIP question above. The simpler design may be
  admissible, and if it is, it inherits accepted cleanup semantics instead of recreating
  them.
- **Point 5.** Agreed, and the five-path table above is the concrete form of it. The Back
  button is the specific gap.
- **Point 6.** Agreed, and it is a blocker with measured numbers, not an open detail.
- **Point 7.** Agreed. The filename-carries-size idea above is a cheaper first step than a
  full snapshot checksum protocol.
- **Point 8.** Agreed: list plus one file first. I would add that the index page should be
  plain HTML with no JavaScript, generated from the same inventory the USB `log list` path
  uses, so the page itself cannot fail in a new way.

## Missing bench gates

Codex's list is good. These are not on it and each catches something real:

1. **USB power loss during an active transfer.** Pull USB power (keeping the data console
   on a separate supply is not possible on this board, so this doubles as a link-loss
   case). Expect: transfer aborted through `finishError()`, appends resumed, mode exited
   with a recorded reason, and the unit then following its normal battery timers. This
   replaces the battery-session gate that JP's USB-power decision retired.
2. **Mode entry refused on battery**, with the reason visible and logged.
3. **Reboot or watchdog during mode.** Confirm the device returns in normal mode - the
   mode must not be persisted to NVS, and must not disturb `screenMem`'s 30 second
   debounce.
4. **Rotation or prune of the archive being served**, verifying the
   `diagnosticsUsbBeforePrune()` equivalent fires on the HTTP reader.
5. **Hotspot drop mid-transfer**, confirming the queue-pressure abort at 8 events and a
   clean reader release, then re-listen after GOT_IP or a visible exit with a reason.
6. **Favicon and second-request handling** during an active transfer: busy response, no
   session theft, no SD access.
7. **MQTT reconnect inside download mode with the server allocated**, as the memory gate.
8. **Truncated-download detection**: force an abort after headers and confirm what Safari
   saves and whether the size in the filename makes it detectable.

## Suggested order, one case at a time

1. iPhone-to-laptop reachability proof, no firmware. Record the iOS version.
2. The IDF PSRAM-stack/lwIP check, deciding Option A or B. No firmware change.
3. Shared reader/session extraction with USB regression gates, before any HTTP code.
4. Mode entry and exit by USB command: guards, power rules, status reporting, repeated
   entry/exit, idle memory. No HTTP yet.
5. Server start/stop and address display inside the mode.
6. Small immutable archive download, byte-for-byte verified from a laptop.
7. `current.log` snapshot with logging continuing afterwards.
8. The failure gates: cancel, lock/background, hotspot loss, prune conflict, shutdown,
   USB power loss.
9. Memory gate: MQTT reconnect inside the mode with the server allocated.
10. Media admission in both directions, all five paths.
11. Single-session competition: USB request during an HTTP transfer and the reverse.
12. Server-off regression: same-sitting Latest/Live/IMU, no new stalls or resets.
13. LVGL entry screen, then the car: CAR build, accepted profile, blank FAT32, retrieval
    with the engine running.

## Decisions recorded on September 20

1. **Retrieval only on USB power**, car running. Resolves the power blocker; see above.
2. **Mode entry by USB command on the bench**, dedicated LVGL screen later, before the car
   step.
3. **USB power loss during a transfer: exit immediately** with a clean abort.
4. **No cranking dropout to design around** - the car is electric.
5. **Hotspot drop during the mode: keep the mode open**, close sockets, re-listen after
   GOT_IP, with the idle timeout as the ceiling.
6. **Verification: the four layers above.** Size in the filename is the phone-side check;
   CRC32 is device-reported and laptop-verified at the bench gate.
7. **Motion-based exit withdrawn**; idle backstop set to **five minutes**, reset by any
   HTTP request or panel touch.

## Open questions for JP

1. iOS version on the iPhone 13 Pro, recorded with the first reachability test.
