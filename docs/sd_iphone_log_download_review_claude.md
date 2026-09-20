# iPhone log retrieval - Claude review of the design

Date: September 20, 2026. Reviewer: Claude. Status: design review before implementation.
Inputs: [original draft](sd_iphone_log_download_plan.md) (now tracked, content unchanged),
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
warning: one predicate, three refusals, two backstops and one host check. The architecture
question - where the socket lives - is now **decided against the installed SDK**: Option A,
`esp_http_server`, for reasons in that section. Everything else is detail that the bench
gates below can handle.

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
  **Ordering is load-bearing:** both still callers run `imageBegin()` *first*
  (`image_fetcher.cpp:648` and `:693`), and `imageBegin()` also terminates the previous
  image as replaced. Admission must therefore be checked **before any lifecycle mutation**,
  not inside the preparation that follows it.
- `videoStreamStart()` (`video_stream.cpp:734`) - add the clause next to the existing
  Wi-Fi-offline refusal, which already returns false and records
  `LIVE_REQUEST result=refused reason=...`. This single point covers both the Live button
  and the motion handover, because both reach the feed through it.

**3. Two backstops at the chokepoints**, for the caller nobody has written yet. Inside
`prepareForRequest()` and at the top of `videoStreamStart()`, refuse and record with a
`path=late` marker if the mode is somehow active. `prepareForRequest()` is `static void`
with two real callers today, so this means changing it to return `bool` and having both
callers honour it.

**Corrected after Codex's review:** `if (!prepareForRequest()) return;` is *not* a complete
backstop contract. Because `imageBegin()` already ran, a late refusal leaves an unpaired
`IMAGE_BEGIN` and has already marked the previous operation replaced. A backstop that fires
must therefore either finish the lifecycle it finds open, or - better - never be the thing
that decides admission. Treat the chokepoint check as a **diagnostic assertion** that
records `path=late` and closes what it finds, while the authoritative guard stays ahead of
all side effects. `videoStreamStart()` is also not a *second*, independent guard for the
Live button, since the direct Live path never touches `prepareForRequest()` - it is that
path's only guard. Preserve `motionTriggered` assignment after preparation (preparation
clears it) and preserve the preparation-inclusive timing that `imageBegin()` measures.

**4. The reverse direction, in one place.** Mode entry is refused unless all of:
`vbusPresent`, logger ready (the existing `usbStatus().ready` definition),
`WiFi.status() == WL_CONNECTED`, `!imageFetcherIsBusy()`, `!videoStreamActive()`, and
`!diagnosticsUsbBusy()`. Each refusal records its own reason so a failed entry explains
itself on the console and in the log.

**That list is insufficient, and Codex found the hole.** `imageFetcherIsBusy()`
(`image_fetcher.cpp:227-230`) tests `pendingEndpoint`, `requestInProgress` and the three
in-flight `httpState` values. When a motion-triggered still finishes displaying
(`:591-597`), `httpState` becomes `HTTP_COMPLETE` and `requestInProgress` becomes false,
while `imageDisplayTimeoutActive` stays true and `motionTriggered` is still pending. For
the whole `MOTION_STILL_TIMEOUT` window the fetcher reports **not busy**, so mode entry
passes every check above - and then the pending handover at `:320-332` calls
`videoStreamStart("motion_handover")`, which the new guard refuses, and the refusal path
runs `returnToPreviousScreen()`, changing the UI underneath a download session. An ordinary
still has the same shape with its automatic return timer.

Entry must therefore also require **no pending display-timeout or handover**: test
`imageDisplayTimeoutActive` / `motionTriggered` as part of admission, or deliberately
cancel the pending transition on the main task before entering. Do not widen
`imageFetcherIsBusy()` itself without reviewing the MQTT retry-deferral callers at
`companion.ino:2497-2506`, which use it for a different purpose.

**5. One host check.** Add `tools/tests/media_admission.test.cjs` in the style of the
existing source-contract checks: enumerate every call site of `prepareForRequest()` and
`videoStreamStart()` across `companion.ino`, `image_fetcher.cpp` and `video_stream.cpp`,
and assert each is either guarded or is itself a guard, scanning the whole production tree
and excluding comments. That converts "did we remember all five paths?" from a code-reading
exercise into a check that fails when a sixth path appears, and it protects this work when
the LVGL screen is added. **It proves nothing about runtime invariants**, so pair it with
behavioural cases: refused entry leaves no pending endpoint, no UI or buffer mutation and
no unpaired begin/end; allowed entry still leaves normal Latest, history-back and motion
handover working afterwards.

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

## Architecture: decided - Option A, `esp_http_server`

The two options were: **A**, an `esp_http_server` task with a bounded handoff from the
writer, and **B**, a non-blocking listening socket polled by the writer itself, reusing the
accepted USB state machine with `send()`/`EWOULDBLOCK` in place of `sendLine()`'s
availability check. The deciding question was whether the installed SDK even permits lwIP
calls from the writer, whose stack is in PSRAM.

### What the installed SDK actually says

Checked in `Arduino15/packages/esp32/tools/esp32s3-libs/3.3.11`, the bundle the
`amoled-1-8-core-3-3-11` profile builds against. No firmware was built or flashed for this
check.

- `CONFIG_SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY=y` and
  `CONFIG_FREERTOS_TASK_CREATE_ALLOW_EXT_MEM=y`. External task stacks are permitted, which
  is why the current writer works at all.
- `idf_additions.h` documents only one restriction on such tasks: the TCB must stay in
  internal RAM, which `DIAG_WRITER_STACK_PSRAM=1` already honours with its static TCB.
  Nothing in the installed headers prohibits lwIP from an external stack; the documented
  hazard for PSRAM stacks is cache-disabled execution (ISRs, flash operations), and a
  task-level socket call is not that.
- `esp_http_server` is present, on the include path and in `ld_libs`, so it needs
  `#include <esp_http_server.h>` and nothing else.

**So Option B is not forbidden.** The question is settled on other grounds.

### Why Option A wins anyway

1. **Stack budgeting, not fault isolation.** Option B adds an unmeasured caller - lwIP -
   to the task that owns the SD card, whose latest accepted logging-on margin is
   **3096 bytes** (boot 91 overlap console; 3608 earlier in the same session). A separate
   task gets a stack that can be sized and measured on its own, without disturbing a margin
   three accepted stages were spent protecting.
   **Correction after Codex's review:** an earlier version of this section claimed that if
   the server task died, logging would survive. That is wrong. The installed bundle sets
   `CONFIG_COMPILER_STACK_CHECK=y` / `STACK_CHECK_NORM` with
   `CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT=y`, so a stack overflow aborts and reboots the
   whole device. Task separation buys scheduling independence and an independently
   measurable stack budget - not process-like containment. The decision stands on that
   narrower claim, and the server task's own stack margin has to be measured like any
   other.
2. **The component already provides what Option B would hand-roll**, and the installed
   defaults are close to what this design wants: `send_wait_timeout` and
   `recv_wait_timeout` are 5 seconds, matching the existing `STALL_MS`; `lru_purge_enable`
   is **false**, so a new connection cannot evict the socket carrying an active transfer -
   the hazard flagged earlier is off by default; `max_open_sockets` is 7 and should be
   lowered to 2 or 3; `stack_size` is 4096 with
   `task_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT`; `max_req_hdr_len` is 1024 and
   `max_uri_len` is 512. Accept, request parsing, socket limits and timeouts are all
   solved and tested.
3. The handoff is needed either way. SD reads stay writer-owned under both options, so
   Option B's "no cross-task handoff" advantage only holds if the writer also owns the
   socket - which is exactly what (1) rules out.

Settings to pin at implementation: `max_open_sockets` 2-3 - noting the header's own comment
that **three further sockets are reserved for the server's internal working**, so this is
not the whole socket budget - and `lru_purge_enable` false (the default), so a new
connection cannot evict the socket carrying an active transfer.

**The shutdown advice in an earlier version of this section was wrong, and Codex is right
to reject it.** `send_wait_timeout` is `uint16_t` **in seconds**
(`esp_http_server.h:200-201`), so no positive value can express a sub-500 ms bound; there
is no setting that makes the handler fit the close budget. Closing the listening socket
does not interrupt an already-accepted connection, `httpd_sess_trigger_close()` queues work
onto the HTTP task, and `httpd_stop()` waits for that task - so a handler blocked in
`send()` delays all three. The 5 second socket timeouts are also not the writer's
progress-based `STALL_MS` or its overall `CURRENT_MS`; they are a different mechanism.

What follows for the design:

- **Writer cancellation and append recovery must not depend on the HTTP handler.** Main
  publishes cancellation; the writer executes SD cleanup on its own turn and resumes
  appends whether or not the handler has noticed. `finishError()` is `static` inside
  `diagnostics_usb.cpp` and closes and reopens SD directly, so `updatePowerStatus()` cannot
  literally call it - power handling raises a flag, the writer acts on it.
- **Never call `httpd_stop()` from the main loop's power-transition or close path.**
- **The mode needs STARTING and STOPPING states**, both exclusive to media, so teardown
  cannot hand media back while the server still holds memory.
- A synchronous streaming handler also delays favicon and second-request handling. Either
  yield inside the handler on a bounded schedule, or accept and test the delayed reply.

### Internal memory this costs - withdrawn as an estimate, kept as a gate

An earlier version of this section estimated 10-16 KB internal from the configured TCP
sizes and concluded no allocation would approach the 20480-byte gate. **Codex is right that
the inference does not hold, and it is withdrawn.** Three specific errors:

- "`CONFIG_SPIRAM_TRY_ALLOCATE_WIFI_LWIP` unset, therefore lwIP buffers are internal" is
  false. The unset branch in `lwipopts.h` maps `mem_clib_malloc`/`calloc` to plain
  `malloc`/`calloc`, and Arduino's `esp32-hal-psram.c` calls
  `heap_caps_malloc_extmem_enable(CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL)` with that threshold
  at **4096**, so allocations above it can land in PSRAM. Placement has to be inspected,
  not inferred from the preference flag.
- `LWIP_TCP_SND_BUF_DEFAULT` 5744 and `LWIP_TCP_WND_DEFAULT` 5760 are **capacity limits,
  not a reserved-memory account**, and they omit receive paths, pbufs, netconn, PCBs,
  mailboxes, HTTP session state, headers and the handoff buffer. Three fully occupied send
  budgets alone are 17232 bytes before any of that.
- "No single allocation near 20480" is not the same as "20480 stays free". Many small
  allocations fragment the contiguous block, which is precisely what the gate measures.

The configured values remain worth recording - `SND_BUF` 5744, `WND` 5760, `MSS` 1436,
`LWIP_MAX_SOCKETS` 16, server stack 4096 with `task_caps` internal, `max_req_hdr_len` 1024,
`max_uri_len` 512 - but as inputs to a measurement, not a prediction. The binding case is
unchanged: **an MQTT TLS reconnect needing a contiguous ~16 KB while the server holds its
allocations**, measured across server idle, active and slow transfers, the maximum admitted
clients, and repeated reconnect/close/re-entry.

One placement note: `LWIP_TCPIP_TASK_AFFINITY_CPU0` pins the TCP/IP task to core 0 while
the writer runs on core 1. Leaving the server task unpinned (`tskNO_AFFINITY`, the default)
is the right starting point; pinning it is an optimisation with no evidence behind it yet.

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

Measured internal largest-block minima, against the retained **20480** gate: `image_https`
**31732**, `live_tls` **31732**, full `live` **28660**. **Provenance correction after
Codex's review:** the normal-window figures 57332 and 95688 come from the **logging-off**
console of 2026-09-20 10:11, not the accepted logging-on build, and `heap_min_boot` is a
historical minimum for the boot rather than current idle free. They are not a baseline for
this feature; take fresh logging-on numbers in the same sitting as the server measurements.

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

1. **The expected size and a transfer ID go in the served filename.**
   `Content-Disposition: attachment; filename="87-0004-current-262208.log"` - boot, a
   per-boot monotonic request number, the file, the expected bytes. **Codex is right that
   boot plus size is not a unique download ID**: two snapshots of `current.log` taken
   minutes apart can share both. The size is known from `fstat` on the snapshot before
   headers are sent, so this costs nothing. **The Files app shows a file's size, so a
   truncated save is visible on the phone itself**, with no tools and no computer - the one
   check JP can actually perform in the car.
2. **The device computes CRC32 while streaming** - `updateCrc()` is already in the reader
   path - and records the transfer ID, expected size, bytes the transport accepted, CRC and
   result together, atomically, in an `HTTP_GET_END` record beside the existing
   `USB_GET_END`. Count the CRC exactly once despite partial sends or retries, and keep
   "send completed" distinct from "browser saved". **Do not claim this record is always
   available**: it cannot appear in its own snapshot of `current.log`, and a shutdown or SD
   failure may prevent it being persisted at all.
3. **The index page shows the last transfer's result** after a refresh: transfer ID, name,
   expected bytes, bytes sent, crc32, result. That turns "did it work?" into something JP
   can answer from Safari without a second device. The refresh must be **SD-free**, and a
   favicon or index request must never overwrite the stored download result.
4. **The bench gate does the real byte-for-byte proof with a laptop**, not the phone:
   `curl` over the same hotspot, comparing each body against **its own** transfer's expected
   size and CRC. For immutable archives, also compare the whole file against the card. For
   `current.log`, a second request is a **different snapshot** - changed at minimum by the
   retrieval records themselves - so compare it against the matching **prefix** of the card
   file after a safe close, never against an earlier download.
5. **Export one actual Safari-saved file** from the phone and compare it byte-for-byte at
   the bench. Laptop `curl` exercises the server; it does not exercise Safari's save path,
   and that is the path JP will rely on.

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

- **Point 3.** The check is done and lands on your recommendation: `esp_http_server`.
  Not because the simpler design is forbidden - the installed SDK permits lwIP from a
  PSRAM-stacked task - but because adding lwIP to the writer spends the 3144-3640 byte
  margin of the task that owns the SD card, and the component already supplies the
  timeouts, socket limits and non-evicting socket policy that design would hand-roll.
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

Added after the Codex round of September 20:

9. **Pending motion handover**: take a successful remote still, enter download mode during
   the `MOTION_STILL_TIMEOUT` window, and verify the mode result, the screen, the network
   result and that no surprise navigation occurs.
10. **Client that stops reading**: confirm prompt main/UI and writer cleanup while a handler
    is blocked in `send()`, including a power-off and an explicit exit during that block.
11. **Occupied keep-alive sockets**: fill the admitted client budget and confirm refusal
    behaviour with `lru_purge_enable` false, plus favicon and second-request servicing
    during an active transfer.
12. **Teardown and re-entry**: repeated mode enter/exit with STARTING/STOPPING observed,
    confirming media is not handed back while the server still holds memory.
13. **Five-minute expiry across a hotspot loss**, confirming the backstop still fires and
    the mode leaves cleanly.
14. **Server task stack margin** measured like the writer's, since an overflow reboots the
    device rather than being contained.

## Suggested order, one case at a time

1. iPhone-to-laptop reachability proof, no firmware. Record the iOS version.
2. Shared reader/session extraction with USB regression gates, before any HTTP code.
3. Mode entry and exit by USB command: guards, power rules, status reporting, repeated
   entry/exit, idle memory. No HTTP yet.
4. Server start/stop and address display inside the mode.
5. Small immutable archive download, byte-for-byte verified from a laptop.
6. `current.log` snapshot with logging continuing afterwards.
7. The failure gates: cancel, lock/background, hotspot loss, prune conflict, shutdown,
   USB power loss.
8. Memory gate: MQTT reconnect inside the mode with the server allocated.
9. Media admission in both directions, all five paths.
10. Single-session competition: USB request during an HTTP transfer and the reverse.
11. Server-off regression: same-sitting Latest/Live/IMU, no new stalls or resets.
12. LVGL entry screen, then the car: CAR build, accepted profile, blank FAT32, retrieval
    with the engine running.

## Codex follow-up, September 20: six corrections accepted

Codex reviewed this document at `4cbe974` and raised six findings
([their review](sd_iphone_log_download_review.md), `6e08c1f`). I verified each against the
installed SDK and the current source rather than accepting them as reported. **All six
hold, and all six are corrections to my proposals rather than to JP's decisions.** They are
applied in place above; in summary:

| # | Finding | Verified against | Effect |
|---|---------|------------------|--------|
| 1 | Shutdown is not bounded by `send_wait_timeout` | `esp_http_server.h:200-201` - `uint16_t`, seconds | My shutdown advice was impossible; replaced with writer-independent cancellation, no `httpd_stop()` on the close path, STARTING/STOPPING states |
| 2 | Reverse guard misses a pending motion handover | `image_fetcher.cpp:227-230`, `:591-597`, `:320-332` | Entry must also test pending display/handover state; real hole in my proposal |
| 3 | `bool prepareForRequest()` is not a complete contract | `image_fetcher.cpp:648`, `:693` - `imageBegin()` runs first | Admission moves ahead of all lifecycle mutation; the chokepoint becomes a diagnostic assertion |
| 4 | The memory estimate is unsupported | `lwipopts.h` unset branch maps to `malloc`; `esp32-hal-psram.c` enables extmem above 4096 | Estimate withdrawn; capacities are not a reserved-memory account; baselines 57332/95688 were from the logging-off session |
| 5 | A separate task is not crash isolation | `CONFIG_COMPILER_STACK_CHECK=y`, `CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT=y` | Claim narrowed to scheduling and stack budgeting; the architecture decision stands on that |
| 6 | Verification needs per-transfer identity | - | Transfer ID added; "always available" removed; `curl` of `current.log` compared to a card prefix; a real Safari-saved file added to the gates |

One of my own numbers was also stale: the writer's latest accepted logging-on stack margin
is **3096 bytes** (boot 91 overlap), not the 3144-3640 range I quoted from Stage 2.

Nothing here changes the architecture choice, JP's power decision, the five-minute backstop
or the four-layer verification approach. Finding 1 is the one that most changes
implementation work, and findings 2 and 3 are the ones that would have shipped as bugs.

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
8. **Architecture: `esp_http_server`**, decided against the installed 3.3.11 SDK rather
   than by preference. SD reads stay writer-owned; the server task never opens a file.

## Open questions for JP

1. iOS version on the iPhone 13 Pro, recorded with the first reachability test.
