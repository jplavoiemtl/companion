# Increment 2 - Claude review

Reviewer: Claude. Date: September 21, 2026. Subject: `fa51d70` against base `c9d2075` and
[spec](sd_iphone_log_download_spec.md) revision 3, sections 3/4/6/8/10/11.
Handoff: [increment 2](sd_iphone_log_download_increment2.md).

**Verdict: one defect to fix before flashing, one spec gap to close or record, two minor
items.** The mode ownership, admission ordering and call-site coverage are correct.

## Defect 1 - unbounded refusal records on the MQTT path - fix before flashing

`image_fetcher.cpp:625-630` emits **both** records when a remote push is refused:

```cpp
diagnet::event("IMAGE_REFUSED", "trigger=%s reason=download_mode", fromNotification ? "mqtt" : "latest");
if (fromNotification) diagnet::imageNotification("ignored_download_mode");
```

Two lines below, the pre-existing Live guard deliberately does the opposite - exactly one
record, chosen by path:

```cpp
if (fromNotification) diagnet::imageNotification("ignored_live");
if (!fromNotification) diagnet::event("IMAGE_REFUSED", "trigger=latest reason=live");
```

That split exists because the two sinks behave differently.
`diagnet::imageNotification()` (`diagnostics_network.cpp:181-193`) suppresses an identical
non-`accepted` result within 5000 ms and counts the suppression. `diagnet::event()` has **no
suppression at all**.

So every MQTT motion push during download mode writes one unbounded `IMAGE_REFUSED` record.
The failure chain is short and realistic:

1. JP enters download mode in the parked car. MQTT stays connected by design, and the
   entrance camera is motion-driven - repeat pushes are the normal case, not an edge case.
2. A USB `current.log` transfer can run while the mode is ACTIVE; entry requires
   `!diagnosticsUsbBusy()`, but nothing blocks a download started afterwards.
3. During that transfer appends are paused, so **the 16-slot queue does not drain**.
4. `stopReason()` aborts at `queued * 2 >= capacity`, i.e. **8 queued records**. Eight
   pushes abort the transfer with `logger_busy`; sixteen start dropping records.

The suppression this bypasses was added for precisely this pressure. **Fix:** match the
existing convention - `imageNotification("ignored_download_mode")` on the notification path,
`IMAGE_REFUSED` on the button path, never both.

## Defect 2 - STOPPING has no bounded escalation - close or record deliberately

Spec section 5: *"If release never arrives, the mode stays in STOPPING, media stays refused,
the reservation and buffer stay held, and the condition is recorded and escalated."*

`logRetrievalTick()` implements the first three and **not the fourth**. A stuck USB release
leaves the mode in STOPPING indefinitely, with media excluded, and nothing recorded. Two
aggravating details:

- The idle deadline is evaluated only for `Active`/`Starting` (`diagnostics_retrieval.cpp:75`),
  so the five-minute rule cannot rescue a stuck STOPPING either.
- The condition is visible only if someone types `log mode status`. In the car there is no
  one to type it, and nothing appears in the log the retrieval exists to collect.

The handoff acknowledges this ("A stuck USB release keeps exclusion, as visible via mode
status") but does not close it. It need not be elaborate - a one-shot record after a bounded
interval in STOPPING would satisfy the spec and make the condition self-evident in the log.
If it is instead deferred to increment 3 with the rest of the release policy, record that
decision explicitly rather than leaving the spec text unmet.

## Minor 1 - the two refusing buttons now disagree about `UI_ACTION`

`buttonBack_event_handler` returns before recording, so a refused Back produces
`IMAGE_REFUSED` only. `buttonLatest_event_handler` is unchanged, so it still records
`UI_ACTION action=latest result=processed` and *then* refuses inside
`requestLatestImage()`. The log therefore claims a processed action that did not happen.

This shape pre-dates the increment (the same is true for `reason=live`), but download mode
makes it routine rather than rare, and this change introduced the inconsistency between the
two buttons. Aligning them is a two-line move.

## Minor 2 - `logRetrievalTick()` runs twice per loop

It is called directly in `loop()` (`companion.ino:2444`) and again inside
`runBackgroundTick()` (`:709`), which `loop()` also calls. Idempotent and harmless, but the
second call is redundant; the `runBackgroundTick()` placement is the one that matters,
because it is what services exits during the Wi-Fi keep-alive loops.

## Verified correct

Checked against the source, not taken from the handoff:

- **124 host checks pass**, counts confirmed: 19 mode, 13 media admission, plus the existing
  92. The mode and admission tests execute the **actual C++ function bodies** translated for
  JS, not reimplementations.
- **Command dispatch order is right, and it matters.** `diagnosticsCommand()` consults
  `logRetrievalCommand()` at `sd_diagnostics.cpp:1440`, **before** `diagnosticsUsbCommand()`
  at `:1498`. Had the order been reversed, `diagnosticsUsbCommand()`'s
  `if (strncmp(command,"log ",4)) return false; if (diagnosticsUsbBusy()) { queueError("busy"); return true; }`
  would have swallowed `log mode off` exactly while a transfer was busy - the moment you most
  need it. The placement avoids that.
- **All five media paths are guarded before side effects.** Latest and Back check before
  `imageBegin()`; Live checks before the `active` fast path and before `ps_malloc`. The
  `media_admission` test enumerates production call sites with comments and declarations
  stripped and asserts exactly four (two `prepareForRequest`, two `videoStreamStart`), all in
  `image_fetcher.cpp`. That is a real inventory check, not a restatement.
- **The `prepareForRequest()` backstop is safe.** It records `path=late`, ends any open
  lifecycle and performs no preparation. An unpaired `IMAGE_END` is impossible because
  `imageEnd()` returns early on `!imageId` (`image_fetcher.cpp:115`).
- **Pending-handover admission is closed in both directions.** Entry refuses on
  `imageFetcherHasPendingDisplay()`, and because Latest is refused while ACTIVE, no new
  pending handover can arise inside the mode. `imageFetcherIsBusy()` is untouched, so the
  MQTT retry-deferral callers at `companion.ino:2497-2506` keep their meaning - asserted by
  test.
- **Live refusal produces no navigation.** `videoStreamStart()` returns false before
  touching the screen, so `buttonNew_event_handler`'s `lv_scr_act() != previousScreen` guard
  is false and `returnToPreviousScreen()` does not run.
- **Power and close integration is main-task and non-waiting.** `logRetrievalExit()` is
  called from `updatePowerStatus()` on the VBUS-loss transition and from
  `diagnosticsClose()`; `logRetrievalTick()` also exits on `!vbusPresent` every tick, so a
  missed transition still recovers. Neither path waits or prints, and
  `diagnosticsClose()`'s bounded caller wait and `false` return are unchanged.
- **Ownership is single-task.** Mode state is touched only from main: commands via
  `handleBenchCommand`, touch via the LVGL indev callback under `lv_timer_handler()`, power
  via `updateBatteryInfo()`, close via `diagnosticsClose()`. No writer or ISR access.
- **Linkage is sound.** `extern bool vbusPresent` matches the definition in `companion.ino`;
  `Status`/`Hooks` aliasing is untouched; the new translation unit sits under `src/`.
- Idle is 300000 ms on `esp_timer`, reset by valid touch only, and the mode is not persisted.

One consequence worth writing down: with `DIAG_ENABLED = 0`, `diagnosticsStorageReady()` is
false, so the mode can never be entered. That is sane - retrieving logs without logging is
meaningless - but it is emergent rather than stated.

## What this review does not establish

Source simulations with mocked platform calls. **Nothing has been compiled**, and
`companion.ino` changed this time, so the generated `companion.ino.cpp` removal noted in the
handoff is genuinely required. No hardware behaviour, timing, memory or event ordering is
evidenced; the bench case remains the first real test.

## Recommendation

Fix defect 1 before flashing - it is a few lines and it protects the transfer path. Close or
explicitly record defect 2. The two minor items can ride along or be deferred with a note.
Then run the bounded entry/exclusion/exit case described in the handoff.

---

## Correction review - `8121d5c..3b9d1cc`, September 21

**All four items are resolved. No new defects. Two low observations, neither blocking.**

Verified against the source and by re-running the suite - **129 checks pass** (22 mode,
15 media admission, plus the existing 92).

### Defect 1 - fixed, and verified against the real suppression code

`requestLatestImage()` now matches the existing Live convention exactly: exactly one record,
chosen by path.

```cpp
if (fromNotification) diagnet::imageNotification("ignored_download_mode");
else diagnet::event("IMAGE_REFUSED", "trigger=latest reason=download_mode");
```

The new test is the right kind: it extracts the **actual `imageNotification()` body** from
`diagnostics_network.cpp` and runs it as the sink rather than mocking suppression. Twenty
refused pushes produce **one** `MQTT_IMAGE` record with `suppressedImage == 19`; a push at
4999 ms is suppressed; at 5000 ms a new record carries `suppressed=20`; and
**zero `IMAGE_REFUSED` records** appear on that path. That closes the queue-pressure chain -
eight refusals can no longer abort a paused `current.log` transfer. The suppression counter
still reaches `NET_HEALTH`, so the aggregate stays visible.

### Defect 2 - escalation implemented, ownership retained

`RELEASE_WARN_MS = 10000` from the first exit request, one-shot via `releaseWarned`, emitting
one queued `RETRIEVAL_STUCK reason=release_timeout exit=... recovery=await_release_or_reboot`
plus one operator line, with `release_stuck` in status. Critically, **the reservation and
media exclusion are retained** - escalation observes, it does not free anything. Rearming is
correctly impossible: `logRetrievalExit()` returns early while already `Stopping`, so a
repeated `off` cannot reset `stoppingAt` or `releaseWarned`, and `enter()` cannot run while
non-`OFF`. The test confirms one record at the threshold, none before, and still one after a
further 300 s plus a repeated `off`.

### Latest event semantics - fixed

`if (requestLatestImage()) diagnet::event("UI_ACTION", ...)` records a processed action only
when admission succeeded. No evidence is lost: every button-path refusal already emits
`IMAGE_REFUSED` with `trigger=latest`, so the press remains visible with an accurate outcome.

### Tick placement - fixed

The direct `loop()` call is removed; the single call site is inside `runBackgroundTick()`,
which `loop()` invokes unconditionally at `companion.ino:2452` and which the Wi-Fi keep-alive
paths also use. Tick frequency and recovery coverage are unchanged. The test asserts exactly
one `logRetrievalTick();` in `companion.ino` and that it precedes `lv_timer_handler()`, which
will catch a future duplicate.

### Observation A - the Live button is now the odd one out

`buttonNew_event_handler` (`image_fetcher.cpp:676`) still records
`UI_ACTION action=live result=processed` **before** calling `videoStreamStart()`, which can
refuse for download mode, Wi-Fi offline or allocation failure. After this correction, Latest
and Back record only on success while Live records unconditionally, so the inconsistency has
moved rather than gone. This is pre-existing and was not introduced by either increment, but
download mode makes Live refusals routine. Aligning it is the same two-line change; JP's call
whether it belongs here or in a later tidy-up.

### Observation B - `release_stuck` is sticky through `OFF`

`releaseWarned` is cleared only in `enter()`, so after a late release recovers, a
`log mode status` in `OFF` still reports `release_stuck=1` until the next entry. The handoff
states this is deliberate, and the log disambiguates through the following
`RETRIEVAL_MODE action=exit result=ok` record. The residual risk is only that an operator
reading the status line alone could read a recovered session as still stuck. Distinguishing
the recovered case, or clearing on the transition to `OFF`, would remove the ambiguity.

Also noted, not a concern: the `USBSerial.println` on the refusal path is unbounded per push,
but it is console output, not a queued record, so it creates no SD queue pressure.

### Recommendation

Ready to build and flash. `companion.ino` changed in this increment, so remove the generated
`build/build_amoled-1-8-core-3-3-11/sketch/companion.ino.cpp` first. Then run the bounded
entry, exclusion and exit case from the handoff.
