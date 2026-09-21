# iPhone log retrieval - increment 2 review handoff

Status: **implemented for Claude review, not compiled/flashed or bench accepted**.
Base `c9d2075`, branch `iphone-log-retrieval`. JP explicitly accepted increment 1 and
approved increment 2 on September 21. Increment 3 is not approved.

## Implemented scope

`diagnostics_retrieval.h/.cpp` owns a main-task-only OFF/STARTING/ACTIVE/STOPPING mode.
There is no HTTP server/task, socket, URL, capability or network transfer implementation.

Commands: `log mode on`, `log mode off`, `log mode status`. Replies include state, link,
result, reason, idle age and `server=absent`. Unknown mode subcommands are rejected;
existing log and network bench commands retain their dispatch. Mode status is explicit
rather than appended to existing USB status frames, preserving that protocol.

Entry requires OFF, observed VBUS, logger not closing and ready, Wi-Fi connected, no
image fetch, no Live, no USB reservation, and no pending image display/motion handover.
Each refusal has its own reason and records RETRIEVAL_MODE. The separate image predicate
is `imageDisplayTimeoutActive || motionTriggered`; imageFetcherIsBusy is unchanged.
STARTING has no resource/failure point in this increment and synchronously reaches ACTIVE.
The future server startup/rollback work remains for increment 3.

All non-OFF states exclude media. Latest (button/MQTT), Back and videoStreamStart guard
before lifecycle/UI/buffer/endpoint changes. The shared Live entry also covers motion
handover. prepareForRequest returns bool with a late backstop: records path=late and ends
an existing image lifecycle, without running preparation. Both callers check its return.
Allowed imageBegin still precedes preparation, preserving timing and motion assignment.

Exit publishes STOPPING and posts an existing USB abort if a retrieval is busy; it never
waits or prints on that path. Main tick completes OFF only after the reservation releases.
With no HTTP resources, no other acknowledgement is needed in this increment. Repeated
off is idempotent and explicit; on while non-OFF is refused. The original exit reason
survives refused on commands while STOPPING. A stuck USB release keeps exclusion. After 10 seconds STOPPING emits one queued
RETRIEVAL_STUCK record and one operator warning; status retains release_stuck=1. A late
release recovers naturally; otherwise operator reboot is the stated recovery. Full HTTP
resource/lifecycle recovery remains provisional section 5 work.

Observed VBUS loss requests exit directly in updatePowerStatus. Central diagnosticsClose
requests shutdown/deep-sleep exit without waiting for OFF or altering the existing caller
wait. No new serial output is added to the close path. Idle is 300000 ms using esp_timer;
valid touch resets it. Status, mode-on retries, USB progress and Wi-Fi reconnection do not.
Link loss remains ACTIVE/link=down; link recovery does not reset idle. No HTTP arrival API
exists yet; add a main-task mailbox for request activity with the server, not a cross-task
call to the touch API. The background keep-alive tick services exits in the ordinary loop and during recovery;
existing blocking operations still bound observation latency. No new real-time guarantee
is claimed. No motion-based exit or screen-memory debounce changes.

## Validation

**129 host checks pass:** existing 92, plus 22 mode and 15 media-admission checks.
Mode checks execute actual C++ function bodies translated for JS with platform mocks:
all entry reasons, repeated commands, exclusive transitional states, abort/release wait,
power and logger exits, link recovery, five-minute deadline even when busy, touch reset,
non-waiting shutdown and exact command matching. Media checks exercise refused requests
without side effects, allowed Latest/Back after exit, late lifecycle cleanup, actual
motion-handover branch and the complete prepare/Live call-site inventory without comments.
Live allocation/decode/render behavior is outside the mocked admission prefix.

Existing assertions are retained. usb_logger_gate mocks the new dispatcher as unhandled
for its unrelated commands. usb_connection_guard now extracts from an explicit file,
resolving Claude increment 1 finding 3; constants also name their owning file. The other
two increment 1 deferrals (refused release/progress diagnostics and explicit buffer
accessors) remain due before a second adapter, no later than increment 3.

Command:

```text
node --test tools/tests/retrieval_mode.test.cjs tools/tests/media_admission.test.cjs tools/tests/reader_session.test.cjs tools/tests/usb_connection_guard.test.cjs tools/tests/usb_logger_gate.test.cjs tools/tests/sd_log_browser.test.cjs tools/tests/network_diagnostics.test.cjs tools/tests/operation_diagnostics.test.cjs
```

These are source simulations, not C++ compilation or hardware evidence. No assistant
build/flash. Normal flags stay logging=1, hooks=0, fixture=0, PSRAM writer=1. Main sketch
changed, so the selected profile's generated companion.ino.cpp was removed to avoid the
known stale-sketch build trap. Historical draft untouched. No network retry, TLS, IMU,
writer placement, queue or retention setting changed.

## Claude review requested before JP builds

Review all changes from c9d2075 against spec sections 3/4/6/8/10/11. Scrutinize media
admission before side effects across all five paths, pending display refusal and allowed
handover, main-task ownership during nested keep-alive, STOPPING/USB-release interaction,
power/central-close integration, timer semantics, diagnostics-disabled behavior, and C++
compilation/linkage risks. Confirm tests do not hide a wrong call-site or event-order
assumption. Report concrete defects; do not build or flash.

## First bench case after review, not issued for execution yet

Normal profile, USB power, Wi-Fi/MQTT connected, idle dashboard, DTR=true/RTS=false.
Capture status; mode on must reach ACTIVE; repeated on must refuse not_off. Latest once
must refuse download_mode without opening the media screen. Mode off then status must
reach OFF; repeated off reports already_off. Latest then must work normally. Capture
status and a CRC-checked current.log for event pairing and continued logging. This is one
bounded entry/exclusion/exit case. Battery admission, active media/pending handover,
VBUS loss, idle/touch and hotspot cases follow separately after its review, one at a time.
No increment 3 implementation before explicit approval and resolution of its lifecycle,
descriptor ownership and provisional socket budget decisions.

## Correction after Claude review 8121d5c - September 21

All four findings addressed; quick Claude review required before JP builds/flashes.
Compare the correction commit to `8121d5c`.

1. **MQTT refusal record bound fixed.** Download-mode refusal now follows the existing
   Live convention: only imageNotification(ignored_download_mode) for MQTT, only
   IMAGE_REFUSED for a button. The test executes the actual notification suppression
   body alongside requestLatestImage: 20 repeated pushes produce one MQTT_IMAGE record,
   the next at 4999 ms is suppressed, and at 5000 ms a new record reports suppressed=20.
   No IMAGE_REFUSED is emitted on that path. Existing USB guards/limits are unchanged;
   suppression reduces pressure but is not a guarantee against all queue-pressure causes.
2. **STOPPING escalation implemented.** At 10000 ms from the first exit request, if
   the reservation is still busy, main tick sets a retained release_stuck indicator,
   queues one RETRIEVAL_STUCK reason=release_timeout record with the original exit reason
   and recovery=await_release_or_reboot, and prints one release_timeout status line.
   Repeated off/on attempts do not postpone or rearm the warning. The reservation,
   buffer and media exclusion remain held: no timeout-based free or forced OFF. A late
   release completes OFF; a new accepted mode clears the warning for that new cycle.
   Ten seconds is an observation threshold (longer than USB's 5-second terminal-output
   expiry), not a new transfer bound, close deadline or automatic reboot. Output occurs
   only on main tick, never in logRetrievalExit or diagnosticsClose. Existing blocking
   main work can delay observation. SD recording uses the normal bounded event queue;
   if the writer is stuck or the queue is full, persistence is not guaranteed. Serial
   warning and status remain available while main/USB function. Future car UI escalation
   and HTTP-specific lifetime/recovery guarantees remain due with their planned increments.
3. **Latest UI_ACTION corrected.** processed is recorded only after requestLatestImage
   returns true. Refusal records remain at the authoritative request guard; no extra
   early guard or duplicated refusal is added. Test covers both refusal and success.
4. **Duplicate tick removed.** Ordinary loop uses its existing runBackgroundTick call;
   mode tick remains there before lv_timer_handler, including recovery keep-alive paths.
   The integration check asserts exactly one source call in companion.ino.

Five new behavioral checks bring the total to 129, all eight suites pass; diff check
passes. No firmware build/flash. The selected-profile generated companion.ino.cpp is
confirmed absent after this main-sketch edit. Normal configuration and historical draft
unchanged. DIAG_ENABLED=0 deliberately prevents mode entry (logger_unavailable).
Increment 3 remains unapproved. Review first; the previously drafted single entry/
exclusion/exit bench case remains pending until corrections are reviewed.

## First bench result - September 21

Claude approved corrections at 5ed5ed1. JP's reordered gate 1 passes on boot 103:
entry/repeated-command semantics, successful Latest while OFF, refusal while ACTIVE,
paired exit records, CRC retrieval. [Evidence and next single pending-display case](sd_iphone_log_download_bench.md).
Increment 2 acceptance remains pending its other gates; increment 3 is unapproved.

September 21 gate 2 also passes: completed image display refuses entry with display_pending;
navigation releases it, later entry/exit succeeds, CRC evidence saved. Next single case:
entry during Live, per the bench record. Other increment 2 gates remain pending.
