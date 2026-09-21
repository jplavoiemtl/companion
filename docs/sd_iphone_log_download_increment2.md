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
survives refused on commands while STOPPING. A stuck USB release keeps exclusion, as
visible via mode status; comprehensive HTTP error/recovery remains the provisional
section 5 work, not solved by this change.

Observed VBUS loss requests exit directly in updatePowerStatus. Central diagnosticsClose
requests shutdown/deep-sleep exit without waiting for OFF or altering the existing caller
wait. No new serial output is added to the close path. Idle is 300000 ms using esp_timer;
valid touch resets it. Status, mode-on retries, USB progress and Wi-Fi reconnection do not.
Link loss remains ACTIVE/link=down; link recovery does not reset idle. No HTTP arrival API
exists yet; add a main-task mailbox for request activity with the server, not a cross-task
call to the touch API. Main and background keep-alive ticks service exits during recovery;
existing blocking operations still bound observation latency. No new real-time guarantee
is claimed. No motion-based exit or screen-memory debounce changes.

## Validation

**124 host checks pass:** existing 92, plus 19 mode and 13 media-admission checks.
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
