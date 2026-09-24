# Increment11 stationary-hold correction - reviewed corrections applied

September24,2026. JP flashed17cfb41: short tap on calibration top band returned home;
a stationary long press did not enter download mode, instead cycling home then G-meter.
No console capture or physical IRQ trace supplied for this attempt. Increment11 first
hardware gate failed at entry; no download/UI acceptance claimed.

## Diagnosis and source evidence

Old my_touchpad_read defaults to REL on every callback and only reports PR on IRQ-flag
or low-pin reads. It clears the flag after coordinates. An otherwise valid held finger
therefore becomes a release on the next no-IRQ read, with later IRQs becoming new taps.
This defect is demonstrated by executing the old callback from17cfb41 with count1 and
one initial IRQ: first sample PR, second sample REL. The fixed callback stays PR across
1200ms of the same sequence. Actual controller IRQ cadence remains unmeasured; the
observed navigation is consistent with this defect, not proof of an electrical trace.
The dashboard's large G-meter button occupies part of the same upper band, explaining
how repeated artificial taps could cross into it. No hold threshold workaround.

Installed Arduino_DriveBus/src/touch_chip/Arduino_FT3x68.cpp returns0/1/2 from the finger
register, and-1 on failure/invalid values. Coordinates likewise return-1 on failed reads.
Installed LVGL8.4 (Arduino15/internal/lvgl_8.4.0_18f8734bf9323e2e/lvgl/src/core/lv_indev.c)
sets indev_act before read_cb and processes reset_query immediately afterwards, before
pointer events. lv_indev_reset(active,nullptr) clears the active pointer target; it is
used for uncertain input, not for ordinary lift.

## Correction in companion.ino

- Idle remains IRQ-gated; no new idle polling.
- Once a verified contact exists, sample finger count and coordinates on subsequent
  LVGL reads even without another IRQ. Confirmed count0 produces normal release.
- Retain the recursive I2C mutex and its10ms acquisition bound; controller transactions
  use the existing Wire50ms timeout. Read errors/invalid coordinates/contact count or
  mutex failure cancel the uncertain input using LVGL reset rather than generate CLICKED.
- After uncertainty, suppress contacts until a verified count0. Recovery polls at most
  once per50ms, including when IRQ is stuck low, so one held finger cannot reacquire a
  different button after an error. Unknown input never refreshes retrieval activity.
- Clear the consumed IRQ before I2C, after acquiring the mutex; a new ISR flag during
  the transaction remains pending. Coordinate validation and raw orientation unchanged.
- No I2C in ISR, no delay/busy loop, global long-press change, screen-specific workaround,
  modified generated files or changes to retrieval ownership/download protocols.

Contact-time I2C traffic increases: up to the existing LVGL10ms read cadence while a
finger is down. Healthy reads involve one count and four coordinate-register reads in
this driver. Actual IMU/I2C impact must be observed on the bounded repeat, not assumed
from mocks. No new IMU-rate investigation or full regression suite is proposed.

## Validation and handoff

All281 host checks pass across13 suites: prior269 unchanged plus12 touch-contact checks.
Tests execute the real callback with mocks for IRQ, count, coordinates, mutex and LVGL
reset, and drive the actual adapted retrieval UI bodies for hold/release and short tap.
They cover continuous no-IRQ hold, verified lift, ghost IRQ, two contacts/movement, ISR
arriving during a read, count/coordinate/mutex errors, suppression until lift, recovery
throttling and tick wrap. Separate old-source execution reproduces false release.
No firmware compilation, flash or hardware test performed. git diff --check passes.

Claude: review contact lifecycle, I2C cadence/error handling, LVGL reset semantics and
possible unintended effects on existing taps/swipes. After clearance JP rebuilds/flashes
and repeats the entry-first case only; do not issue another case now. The selected
profile's generated sketch must be absent before that rebuild (removed with this fix).
If entry works, the previously planned screen/current-download/Stop observations follow
in the same case. The previous code's stationary-hold failure is not a user gesture error.

## September24 - Claude follow-up review and corrections before rebuild

Claude independently ran281 checks and approved the contact-polling diagnosis. Two
requested corrections are now applied, superseding the initial reset/error description
above:

1. Use lv_indev_wait_release(active) with this read returning REL. Verified installed
   LVGL8.4 indev_proc_release sends PRESS_LOST and clears the active object before the
   ordinary release/click path; unlike reset, the object and handler receive cancellation.
   Driver suppression until a verified physical lift remains unchanged.
2. On an active contact, tolerate up to5 consecutive bad coordinate samples, capped also
   at50ms since the last valid point. Return the last point as PR without refreshing
   retrieval activity. A valid sample resets the streak; confirmed count0 releases
   normally even during grace. Bad initial coordinates are ignored with REL, no cancel
   or suppression. Invalid count or mutex failure still cancels immediately.
3. Optional diagnostics added: queued TOUCH_CANCEL and console [TOUCH CANCEL] with
   reason=count/coordinates/mutex, globally limited to one report per5000ms. All logging
   happens outside the I2C mutex. Repeated unknown input does not flood the SD queue.

All288 host checks pass across13 suites: prior269 plus19 touch checks. Added checks cover
coordinate recovery/streak/age bounds, valid lift during grace, ignored first bad point,
count-error precedence, rate limiting and use of wait_release rather than reset.
No firmware build/flash/hardware access. Generated selected-profile sketch is removed or
confirmed absent before handoff. Claude's conditional build clearance is fulfilled by
these corrections. Ready for JP rebuild and the same entry-first case; hardware remains
unproven. No additional broad bench suite or IMU-rate investigation.
