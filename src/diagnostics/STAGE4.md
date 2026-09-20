# Stage 4 - car use and retention observation

## September20 revision: wireless retrieval before car testing

JP changed the sequence: develop iPhone log retrieval on the companion bench
unit, review with Claude, and bench-accept it before real car testing. Then use
accepted firmware and a blank FAT32 card in the car unit. Stage3 remains accepted;
Stage4 field measurements are pending. The USB-after-trip procedure below is
historical and is not the current action. No firmware/build/flash changes.
See [wireless plan review](../../docs/sd_iphone_log_download_review.md) and the
[Claude review](../../docs/sd_iphone_log_download_review_claude.md), which records JP's
September 20 decisions: USB power required, USB-command entry on the bench with the LVGL
screen before the car step, immediate exit on USB loss, mode kept open across a hotspot
drop, a five-minute idle backstop, and the four-layer download verification. Claude's
original untracked draft remains untouched. Work continues on the `iphone-log-retrieval`
branch. Next: the no-firmware iPhone reachability proof and the IDF PSRAM-stack/lwIP check,
one case at a time.


JP accepted Stage3 on September20,2026 after checkpoint f8ac6d8. Stage4 is
field observation first. Current firmware is ready; no rebuild/flash required.
Optional tail and previous coverage limits remain deferred, not implicitly tested.

## First case: one ordinary trip

1. Note local date and approximate companion power-on/start time.
2. Use the companion and hotspot normally for one trip. Note any incident and
   its approximate time, and the power-off/end time. Use controls only parked.
3. After the trip, parked with the computer, connect the web console with
   DTR=true, RTS=false, test switches off. Normal power-down/restart is fine;
   note a retrieval restart separately from an unexpected in-trip restart.
4. Run `status`, click **Refresh files**, then **Download current + newest 3**.
   When downloads finish, run `status` again.
5. Send console output and trip times/observations; leave downloaded logs
   available for inspection. Keep existing card logs intact.

Expected: ordinary device behavior, logger ready after startup, CRC success,
zero queue drops/storage errors. Any field incident is useful evidence; it is
not a reason to silently restart the case. No minimum trip length, forced
outage or repeated Live stress is prescribed.

## Analysis after evidence arrives

- Identify trip boots, Montreal timestamps/clock quality and monotonic intervals.
- Separate trip startup, actual powered use and post-trip USB retrieval.
- Measure record bytes per trip and powered hour; report boundary uncertainty.
- Account for rotation using archive contents rather than current-size subtraction.
- Inspect network/media recovery, power transitions, gaps, resets, resource
  health and missing sequence numbers. State observational coverage limits.
- Request missing archive generations only if this bundle does not cover the trip.
- Keep2MiB files,30 archives,62MiB cap and16MiB reserve until representative
  growth data supports a change. Weekly rotation remains deferred. One trip
  cannot establish long-term reliability or measured daily retention.

Stage4 first trip pending. No code changes or field measurements yet.
