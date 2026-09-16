# Overnight SD log analysis - 2026-09-16

JP supplied F:/current.log after normal shutdown. Analysis only; firmware and
probe behavior are unchanged. The original is preserved byte for byte in
[bench_data/sd_current_2026-09-16_0807.txt](bench_data/sd_current_2026-09-16_0807.txt).

## Finding

The low headroom is not explained by a steady overnight decline in the sampled
free memory. There are two distinct observations:

1. A transient low was first retained between 07:07:15.723 and 07:08:15.745.
   Both surrounding minute snapshots say Wi-Fi and MQTT offline, idle dashboard.
   At 07:08, current free memory had already recovered.
2. At the first recorded connected snapshot at 07:34:15, the largest internal
   block was only 16372 bytes despite the same total free internal memory as
   before the outage. Live then observed 14324 bytes and failed the 20480 floor.

Reconnect/allocation ordering is a useful hypothesis, not an established cause.
The earlier transient might involve an unobserved brief connection or other
allocation. Stage 1 has no per-attempt network events yet. JP recalls returning from a car trip, during which he used the car companion,
then coming downstairs near the bench unit and turning the iPhone hotspot back
on to connect it. He does not recall the exact time or activity at 07:07-07:08.
This supports testing hotspot recovery, without assigning that action to the
earlier transient or confusing the car unit with this bench log.

## Timeline (Montreal local time)

| Time | File line | Wi-Fi / MQTT | Operation | Current internal free | Current largest internal block | Historical internal minimum / DMA largest minimum |
|------|-----------|--------------|-----------|-----------------------|--------------------------------|--------------------------------------------------|
| Sep 15 22:22:05 | 394 | 1 / 1 | idle | 46544 | 30708 | 34964 / 26612 |
| Sep 15 22:23:05 | 395 | 0 / 0 | idle | 85348 | 32756 | 34964 / 26612 |
| Sep 16 07:07:15 | 919 | 0 / 0 | idle | 85348 | 32756 | 34964 / 26612 |
| Sep 16 07:08:15 | 920 | 0 / 0 | idle | 85348 | 32756 | 22676 / 14324 |
| Sep 16 07:33:16 | 945 | 0 / 0 | idle | 85348 | 32756 | 22676 / 14324 |
| Sep 16 07:34:15 | 947 | 1 / 1 | idle | 46544 | 16372 | 22676 / 14324 |
| Sep 16 07:35:15 | 948 | 1 / 1 | live | 46548 | 14324 | 22676 / 14324 |
| Sep 16 07:36:15 | 949 | 1 / 1 | idle | 90168 | 21492 | 22676 / 14324 |

Boot 12 has 706 HEALTH records. All 551 consecutive offline snapshots, from
22:23:05 to 07:33:16, show a largest internal block of 32756 bytes.
Their current free internal memory ranges from 85152 to 85348 bytes; PSRAM free
ranges from 8337032 to 8337096 bytes. The minute snapshots do not exclude brief
intervening connections or allocation dips.

The first historical change is exactly 12288 bytes in each of internal_min,
dma_min and dma_largest. This is arithmetic, not evidence of a named 12 KiB
allocation or a single allocation. Both total and contiguous space recovered
by the 07:08 snapshot; historical minima stay low for the rest of the boot.

At 07:34 the connected current free total equals the pre-outage total (46544),
but the largest block fell from 30708 to 16372. That is consistent with a
different allocation layout or fragmentation rather than simply less total
free memory. It does not identify owners or rule out a leak elsewhere.
After Live ends, total free becomes 90168 while the largest block is 21492.
For comparison, boot 13 at 08:04:34 has current free 90164 and largest 31732.
These are different histories, not a controlled allocation comparison.

Clock correction at 07:34:09 was -933 ms. It occurs after the 07:08 historical
drop and cannot explain it as a logging time reversal. Use boot and up_ms for
ordering; synchronized local times still have the usual clock accuracy limits.

## Field semantics and source checks

- sd_diagnostics.cpp::writeHealth queries current internal_free, internal_largest
  and psram_free when composing the record. Network/screen/operation fields come
  from the main-task health snapshot; its age in boot 12 is 3-999 ms.
- memorySample retains historical internal_min, dma_min and dma_largest.
  Serial log status internal_largest is also historical, unlike the file HEALTH
  field with the same name. Do not treat retained low values as current usage.
- internal_min is the SDK sum of per-region historical low watermarks, not a
  simultaneous global minimum. It cannot name an allocation owner.
- companion.ino calls mqttClient.loop and netCheckMqtt while Wi-Fi is connected,
  deferring reconnect during active still or Live operations.
- The still-image and video modules share one secure image client. Image cleanup
  explicitly stops it; image completion calls httpClient.end. Video reconnect
  stops before connecting; every video stop calls closeConnection and stops it.
  These paths make connection lifetime/order worth inspecting after reproduction,
  but source inspection here does not establish a defect or justify an edit.
- Video deliberately retains its PSRAM frame buffers after first use. The
  overnight PSRAM drop when Live first runs is consistent with that design, not
  by itself evidence of a leak.

## Logger and reset observations

There are 989 complete field-parsable records, 903 HEALTH records and 12 recorded
boots. Boot 3 is absent, consistent with JP's no-card test; absence alone would
not prove why a boot was not recorded. Sequence numbers are consecutive within
each recorded boot. Boot 1's FILE_OPEN precedes its queued BOOT stamp, so up_ms
goes backward there by design of header emission, not during boot 12 or 13.

Boot 12 has no reboot within its recorded 11-hour-46-minute session. Boot 12
and 13 HEALTH records have zero drops, suppressed records, truncation and slow
writes. Writer stack margin remains 3988 bytes in both. All sampled screen
states during boot 12 are dashboard except the one Live snapshot.
This is sustained ordinary logging evidence, not a substitute for stress tests.

SESSION_END reason=shutdown pending=0 is present for boot 12 at 07:38:20 and
boot 13 at 08:07:37, matching JP's normal shutdowns. File presence supports the
ordinary close path, not a durability guarantee or full power-loss test.

BOOT records 2, 8, 10 and 12 report task_watchdog resets. These precede their
respective sessions; they are not resets during the overnight boot-12 interval.
The supplied log has no panic backtrace or exact triggering action, so do not
assign them to the logger, USB, flashing or PSRAM stack. Keep this as separate
follow-up evidence. Writer/main breadcrumbs are last recorded phases, not
crash stacks; a phase of idle does not clear a component.

## First targeted bench test (completed; result below)

JP's recollection confirms hotspot recovery was part of the morning activity,
but does not establish its timing. Run one controlled Wi-Fi recovery check. Reinsert the card only with the
board off. Start the same B firmware with hooks off; no rebuild is required.

1. Wait for Wi-Fi and real MQTT connected; connect the console and send log status.
2. Request Latest once, wait for the image, return to the dashboard, log status.
   Do not run Live yet, to resemble boot 12's prior media history.
3. Turn the iPhone hotspot OFF for about 90 seconds. Confirm red offline, retain
   the normal probes and send log status while offline.
4. Turn the hotspot ON, wait for green remote MQTT, then send log status.
5. If no sub-20480 probe or functional failure has occurred, run one complete
   Live cycle and send log status.

Send the full serial capture, including mqtt_connect, normal and media probes.
Use actual hotspot controls, not serial off/on, which only redirect MQTT.
Stop at a failure and preserve its evidence; do not restart to erase the state.
A short outage need not reproduce a nine-hour outage, so a pass will narrow the
next test without resolving the overnight failure.

The 20480-byte floor is unchanged; Stage 1 acceptance and hooks stress remain
on hold. USB retrieval is still Stage 1B, after Stage 1 acceptance.
No firmware change, build, flash, commit or push was performed for this analysis.

## Source integrity

Original bytes: 523255. SHA-256: f4af02e8cdce1a646830e94e25bada1db178c2b72da997da985a3ea6cd06e7b9.


## Follow-up: short outage reproduces the memory failure (boot 14)

JP reports normal operation in the 08:20-08:23 test. Latest initially passed
with largest_min=26612, 35085 bytes and 1401 ms total. The logged offline-to-green
interval was about 80.433 seconds; exact hotspot switch times are not logged.

At 08:22:33.412, mqtt_connect run 2 measured largest_min=14324 over 618 ms,
with 62 periodic samples and a 10273 us maximum gap. This is before Live starts.
heap_min_boot fell from 34944 to 22656, a 12288-byte arithmetic change matching
the overnight pattern. Both subsequent Live TLS windows and full Live also
measured 14324. The memory floor fails by 6156 bytes despite successful operation.

The repeat establishes that this short Latest/outage/reconnect sequence can
produce the same low-headroom value, with the first measured drop inside the
MQTT connect window. It does not prove that MQTT owns the obstructing allocation:
other task activity and prior retained allocations remain possible contributors.
It also cannot identify the unobserved 07:08 operation in the original boot.

Pinned core source supplies a candidate: HTTPClient::end calls disconnect(false),
which leaves the underlying client open if _reuse and _canReuse are true.
_reuse defaults true. The image completion path calls end; the screen-unload
path frees image buffers but does not explicitly stop its TLS client.
The next still preparation and Live teardown do stop that client.
This is a retained-connection hypothesis, not enough evidence to change cleanup.

Next control: normal shutdown/restart B with hooks off, no media requests, then
hotspot off about 90 seconds and on. Capture log status before, during and after,
and all mqtt_connect and normal probes. Stop after the reconnect; no Live needed.
This isolates whether prior media history is needed. No new firmware is proposed
until the result distinguishes these paths.

The repeat had unchanged writer used/margin 4204/3988, zero drops and no logger
errors. Video was normal to JP, 170 frames in 60.3 seconds at 2.8 fps.
Measured transfer was slower (89 KB/s, 212 ms transfer); do not attribute the
fps difference to fragmentation without a controlled comparison.
Full capture and measured details are in sd_diagnostics_bench_results.md.
