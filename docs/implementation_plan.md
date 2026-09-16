# Return to Previous Screen (No Gen-Code Mod)

This plan achieves the same goalâ€”returning to the previous screen after image viewingâ€”but respects the constraint of NOT modifying SquareLine Studio generated files (`ui.h`, `ui.c`, `ui_Screen2.c`).

## User Review Required
> [!NOTE]
> This plan reverts previous changes to generated files and moves logic to `companion.ino` and `image_fetcher.cpp`.

## Proposed Changes

### 1. Revert Generated Files
-   **[MODIFY] [ui.h](file:///e:/DataJPL/arduino/arduino_maker/companion/ui.h)**: Remove `ui_previous_screen` declaration.
-   **[MODIFY] [ui.c](file:///e:/DataJPL/arduino/arduino_maker/companion/ui.c)**: Remove `ui_previous_screen` definition/initialization.
-   **[MODIFY] [ui_Screen2.c](file:///e:/DataJPL/arduino/arduino_maker/companion/ui_Screen2.c)**: Revert `ui_event_Button2` to original state.

### 2. State Management & Logic
#### [MODIFY] [companion.ino](file:///e:/DataJPL/arduino/arduino_maker/companion/companion.ino)
-   **Define Global**: `lv_obj_t* ui_previous_screen = NULL;`
-   **New Handler**: Create `custom_buttonBack_event_handler(lv_event_t * e)`.
    -   Checks `ui_previous_screen`.
    -   Calls `lv_disp_load_scr(ui_previous_screen)` if set, else `_ui_screen_change(...)` or `lv_disp_load_scr(ui_Screen1)`.
-   **Setup Hook**: In `setup()`, after `initUIHandlers()` (or inside it):
    -   Remove the default handler: `lv_obj_remove_event_cb(ui_Button2, ui_event_Button2);`
    -   Add custom handler: `lv_obj_add_event_cb(ui_Button2, custom_buttonBack_event_handler, LV_EVENT_CLICKED, NULL);`

#### [MODIFY] [src/image/image_fetcher.cpp](file:///e:/DataJPL/arduino/arduino_maker/companion/src/image/image_fetcher.cpp)
-   **Extern Declaration**: Add `extern lv_obj_t* ui_previous_screen;`.
-   **Logic**: Keep the logic that sets `ui_previous_screen` in `prepareForRequest` and uses it in timeouts. (This part was valid, just needs the variable to be available).

## Verification Plan
1.  **Revert check**: Ensure `ui_Screen2.c` is clean.
2.  **Functional check**: Same as before (Manual Back, Timeout, G-Meter return).

## Retained MQTT serial diagnostics

The tested MQTT recovery changes and serial controls are retained in the main
firmware for future bench and car diagnostics. The unit starts with its real
broker after every reboot; outage simulation requires an explicit serial command
and is never saved to NVS. The user compiles and flashes through VS Code.

### Serial monitor setup and commands

Open the unit's USB serial monitor at **115200 baud**, enable timestamps, and
select **CR, LF, or CRLF** as the line ending. Type one lowercase command and send
it with Enter. Surrounding spaces and blank lines are accepted.

| Command | Behavior |
|---------|----------|
| `status` | Reports ON, OFF, or RESTORING, Wi-Fi and MQTT connection states, time since `off`, and uptime. ON means the real broker is selected; check the separate MQTT field to confirm a connection. |
| `off` | Requires Wi-Fi and the real MQTT broker to be connected. Disconnects MQTT and redirects its retries to the test address `192.0.2.1` on the current MQTT port. Wi-Fi and HTTPS settings remain unchanged. |
| `on` | Selects the real broker again and makes reconnection eligible. Wait for the green remote-connected message or the serial confirmation before starting another outage cycle. |

`off` simulates an unreachable broker; it does **not** stop MQTT retry work.
The first attempt becomes eligible after 5 seconds, then failed attempts retain
the existing 15-second retry interval. Repeating `off` leaves the current outage
running. There is **no automatic restore timer**: send `on` or reboot to restore
normal broker selection. During RESTORING, another `off` is refused until the
real broker connects. Sending `on` when the real broker is already selected is
harmless. The old `test` and `restore` commands are no longer supported.

### Timing and expected behavior

MQTT reconnect attempts are deferred while a still image is being fetched or
Live is active, so they do not interrupt those operations. An existing MQTT
session continues to be serviced. If `on` is sent during Live, broker selection
changes but reconnection waits until Live ends.

Connection attempts still run synchronously: outside the image/video guard,
the UI and serial commands can pause until an attempt returns. The TCP connect
timeout is 5 seconds; TLS handshake and MQTT CONNACK waits have separate
5-second limits, so an entire attempt is not guaranteed to finish in 5 seconds.
The bench test measured a failed attempt at 5,003 ms instead of 18,282 ms, followed
by a successful real-broker reconnection in 520 ms.

### Repeatable bench procedure

1. Leave the hotspot on. Wait for the green MQTT remote-connected message and
   send `status` to confirm Wi-Fi and MQTT are connected.
2. Send `off`. Expect an OFF acknowledgment and the orange Wi-Fi-connected
   indication. Use Latest or Live for the behavior under investigation; for the
   live interruption test, start Live before sending `off`.
3. Keep the timestamped `[TEST]` attempt BEGIN/END messages, `[NET]` transitions,
   and image/video messages. Use each attempt's internal `elapsed` value when
   serial output arrives in a batch.
4. Send `on`, wait for real-broker connection and the green indication, then
   send `status`. Repeat the cycle as needed.

The Latest button uses HTTPS and can work while MQTT is disconnected. A press
during a blocking attempt may be missed. The test endpoint's behavior depends
on the network; this reproduces MQTT retry interference, not every possible
cellular or car failure. No production MQTT credentials are sent to the test
endpoint.

See [Serial MQTT outage test](mqtt_bench_test.md) for the implementation details
and the before/after bench measurements, including uninterrupted Live with the
reconnect guard and successful Latest requests after recovery.

## Proposed SD card diagnostics

Project owner and bench tester: **JP**.

The [SD card diagnostic logging plan](sd_diagnostics_plan.md) has been
[reviewed and counter-reviewed](sd_diagnostics_plan_review.md). It remains
the accepted design reference. JP authorized Stage 1 after accepting the Step 0
and Stage 0 checkpoint. JP built and flashed Stage 1; initial readiness and normal
operation passed, with the remaining bench gate pending; see the [handoff](../src/diagnostics/STAGE1.md). Before firmware work, the
owner reviews the finalized plan and commits all seven documents listed in its
"Before firmware work" section as the fixed reference. Firmware implementation
starts with measurement probes, then basics and the Stage 1 acceptance gate.
USB retrieval uses the existing serial port and a Web Serial page: Step 0 checks
connection behavior, Stage 1B follows Stage 1 acceptance, and Stage 2 follows
Stage 1B acceptance.

Bench status (2026-09-15): Step 0 passed on the tested board and Chrome with
explicit DTR=true, RTS=false, including reconnecting during Live.
The VS Code monitor close freeze remains unresolved. Stage 0 initial measurement
coverage is complete, including failed and successful MQTT connects. JP accepted
the checkpoint; Stage 1 initial readiness passed, but Latest failed the 20480-byte
memory floor with 14836 bytes. No-card HTTPS recovered to 28660 bytes. The writer-start-order experiment still
measured 14836 bytes with the card installed. Retained startup snapshots passed
late USB retrieval; the writer-creation interval used 6528 internal bytes. The
mount interval overlaps Wi-Fi. Latest still measured 14836 bytes.
Stage 1 acceptance remains on hold. See [bench results](sd_diagnostics_bench_results.md).
JP accepted the memory experiment review and built both stack variants with hooks
off. Latest measured 14836 bytes in A and 26612 in B; B passes the initial
20480-byte memory check. Both used 4204 bytes of stack, with correct placement.
On 2026-09-16, B completed full Live normally after about 11 hours 43 minutes of
uptime, but both TLS windows and full Live measured 14324 bytes, failing the
unchanged floor. The pre-Live status already retained this minimum.
Fresh-boot B passed full Live at 25588 bytes, with TLS minima 25588 and 27636,
normal video and no logger errors or drops. The overnight cause is unresolved.
Three more same-boot Live cycles passed at 25588, 25588 and 26612 bytes with
normal video and no logger errors or drops. Stage 1 acceptance remains on hold.
G-meter selection and both ordinary preference saves passed; following Live
measured 26612 bytes, with responsive controls, normal video and no logger errors
or drops. Inclinometer navigation, both saves and following Live also passed
at 25588 bytes, with unchanged stack use and no errors or drops.
Latest-to-Live passed at 26612 bytes in both operation windows, with unchanged
stack usage and no logger errors or drops. Normal shutdown and
card-reader retrieval were completed. The [overnight analysis](sd_diagnostics_overnight_analysis.md)
found a transient low between 07:07 and 07:08 while sampled offline, followed by
smaller contiguous blocks at the 07:34 recorded reconnection. A short hotspot
outage after Latest reproduced 14324 inside mqtt_connect and subsequent Live.
The fresh-boot no-media control passed at 31732; outage duration was shorter.
JP approved the explicit TLS close after still-image body receipt. The single
stop call is implemented in image_fetcher.cpp; JPEG buffers, Live reuse, probes
and timeouts remain unchanged. JP built and tested it in boot 17: Latest measured
25588 bytes and MQTT reconnect 31732, both above 20480, with no errors or drops.
The following full Live cycle in the same boot also passed: TLS minima 26612 and
25588, full-Live minimum 25588, 196 frames in 60.3 s and normal video reported by JP.
No errors or drops; writer stack margin stayed 3988 bytes. Manual early Live exit
and hotspot loss during Live also passed, including subsequent Latest retrieval.
Back passed at 26612 bytes and 1427 ms. Automatic camera still-to-Live also passed:
lowest block 25588, first frame 1036 ms, 195 frames in 60.3 s, normal video.
The targeted cleanup checks are complete; review this checkpoint before the
remaining Stage 1 gates. The overnight event's exact cause remains unresolved.
The same-session logging-OFF/ON pair passed on 2026-09-16: Live fps decreased
3.46% (within 5%), ON media block minima were 26612, and normal IMU average was
49.18 versus 49.26 Hz. No logger errors or drops; stack margin stayed 3988.
The hooks-only NVS/SD runtime stress also passed: 283 flash commits and 993 SD
records, no errors or drops, dummy key removed, writer stack margin 3476.
Card inspection verified all 993 stress records, the final counters and normal
SESSION_END with pending=0; a byte-identical backup is preserved. It also exposed
startup watchdog resets before boots 19 and 21's successful timed tests.
JP reproduced the pre-existing VS Code monitor-close freeze without flashing:
UI remains frozen over 30 seconds; the browser opens but status commands get no
reply. No automatic restart was observed. This remains separate from the earlier
watchdog markers. Use USB recovery and browser-only serial monitoring while the
host/core issue is investigated; no USB or watchdog settings were changed.
JP confirmed recovery in boot 23. Normal-limit forced rotation passed in boot 24:
one archive, generation 2, new file growth 449 to 1008 bytes, no errors/drops,
writer margin 3412. Small-limit rotation/pruning also passed: generation 6,
three archives, two pruned, no errors/drops and unchanged stack margin. Normal
limits were restored by command. Simulated full-card failure then passed:
logger disabled/parked as intended, final writer stack margin 3412, Latest still
working at 26612-byte HTTPS minimum and 1274 ms completion. Reboot recovery
also passed in boot 25: ready, generation 6 and three archives retained, file
growth 4108 to 4664. Interrupted-rename recovery then passed in boot 26:
JP confirmed the pause; generation 7, four archives and file growth 1259 to 1815
were observed. Partial-header salvage then passed its runtime check in boot 27:
generation 9, six archives, file growth 1284 to 1840, no errors/drops.
The supplied card copy now confirms archives 3 through 8, retained sequence
continuity, boot-25 append, boot-26 rename recovery, and boot-27 salvage into
generation 9. Archive 8 preserves the deliberate partial prefix plus a shutdown
record. Current ends with a clean shutdown at 15:11:53, pending=0.
Byte-identical copies and hashes are retained in docs/bench_data/sd_logs_2026-09-16_1511/.
The empty-header hook and intentional panic then passed runtime recovery in
boot 29: generation 10, seven archives, file growth 1397 to 1953, no errors/drops.
Paused file_bytes was a stale snapshot, not a measurement of the new empty file.
The recovery header and reset breadcrumbs await the next combined card inspection.
Next is the controlled watchdog test; remaining storage and clock/reset gates
are still pending.
Stage 1 acceptance remains pending.
JP's local switch is 1 for diagnosis; the intended default remains 0.
