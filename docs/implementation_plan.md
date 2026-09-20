# Return to Previous Screen (No Gen-Code Mod)

**2026-09-17: JP accepted Stage 1 and authorized Stage 1B USB retrieval.**
The [checkpoint](sd_diagnostics_stage1_checkpoint.md) records the accepted scope and limitations.
Earlier pending-acceptance statements below are historical.
Stage 1B was accepted by JP on September 19. JP accepted Stage 2 on September 20. Stage 3 operation-context instrumentation is prepared for build/bench.
See the [September 19 checkpoint](sd_diagnostics_checkpoint_2026-09-19.md) for current state.

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

**Current checkpoint (2026-09-19):** Stage 1 is accepted. Stage 1B transport,
storage/close/NVS regression, resource and card-prefix tests pass. JP accepts
the rare concurrent-download Live FPS tradeoff and the newer profile's 42-43 Hz
IMU rate. Core 3.3.11 logging on is restored with test flags off. TLS/memory
review and paired Live checks pass. JP explicitly accepted Stage 1B on
September 19, 2026; checkpoint `f899e3c` is committed and pushed. Stage 2 is built/flashed by JP; boot-84 record/health/download checks pass, with the prior watchdog confirmed by JP as following monitor closure. Further [bench cases](../src/diagnostics/STAGE2.md) remain. See the [resume checklist](sd_diagnostics_checkpoint_2026-09-19.md).
The dated findings below are historical; their earlier hold status is superseded
by the accepted Stage 1 checkpoint.


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
Evening SD inspection corrects this coverage: reason=new confirms missing-file
recovery after the empty-header pause; surviving-empty-file recovery is still untested.
Controlled watchdog triggering, restart and continued logging passed in boot 30:
diag_wdt timed out after about 5 seconds; the logger returned ready at generation
10. Follow-up confirmed file growth 80950 to 82629 and writes 7 to 10, no errors
or drops, largest block 31732 and writer margin 3588. Evening SD inspection verified
panic and task_watchdog reset classes and both valid main/writer breadcrumbs. The spring clock hook then passed
runtime checks: synced -> approx -> synced, file growth 85436 to 87852,
writes 15 to 23, no errors or drops and unchanged memory/stack minima.
The autumn hook also passed runtime checks: synced -> approx -> synced,
file growth 88414 to 90798 and writes 24 to 32, with no errors/drops and unchanged
memory/stack minima. Evening SD inspection verified both DST offsets, real-time
restoration, approximate-to-synced correction records and the final clean shutdown.
Eight files and hashes are backed up in docs/bench_data/sd_logs_2026-09-16_1802/.
No-card startup and PSRAM-writer terminal cleanup then passed in boot 31:
expected mount failure, parked writer, final stack margin 4116, Latest successful
in 4722 ms with HTTPS largest block 26612. JP confirmed normal board operation.
JP confirmed E:\sdcard as the actual card. Its logs matched the evening backup.
The 92055-byte current file is preserved as archive 10, and a flushed, closed
zero-byte current.log was prepared. Boot 32 then passed runtime recovery:
generation 11, eight archives, file growth 1257 to 1814 and writes 8 to 9, no
errors/drops, largest block 31732 and writer margin 3588. On-card
reason=empty_recovery still needs verification. The next morning's small-limit
attempt began with a 469869-byte current file, above the below-7000 precondition.
It pruned eight backed-up archives, then disabled on the 32768-byte content budget
with reserve_exhausted before rotating. Current remained 470005 bytes; the writer
parked with margin 3412. The initially offline hotspot was not the cause.
Normal reboot recovered logging in boot 33: ready, generation 11, current
470855 bytes (850 bytes appended), five writes, no errors/drops and largest
internal block 31732. Clock was still unknown at the early 17.5-second status.
Card inspection then verified generation 11 reason=empty_recovery, contiguous
boot-32/33 records, sync at boot-33 uptime 19372 ms and clean shutdown.
The 472389-byte log is backed up in docs/bench_data/sd_logs_2026-09-17_0828/
and preserved on card as archive 11. A flushed empty current.log is prepared.
Boot 34 starting status is verified: ready, synced, generation 12, 1276 bytes,
one archive, no errors/drops and largest internal block 31732. The small hook was
accepted; idle fill then produced generation 13 and rotations=1 before restoring
normal limits. Runtime rotation passes: no errors/drops, largest block 31732,
writer margin 3380. Normal restoration was accepted and logging continued.
Card inspection verified archive 12 at 8136 bytes, current reason=size,
continuous boot-34 sequences 1-31 and clean shutdown. Natural rotation content
verification passes. Originals are backed up in docs/bench_data/sd_logs_2026-09-17_0852/.
The 56-byte incomplete-tail fixture recovered in boot 35: ready, generation 13,
file growth 5099 to 5939, writes 7 to 10, unknown -> synced, no errors/drops,
largest block 31732 and writer margin 3588. Runtime check passes; on-card
TAIL_RECOVERY and prefix preservation await a combined inspection. JP requested
a finite remaining checklist: tail content, unrelated-file protection/no-clock
rotation and deep-sleep close/wake. JP explicitly deferred bad/unsupported-card
testing for version 1 on 2026-09-17; retain this limitation. The 2026-09-17 09:31 card inspection confirms short-tail repair with the original
prefix intact, complete sequences across boots 34–37, and clean deep-sleep close.
Boot 37 records deep_sleep reset, wake_code=2, both retained sleep/close breadcrumbs,
approximate time followed by sync (-118 ms), and a final clean normal shutdown.
JP confirmed touch-only wake and normal operation. These checks pass.
Backups and the preservation fixture are in
[09:31 evidence](bench_data/sd_logs_2026-09-17_0931/README.md).
Current was preserved as archive 13; an empty current and hashed unrelated files
are ready for the remaining preservation and unknown-clock rotation test.
Boot 38 passed the unknown-clock rotation runtime check: three rotations, two
prunes, generation 17, three archives, no errors or drops. The normal-limit hook
was sent and processed; logger stayed healthy. Final card inspection confirms
three FILE_OPEN reason=size records with unknown time, continuous sequences 1–61,
and clean shutdown. All six protected-file names and hashes match.
The agreed bench sequence is complete. See the
[Stage 1 acceptance checkpoint](sd_diagnostics_stage1_checkpoint.md).
JP acceptance remains pending; no further Stage 1 runtime repeat is proposed.
Do not repeat passed tests or start Stage 1B before acceptance.
Earlier, the large-file/small-limit run disabled logging and rejected restoration.
Subsequent guarded runs verified natural rotation and restoration. The original
header-hook reset yielded reason=new; the separately prepared surviving-empty-file
fixture later verified empty_recovery. Both histories remain in the bench record.
Stage 1 acceptance remains pending.
JP's local switch is 1 for diagnosis; the intended default remains 0.
