# Serial MQTT outage test

Retained serial diagnostics in the main firmware, developed and validated on
`codex/mqtt-outage-bench-tests` from `9c89ef8`. See also the
[project plan](implementation_plan.md#retained-mqtt-serial-diagnostics) for the command reference.
The user compiles and flashes through VS Code. No automatic upload or serial-port
control is required. The current revision applies the TCP timeout correction below.
The tested first correction defers MQTT reconnect attempts during live video,
extending the existing still-image guard. Established MQTT sessions are still serviced.

## Commands

Use 115200 baud and CR, LF, or CRLF as the serial monitor's line ending. Commands
are lowercase. Surrounding spaces and blank lines are accepted. Input is bounded
and non-blocking; overlong commands are discarded as a whole.

- `status`: report ON, OFF, or RESTORING, Wi-Fi/MQTT connection status, time since
  the current outage began (zero in ON mode), and device uptime. ON means the real
  broker is selected; the separate MQTT field reports whether it is connected.
- `off`: requires working Wi-Fi and an already-connected real broker. Disconnect
  MQTT and redirect only its client to 192.0.2.1 on the current MQTT port, using
  the existing TCP/TLS settings. Keep doing real failed connection attempts until
  `on`; this does not suspend MQTT retries. The first attempt is eligible after
  5 seconds; subsequent failures retain the existing 15-second retry interval.
  No production MQTT credentials or client ID are used at the test endpoint.
- `on`: select the real broker again. It is harmless if that broker is already
  selected. Restoration makes the real broker immediately eligible, subject to
  the still-image and live-video busy guard. It does not guarantee instant connection.

There is no 45-second timer or automatic restoration deadline. Commands run on
the main loop: they cannot interrupt a blocked connection call. Logs show actual
attempt durations and when `on` is processed. Compare the monitor's Sent timestamp
with the ON acknowledgment to measure a queued command's delay.
Repeated `off` commands leave the existing outage running without restarting its
retry timer. During RESTORING, wait for the real broker to reconnect before another
`off`. A reboot starts with the real broker; no test state or endpoint is persisted.
The earlier `test` and `restore` commands have been replaced, not kept as aliases.

## What this measures

192.0.2.1 belongs to the documentation-only TEST-NET-1 block specified by
[RFC 5737](https://www.rfc-editor.org/rfc/rfc5737.html). On the user's first timed
bench run, two attempts to this address each failed after 18,501 ms (MQTT state
-2). Automatic restoration requested for 45 seconds occurred at 57,210 ms because
an attempt was blocking, and the real broker then connected in 486 ms. This is
measured evidence of long blocking attempts on that network, not proof of the
original car failure or a guarantee of identical behavior on other networks.

This setup isolates reconnect attempts while Wi-Fi and HTTPS remain available;
it does not simulate cellular loss or a lost existing TCP session. An unexpected
successful MQTT connection to the test destination aborts the experiment and
restores the real broker. Simulated failures do not consume the startup give-up
budget. No artificial sleep is added to mimic a hang.

Existing image logs measure accepted button presses through HTTP and rendering.
A touch missed while the main loop is blocked will not produce a button log;
physical observation is still needed to identify that symptom.

## Initial verification on the board

1. After setup, send `status`. Expect ON and both Wi-Fi/MQTT connected.
2. Send `off`. Expect an OFF acknowledgment, orange Wi-Fi-connected status,
   and connection-attempt BEGIN/END logs. MQTT stays unavailable until `on`.
3. Send `on`. It may wait for the current blocking attempt to return. Expect
   the ON acknowledgment, a real-broker attempt, and the ready message.
4. Send `status` again. Expect ON and MQTT connected. Another cycle is allowed.

The assistant reviewed the source and checked the diff but did not compile or
flash. The user compiled/flashed and validated both timed and manual off/on modes,
including successful reconnection to the real broker.

## Live-feed reconnect guard validation

The baseline live-video test ended its feed with a response timeout immediately
after an 18,195 ms MQTT connection attempt. With the guard added, start Live while
MQTT is connected, send `off`, and let the feed finish naturally. No MQTT reconnect
attempt should begin while the feed is active. Attempts become eligible after it
ends. Send `on` to restore the broker; a blocked attempt must still return first.
If `on` is sent during the feed, the real broker is selected immediately but its
reconnection also waits until the feed ends. This guard was validated before the
TCP timeout correction.

The user validated the guard with uninterrupted video: 201 frames over 60.8 seconds
at 3.3 fps. The first MQTT attempt began only after the feed ended and still took
18,282 ms with the original timeout configuration. A queued `on` was then handled
and the real broker connected in 487 ms. This is the baseline before correcting
the TCP timeout setter.

## TCP timeout correction validated on the bench

The guard and original-timeout baseline were committed as `f32bc97`. The timeout
correction uses `setConnectionTimeout(5000)` on both MQTT transports and on the shared
secure client before Live connects. ESP32 core 3.1.3 implements this setter
separately from the inherited Stream `setTimeout`. The five-second TLS handshake
and MQTT CONNACK limits remain unchanged; the complete connection attempt is
not guaranteed to finish within five seconds if those later stages are reached.

The user compiled and flashed this correction in VS Code. On the dashboard,
the failed test-endpoint attempt took 5,003 ms, down from the 18,282 ms baseline.
An `on` command sent during the attempt was handled when it returned, and the
real broker reconnected in 520 ms. Connection attempts remain synchronous.

A subsequent visual check passed: Live displayed 33 frames over 10.8 seconds
at 3.0 fps and stopped normally when the screen was left. Latest then displayed
successfully in 1,278 ms. No compile or flash was performed by the assistant.
