# P009 implementation handoff

October 8, 2026. Author: Codex. Ready for Claude code review; no firmware build,
flash or hardware validation. JP approved the [design](p009_live_connect_diagnostics_design.md)
after Claude clearance 718fff9 / 125fc18. Implementation baseline: 125fc18.

## Changes

- Live ensureConnected measures the existing resolver, IP TCP/setup overload and
  native startTLS separately. The original hostname and CA reach the IP overload.
  TCP remains 5000 ms and TLS 5 s; DNS has no application deadline. The source
  documents main-loop blocking of about 10 s plus DNS and local overhead.
- The existing image client is now a tiny MediaSecureClient subtype exposing a
  protected flag reset. No added client state, allocation, virtual override or
  second TLS client. Image source changes are only the global/accessor types.
- MediaPlainStartGuard clears the sticky flag on both native failure paths and
  success. An early exit between successful TCP and TLS closes the plaintext
  socket first. Native failures already close, so cleanup does not close twice.
  Encrypted keep-alive remains reusable; a flagged plaintext connection cannot
  pass the fast path. No HTTP bytes are sent before successful TLS.
- One endLiveConnect completion emits matching NET_END and LIVE_CONNECT fields:
  dns_ms, tcp_ms, tls_ms, phase_valid (bits 1/2/4), failed_phase and error freshness.
  TCP/setup failures capture lastError immediately. Native startTLS does not
  refresh it, so TLS failure is explicitly code=0, tls_queried=0,
  tls_fresh=unavailable, failed_phase=tls. Zero is not success evidence.
  Live serial output uses this snapshot too. Generic Span::end is unchanged.

No retry, timeout, worker, MQTT, UI or still HTTPClient-flow changes. P004 remains
deferred. The subtype relies on protected _stillinPlainStart in the installed
3.3.11 target and 3.1.3 rollback sources; neither profile was compiled here.

## Host validation

All **426 checks in 18 suites pass** with Node over tools/tests/*.test.cjs.
Existing 408 checks retained; new live_connect.test.cjs adds 18. The existing
network suite now checks the explicit split instead of requiring the old combined
connect call; its still-transport and timeout checks remain.

Counts: connection_status_ui 7; http_lifecycle 43; http_transfer 35;
live_connect 18; log_time 20; media_admission 16; mqtt_owner 54; mqtt_recovery 43;
mqtt_service 14; network_diagnostics 16; operation_diagnostics 8;
reader_session 28; retrieval_mode 42; retrieval_ui 15; sd_log_browser 20;
touch_contact 19; usb_connection_guard 16; usb_logger_gate 12.

The new suite adapts the actual C++ orchestration, guard and completion bodies
to JavaScript with a native-behavior mock. It covers all phase outcomes, skipped
phase bits, fresh/stale error handling, failed TCP/TLS followed by still TLS,
early cleanup, keep-alive, unexpected plaintext state, hostname/CA/timeouts,
one completion pair, breadcrumb restoration and diagnostic-disabled cleanup.
A negative control reproduces the plaintext leak without the guard. Maximum-width
record formats stay under 456 bytes. Source checks preserve the still body and
generic Span end exactly. These are host simulations/contracts, not C++ compilation
or proof of real certificate validation/network timing.

## Claude review focus and next step

Please check shared-client cleanup against both native failure paths, hostname/CA
preservation, truthful unavailable TLS errors, and the revised host assertions.
The design approval section narrows review A4: near-limit versus fast failure
does not alone prove remote silence versus active refusal.

After Claude clears the code, JP builds/flashes the normal 3.3.11 profile and
records its build/deployment boundary. companion.ino is unchanged; this change
does not require deleting its generated sketch. No hardware case is issued now.
Ordinary rides provide the planned validation, with the next connection failure
read by phase/duration and correlated with server evidence. A successful connect
followed by response timeout remains a separate issue. No further change is
automatically authorized by this diagnostic increment.
