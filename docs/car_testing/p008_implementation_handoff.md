# P008 implementation handoff

October 3, 2026. Author: Codex. Ready for Claude code review; not hardware-validated.
JP approved [P008](p008_mqtt_tls_limit_design.md) after design clearance b3dc48a.

## Change

Only firmware edits are three constants in src/net/net_worker.h:
TLS_MS 10000 -> 15000, ATTEMPT_MS 45000 -> 50000, STUCK_MS 50000 -> 55000.
Existing symbolic consumers apply the TLS value to native handshake timeout and
phase admission. No worker, retry, lease, memory, protocol or UI logic changed.
DNS/TCP/MQTT/subscription allowances remain 15/5/10/2 s.

## Host validation

Ran every tools/tests/*.test.cjs with Node: **408 checks, 17 suites, all pass**.
Counts: connection_status_ui 7; http_lifecycle 43; http_transfer 35; log_time 20;
media_admission 16; mqtt_owner 54; mqtt_recovery 43; mqtt_service 14;
network_diagnostics 16; operation_diagnostics 8; reader_session 28;
retrieval_mode 42; retrieval_ui 15; sd_log_browser 20; touch_contact 19;
usb_connection_guard 16; usb_logger_gate 12.

Updated owner/recovery fixture budgets without dropping checks. Added one source
simulation advancing DNS + lease + all phases to 47.1 s, leaving 2.9 s, with
unchanged dispatch origin. The allowance boundary loop now includes TLS 15000:
full allowance ending exactly at attempt deadline passes; one ms later refuses.
Stuck test now proves no fault at 54999, fault at 55000, one-shot escalation and
retained lease for both held/not-held cases; stale READY still cannot revive it.
Existing cancellation, retry and admission checks remain passing unchanged.
These checks simulate/examine source; they do not compile ESP32 firmware or
prove native TLS timing, cancellation latency or behavior on cellular networks.

## Review focus and next step

Please verify the small production diff and the tests' new boundary values.
Design review A1-A3 and the approximately 10-hour observation horizon are
integrated in the design approval section. A1's fast field cancellation is
observed evidence, not an unconditional native-call guarantee. Setup's possible
three-attempt extra 15 s is documented; no shutdown wait is introduced.

After Claude clears code, JP builds/flashes the normal profile and collects
ordinary rides. companion.ino is unchanged, so no generated-sketch deletion is
needed for this change. Record the new flashed build boundary. Watch tls_ms >10000
successes, near-15000 failures, service/memory gates and practical media refusal
delays. No specific bench case is issued. No build or flash performed by Codex.

## Claude review

Pending.
