# P007 implementation handoff for Claude

September 27, 2026. Author: Codex.
Status: implemented after JP approval; awaiting Claude code review before JP builds.
Design: [revision 2](p007_link_return_retry_design.md), incorporating approved S1.
Review base: 2362ea0 on main. No firmware compilation, flash or hardware test performed.

## Result and scope

A real MQTT session lost across a usable WiFi down transition, or an accepted attempt
cancelled across that transition, now earns prompt eligibility independent of ONLINE
age. It still waits for usable link and completed cleanup. A broker-only short session
and a genuine failed attempt retain 15 seconds. The >=60 s broker-only prompt rule,
bench precedence, initial failure budget, shutdown stop guard and resource admission
remain unchanged. This reduces avoidable idle time; it does not fix beacon loss.

Changed firmware files:
- src/net/net_worker.h: one uint32_t linkDowns field in View.
- src/net/net_worker.cpp: increment on !up && status.link within the existing mux,
  before the unchanged epoch invalidation. No worker execution/protocol changes.
- src/net/net_module.cpp: main-owned attempt/session baselines and one pending flag;
  pre-request and pre-READY snapshots; terminal retry scheduling and bounded
  MQTT_RETRY_DECISION logging; clearing on intentional disconnect/configure/shutdown.

There is no added observer lock, task, allocation or socket. The View field may use
existing alignment padding depending on the target ABI; do not infer its compiled
size from these host checks. The owner's existing static metadata-size assertion
remains in place for JP's build. companion.ino and generated UI files are untouched;
no generated sketch deletion was needed or performed.

## Review focus

1. Verify serial and epoch update under the same owner mux. Duplicate false events
   and repeated GOT_IP do not create extra link transitions.
2. Check snapshots occur before accepted request / READY acknowledgement. Refused
   dispatch retains eligibility; accepted dispatch consumes it. A racing full cycle
   is conservatively included, as the approved design specifies.
3. Trace cancelled Result and revoked READY cleanup. A drop after the stale check
   can make acknowledgement fail; the retained attempt baseline lets cleanup detect
   it. Pending eligibility survives the two terminal messages. Genuine failure clears
   both pending eligibility and baseline; old cycles cannot override its 15 s wait.
4. Check exclusive scheduling precedence: restore, first test, normal link return,
   normal stable session, backoff. Existing stop/busy/link/lease/mailbox/fault request
   gates still own admission. No direct main-side MQTT access or forced cleanup added.
5. Logging occurs only on terminal scheduling decisions. policy=wifi_return means
   eligibility is prompt, not that BEGIN must occur immediately while offline or busy.
   Successful Result adoption also records its default scheduling decision before
   READY adoption, as specified; connected status continues to report no backoff.

## Host verification

All 17 tools/tests/*.test.cjs suites were run with node: **405 checks pass** (386 + 19).
These execute adapted source bodies and static contracts; they are not a C++/ESP32
build, concurrency stress test, ABI proof or hardware validation.

| Suite | Checks |
|---|---:|
| connection_status_ui | 7 |
| http_lifecycle | 43 |
| http_transfer | 35 |
| log_time | 20 |
| media_admission | 16 |
| mqtt_owner | 53 |
| mqtt_recovery | 41 |
| mqtt_service | 14 |
| network_diagnostics | 16 |
| operation_diagnostics | 8 |
| reader_session | 28 |
| retrieval_mode | 42 |
| retrieval_ui | 15 |
| sd_log_browser | 20 |
| touch_contact | 19 |
| usb_connection_guard | 16 |
| usb_logger_gate | 12 |

mqtt_recovery expands from 22 to 41 checks. Its harness now executes the real result
adoption body and scheduling helpers as well as the loss and dispatch paths, with
mock owner request admission using the actual source guard. The new checks cover:
short-session link return, offline/refused dispatch, cancellation with/without link
loss, stale and raced READY cleanup, request snapshot races, coalesced bursts, new
cycles during replacement attempts, genuine-failure backoff, new-session history
reset, serial wrap, matching attempt IDs, bench precedence, intentional resets,
shutdown, failure budget and bounded decision records. Existing stable-session,
profile-selection, deadline and wraparound assertions remain.

Two other suites had location-only changes: mqtt_owner and network_diagnostics used
to search for the inline lastMqttAttempt assignment. They now follow adoptResultRetry
and scheduleRetry, retaining the completion-after-cleanup/probe ordering checks.
The owner suite also initializes linkDowns=0 in its mock View. No unrelated assertion
was removed. git diff --check passed.

## Validation and next decision

Claude reviews this implementation against the approved design. After clearance JP
may build and flash; neither assistant does so. No new bench campaign. Use ordinary
rides to assess link-return-to-BEGIN, failure-to-next-BEGIN, total outages, service
and memory gates, and IMAGE_REFUSED/LIVE_REFUSED reason=mqtt_reconnecting frequency.
Counterexamples should separate delayed admission from retry waiting. A ride without
a relevant link loss does not exercise P007 or prove a WiFi fix. Keep P004 deferred.

## Claude review

Pending.
