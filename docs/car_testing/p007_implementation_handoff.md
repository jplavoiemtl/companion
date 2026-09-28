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

## Claude code review - September 27, 2026 (ec487bc against 2362ea0)

**Verdict: cleared for JP's build.** No blockers. Host checks re-run: **405 pass in 17
suites**. Not compiled, per the workflow.

- **Counter (S1).** `++status.linkDowns` happens only on `!up && status.link`, inside
  the owner mux and before the unchanged `invalidateLocked`. It is therefore atomic
  with the epoch bump. Duplicate `false` events and GOT_IP while up cannot count twice.
  The `View` grows by 4 bytes, well inside the 2048-byte metadata `static_assert`.
- **Dispatch.** The snapshot is taken before `request()`. A refused request leaves
  baselines and `linkRetryPending` untouched. An accepted one installs
  `attemptLinkId = attemptId+1`, which matches `result.id`, and clears the session
  baseline and the pending flag, so the opportunity is consumed.
- **Races traced through `netMainTick`.**
  - *Stale READY* (epoch changed before `takeResult`): reclassified as cancelled. The
    attempt baseline shows the change, so pending is set and the baseline retained. The
    worker keeps the lease until cleanup, so `request()` cannot dispatch in between.
    The cleanup loss (`observedConnected=false`) re-reads the same attempt baseline and
    schedules `wifi_return` once, then retires the baseline.
  - *Drop between the stale check and `acknowledgeReady`*: the acknowledgement refuses,
    no session is created, and the attempt baseline survives to the cleanup, as above.
  - *Drop after acknowledgement*: the session baseline is the pre-acknowledgement
    snapshot (`readyLinkDowns`), so the later loss sees the change.
  - *Genuine failure*: `clearLinkRetry()` drops pending and both baselines, giving 15 s.
- **Protections preserved.**
  - `scheduleRetry` precedence is restore, then bench_first, then Idle+pending, then
    Idle+stable, then backoff, with a single stamp. That reproduces the old result
    stamp, since `restorePending` and `benchFirstPending` are cleared at dispatch. It
    also reproduces the old loss stamp and adds `wifi_return`.
  - Test results never set pending. Bench non-Idle never earns `wifi_return`.
  - `intentionalDisconnect`, `netConfigureMqttClient` and `netShutdown` clear all P007
    state. Shutdown dispatch remains blocked by the owner's stop check.
  - Failure counting, `giveUp`, request guards, lease and media/retrieval admission are
    untouched.
- **Tests.** The old policy harness was replaced by one that executes the real result,
  loss, dispatch and scheduling bodies. The only other changes are a `linkDowns` field
  in the owner mock and three source-order assertions retargeted from the old inline
  stamp to `adoptResultRetry`. No behavioural assertion was dropped without a
  replacement.

Nonblocking:
- **Misleading log on success.** Every *successful* result also logs
  `MQTT_RETRY_DECISION source=result policy=backoff wait_ms=15000`, although the
  connection is up and the stamp is irrelevant. That can mislead ride analysis.
  Consider skipping the record on `result.ok`, or labelling it `policy=connected`, in a
  later touch.
- **Admission refusals still wait 15 s.** A prompt attempt refused by admission
  (`lease_deferred`, `memory_refused`, `dns_slots_busy`) is a non-cancelled result, so
  it waits 15 s, as the design specifies. When reading rides, separate these from
  genuine network failures.
