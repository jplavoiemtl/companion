# P005/P006 - MQTT recovery timing and joined-network profile

Revision 2 - September 25, 2026. Author: Codex.
Status: design for Claude review, then JP implementation approval. No firmware changes,
builds or flashes. Source checked at 092b32c and f5a25b7 on codex/car-improvements-p001;
the intervening changes are documentation only. Integrates Claude review f5a25b7.

JP approved the direction: P005 A+B and P006 in one increment, P004 afterward.
This document specifies that increment; direction approval is not implementation approval.
Inputs: [Claude proposal](p005_recovery_timing_proposal.md),
[F003 and review](field_journal.md), and [P001](p001_mqtt_responsiveness_design.md).

## 1. Review conclusion and evidence corrections

Agree with the direction. P001 removed MQTT connect work from main; recovery now pays
an avoidable initial wait, and some successful protocol phases approach their limits.
There is a real requested/associated profile mismatch. None requires changing WiFi radio
policy or moving image work in this increment. The following corrections matter when
setting expectations; Claude's original proposal/review are preserved as historical input.

Recomputed from evidence/2026-09-25-ride-3/
start-unknown_49-1-current-524908.log, boots 45-49. Times below use same-boot up_ms,
from the driver beacon-timeout record, not wall-clock subtraction.

| Boot/local loss | GOT_IP after loss (s) | First MQTT BEGIN after loss (s) |
| --- | ---: | ---: |
| 45 / 15:27:16 | 13.243 | 15.015 |
| 45 / 15:28:34 | 3.571 | 15.022 |
| 45 / 15:39:16 | 13.194 | 15.008 |
| 46 / 16:09:57 | 3.546 | 15.011 |
| 46 / 16:11:05 | 5.437 | 15.007 |
| 47 / 18:27:32 | 3.633 | 15.009 |
| 47 / 19:11:35 | 3.643 | 15.007 |
| 47 / 19:12:21 | 25.250 | 25.257 |
| 49 / 20:41:30 | 3.641 | 15.014 |

Thus six, not seven, of nine recovered IP in 3.5-5.4 s. Eight began near 15 s;
the ninth was link-limited. A removes about 9.6-11.5 s of idle wait in those six,
about 1.8 s in two, and essentially none in the last, assuming unchanged other work.
These are counterfactual scheduling savings before the revision-2 stability filter,
not promised recovery times. Revision 2 grants prompt retry only after >=60 s ONLINE;
short-lived sessions deliberately retain the first wait even when IP returns quickly.

At 20:26, WiFi stayed associated. Three attempts took 8042, 5131 and 1011 ms,
with approximately three 15 s waits, producing the 59.209 s MQTT outage. Removing
only the first wait would give about **44.2 s, not 30 s**, with identical outcomes
and phase durations. B might avoid a failed attempt, or instead spend longer failing.
The logs cannot establish that a TLS failure at 5003/5004 ms would succeed at 10 s.
Successful TLS 4975 ms and MQTT exchange 4986 ms justify trying more allowance;
TCP's 3903 ms maximum success does not justify changing lifetime socket timeouts.

F003's 14 post-startup worker attempts kept UI service gaps <=23 ms and IMU/loop gaps
<=25 ms. Memory/stack minima during MQTT were 47092/7228 bytes. These are measured
baselines, not the new acceptance limits. Separately, the review's image-gap count
needs correction: seven of eleven explicit LOOP_GAP records are <=1.4 s, two are
1.784/1.969 s, and two are 5.192/5.264 s. P004 remains warranted after this increment.
Neither P005 nor P006 fixes the underlying beacon losses or identifies a carrier/broker fault.

## 2. Source checks and boundaries

- net_module.cpp: netMainTick adopts loss, stamps lastMqttAttempt, then clears
  observedConnected. This is the correct place for A; GOT_IP is not.
- netCheckMqtt admits through the existing rate limit and owner request. Main loop
  still defers dispatch while images/Live are busy. Owner requires link, cleanup,
  no outstanding result/loss, and no held lease before accepting another request.
- net_worker.cpp: DNS precedes lease; split TCP, TLS, MQTT and subscription phases
  use admitPhase and the original dispatch clock. READY retains its lease until
  main accepts the matching epoch. Cancellation/cleanup ownership stays unchanged.
- OwnedPubSubClient.cpp: both the CONNACK wait and readByte consult socketTimeout;
  raising only the outer phase allowance would leave a five-second inner timeout.
- Installed core 3.3.11 NetworkClientSecure/src/ssl_client.cpp stores the connect
  timeout in socket_timeout (line 116), installs receive/send timeouts (171-172),
  and uses it for write progress (453). Handshake timeout is separate (335), with
  vTaskDelay(2) between WANT_READ/WANT_WRITE iterations (338).
- companion.ino: successful connectToWiFi currently configures the requested
  connection; the late-boot path selects from WiFi.SSID(). F003 boot 46 associated
  primary but configured connection 2. The selection defect is independently confirmed.
  The proposal's CAR/HOME endpoint comparison is supporting context from Claude;
  correctness must not depend on two profiles having equal endpoints.

Target remains the accepted amoled-1-8-core-3-3-11 profile. No 3.1.3 branch,
new task, extra TLS client, buffer, authentication change, or WiFi scanning change.

## 3. P005 A - one prompt attempt after a stable established-session loss

Add PROMPT_MIN_ONLINE_MS=60000 and a main-owned uint64_t onlineAdoptedAtMs, using
esp_timer_get_time()/1000. Set it only in the real READY adoption branch, after
acknowledgeReady and confirmation that netIsMqttConnected() is true, alongside
observedConnected=true. Do not stamp it on worker completion, GOT_IP, an unadopted
READY, or repeated status observations. observedConnected is its validity flag; zero
is not a special timestamp. Each newly adopted real session starts a fresh interval.

At takeLoss adoption, capture the monotonic main timestamp and snapshot
observedConnected before clearing it. Only BenchPhase::Idle, observedConnected=true,
and lossAdoptedAtMs-onlineAdoptedAtMs >=60000 together earn prompt eligibility:
backdate lastMqttAttempt by MQTT_RECONNECT_INTERVAL. A session of 59999 ms does not;
a session of exactly 60000 ms does. Measure READY adoption to loss adoption as agreed,
not inferred RF uptime. Main delivery delay is therefore included in this duration.
Continue emitting MQTT_LOST and clear observedConnected exactly as today; clear the
stored timestamp on loss and intentional disconnect for clarity. No credit counter or
accumulation across sessions is needed. Repeated short accepted-then-dropped sessions
all keep the normal 15-second wait, preventing repeated prompt TLS handshakes.

Scheduling priority is explicit, mutually exclusive:

1. restorePending: retain existing immediate bench restoration.
2. benchFirstPending: retain existing five-second first test-attempt delay.
3. established normal loss after >=60000 ms ONLINE: immediate eligibility.
4. Short-session losses and other cleanup/loss acknowledgements: normal 15-second stamp.

Do not subtract multiple intervals if flags overlap. Use the existing unsigned
millis subtraction idiom; host checks include wraparound.

Eligibility is not a promise of immediate network I/O. With link down it waits for IP;
with media active or cleanup pending it waits for existing admission. Do not bypass
those guards, preempt media, or introduce a GOT_IP settle delay. Repeated GOT_IP while
up remains ignored by the owner. Link flapping before dispatch does not consume extra
attempts; flapping during an attempt invalidates its epoch and waits for worker cleanup.
A failed or cancelled attempt stamps the normal 15-second retry interval on result
adoption, including memory/lease deferrals. Revoked READY that was never adopted is
not an established connection and earns no prompt retry. A later genuine success and
loss may earn another prompt attempt only after its own >=60 s ONLINE interval. Startup
and the initial five-failure budget stay unchanged. Deliberate bench disconnects clear
observedConnected and earn no prompt eligibility. During shutdown a loss adoption may
backdate the retry stamp; the required invariant is no dispatch because owner request()
refuses while stop is set. Do not add shutdown state solely to prevent an unused stamp.

A mid-attempt flap still incurs 15 s after cancellation adoption, even if IP returns
sooner. This is an intentional anti-storm trade-off, not a failed prompt retry in field
analysis. The 60-second rule also limits the proposal's retrospective timing savings.

The interval currently starts at main result adoption, slightly after owner completion.
Keep that conservative behavior; do not claim exactly 15 s from the worker timestamp.
No retry timer is reset by GOT_IP. No automatic broker failover is introduced.

## 4. P005 B - separate connect allowances from ONLINE limits

Use named worker constants so tests and phase calls share the same policy.

| Limit | New value | Applies to |
| --- | ---: | --- |
| DNS_MS | 15000 ms, unchanged | bounded DNS wait and retained-slot rules |
| TCP setup | 5000 ms, unchanged | plain/secure setConnectionTimeout and TCP allowance |
| TLS handshake | 10000 ms | secure.setHandshakeTimeout(10), TLS allowance |
| MQTT exchange | 10000 ms | MQTT allowance and connect-only PubSub socket timeout |
| Subscriptions | 2000 ms, unchanged | subscription writes, not broker SUBACK proof |
| ONLINE operation / PubSub timeout | 5000 ms / 5 s, unchanged | loop, publish, packet reads/discard |
| ATTEMPT_MS | 45000 ms | original dispatch origin through attempt completion |
| STUCK_MS | 50000 ms | visible fault/retained ownership if cleanup does not return |

Set PubSub socketTimeout to 10 only in a tightly scoped connect exchange. Restore to
5 immediately after connect returns, before subscriptions, READY publication, result
handling or any continue. Prefer a small worker-local scope guard whose destructor
restores 5 on every exit. If CONNECT fails, is cancelled, or is refused by its operation
guard, restoration still happens. If native code has not returned, there can be no
concurrent ONLINE use: this worker remains the sole owner. Both secure and plaintext
MQTT exchanges receive the ten-second allowance; lifetime TCP/read/write settings stay 5 s.
Do not change OwnedPubSub's default globally or loosen its oversized-packet rules.

The absolute operation guard still bounds the entire exchange, including partial
CONNACK reads; readByte must not grant ten additional seconds per byte. Preserve tick
delays in all active polling loops, ten-ms idle/ONLINE cadence and bounded sampling.
Attempt admission still requires the whole next phase allowance to fit under the
original dispatch deadline; neither DNS completion nor GOT_IP resets that deadline.
DNS tombstones, ERR_MEM handling, cancellation and late callback safety are unchanged.

45 s = 15 DNS + 5 TCP + 10 TLS + 10 MQTT + 2 subscriptions + 3 scheduling/cleanup
allowance. Nominal lease-covered phases grow from 17 to 27 s. These are cooperative
budgets, not guaranteed hard wall-clock maxima: native calls may return late and READY
adoption/cleanup add time. Main never forcibly closes the worker's transport or frees
its stack. At 50 s the existing visible stuck fault holds exclusion; late cleanup or
restart is still the recovery path. Do not describe this as a hard 27 s lease bound.

## 5. P006 - select from the actual joined network

Use one main-task helper for the initial-success and late-boot configuration paths.
Take one WiFi.SSID() snapshot while connected. Match primarySsid first and return
primaryNetworkNum; otherwise match secondarySsid and return secondaryNetworkNum;
return 0 for neither. These are network numbers used by MQTT, not priority-role indices.
With WIFI_PRIORITY=1 the primary/secondary numbers are 1/2; with WIFI_PRIORITY=2 they
are 2/1. If configured names are equal, the primary check wins and returns its network
number (2 under priority 2), matching diagnostics_network's existing matcher. Never
log SSID or broker credentials.

Configure the MQTT owner using this actual selection, never connectToWiFi's requested
argument. Both paths use the helper so they cannot drift. Do not copy the late path's
implicit 'anything else is secondary' fallback. Unknown/empty or no longer connected
means defer configuration, not guess profile 2; do not call netConfigureMqttClient(0).
The late path remains eligible until a recognized profile has actually been configured,
including when WiFi succeeded at boot but profile selection was temporarily unavailable.
Represent successful configuration with one sketch flag shared by both paths (replace
or rename the current late-only flag as needed); WiFi success itself retains its existing
meaning. No repeated invalidation/configuration on every loop or ordinary GOT_IP.

One bounded MQTT_PROFILE_SELECT event per selection outcome change records
source=initial|late requested=0|1|2 actual=0|1|2 result=configured|deferred and mismatch=0|1.
Late path uses requested=0 (not applicable); mismatch is true only when a nonzero
requested profile differs from a recognized actual one. Coalesce repeated deferred
outcomes so an unknown SSID cannot fill the log. Preserve existing MQTT_CONFIG,
MQTT_WIFI_PROFILE and BEGIN connection/wifi_connection telemetry.

This fixes initial and late-boot selection, the demonstrated defect. Automatic switching
of an already configured broker profile after later roaming is not added here; existing
association mismatch telemetry exposes it if it occurs. Existing epoch checks cancel
in-flight work on link changes; this helper is not a new atomic association guarantee.
Do not change endpoint credentials, port-to-TLS policy, retry scans or bench endpoint selection.

## 6. Resource and responsiveness acceptance

Retain the single worker's 12288-byte PSRAM stack/internal TCB, no main-thread MQTT
access, lease only after DNS, mutual exclusion with Latest/Live and retrieval (including
USB entry), and 20480-byte internal-largest admission gate. No lowered gate or insecure
TLS fallback. Longer attempts hold existing TLS allocations for longer; the design does
not assume peak memory is unchanged just because no buffer was added.

Retained bench gates: UI/IMU service gaps <=100 ms during reconnect-only windows,
no >=1000 ms main-loop gap, worker stack margin >=2048 bytes, internal-largest samples
>=20480, no new reset, stuck fault, queue loss or logger error. Record DMA minima too.
F003's 23/25 ms values are a comparison baseline, not tighter mandatory gates. Avoid
Latest/Live during these timing windows; their unresolved stalls belong to P004.

Existing BEGIN/END (valid mask, separate DNS/TCP/TLS/MQTT durations, failed phase),
MEM, SERVICE, MQTT_LOST/CONNECTED, WIFI_GOT_IP and backoff_ms are sufficient. Preserve
original dispatch/completion stamps and measured-vs-unobserved labels. Successful
subscription writes still do not establish SUBACK. No fabricated TLS error freshness.

## 7. Implementation and host review scope

One increment changes retry adoption in net_module, worker limits/connect timeout scope,
and the two sketch profile-selection paths. Add focused checks to the existing MQTT
owner/service suites (a small profile selection harness is acceptable). All existing
suites must pass; update only assertions whose specified timing changes, not unrelated
ownership, cancellation, media/retrieval or logging expectations.

Required host evidence:

- Stable established loss earns one prompt eligibility at >=60000 ms from real READY
  adoption. Exercise 0, 59999, 60000 and >60000 ms; repeated short successes/losses
  each retain 15 s and never accumulate ONLINE time. New success resets the timestamp;
  repeated status/GOT_IP does not. Exercise a valid zero-time adoption and delayed main
  adoption to ensure the clock is neither worker completion nor loss-event time.
- No startup, cleanup-only, revoked READY or bench-disconnect credit. Shutdown asserts
  no dispatch with stop set, even if a loss adoption backdates the stamp. Failure or
  cancellation returns to 15 s. Repeated same-IP GOT_IP and flapping do not re-arm it;
  a new adopted success must independently survive >=60 s before earning prompt retry.
- Preserve off's first five-second wait, on's immediate restoration, precedence without
  double subtraction, blocked admission, unsigned wraparound and initial failure budget.
- Pin 15/5/10/10/2/45/50-second policy and unchanged ONLINE five-second bound. Exercise
  full-allowance admission at the deadline boundary; reject insufficient remaining time.
- Execute the connect-timeout scope with successful, failed and cancelled exchanges:
  CONNECT observes 10; subscriptions, READY and subsequent ONLINE reads observe 5.
  Exercise delayed/fragmented CONNACK beyond 5 but below 10 s and timeout at 10 using
  deterministic mocks; verify absolute guard prevents per-byte deadline extension.
- Keep real PubSub polling tick-delay and oversized-discard/session-retention checks.
  Keep stale epoch rejection, lease retention, DNS slots/ERR_MEM, 50-second fault and
  late-cleanup recovery checks. No new task/lifetime socket timeout mutation.
- Execute selection with requested network 2/joined ssid1 and requested network 1/joined
  ssid2, matching profiles, equal configured names, unknown/empty and disconnected
  snapshots. Under WIFI_PRIORITY=2, primarySsid=ssid2/primaryNetworkNum=2 and
  secondarySsid=ssid1/secondaryNetworkNum=1: assert each joined SSID selects its network
  number regardless of requested number. Equal names return primaryNetworkNum=2. Also
  retain the priority-1 cases; do not hard-code primary=1 in the harness.
  Verify both call sites configure once with actual, deferred selection can recover,
  and repeat deferrals do not produce per-loop records. Assert no secret values in events.

Write the implementation handoff with changed files and host counts for Claude review.
No firmware compilation by either assistant. Because companion.ino will change, delete
only the selected profile's generated sketch before JP's later rebuild. Do not delete it
as part of this design-only work.

## 8. Essential bench scope and later field evaluation

Two cases maximum planned, issued separately after code review and JP's build. Reuse
P001 evidence for everything unchanged; expand only for a failure or unresolved gate.

1. **Normal hotspot loss and recovery after a stable MQTT session.** Leave the real
   adopted connection ONLINE for at least 60 s (allow 65 s before triggering loss), with
   media/retrieval idle and G-meter visible. Capture status and the resulting log.
   Verify first BEGIN follows usable GOT_IP/cleanup without the old remaining 15 s wait,
   actual/configured profiles agree, eventual success, responsiveness and resource gates.
   Target BEGIN within 1 s of eligibility on this idle bench; if later, inspect timestamps
   and admission before concluding a defect. Serial off/on alone is not this test:
   on already bypassed backoff before P005, so it cannot prove A. For a discriminating
   timing result, IP must return before the former 15-second eligibility point; otherwise
   this proves recovery only, and the prompt-policy timing remains host-verified until a
   suitable field episode. Do not force repeated cycles just to manufacture that timing.
   No extra settle delay.
2. **Retained pending-attempt flap / failed retry case.** Use the existing serial test
   endpoint to make an attempt observable, interrupt hotspot while it is pending, and
   restore the link while still targeting test. Capture one failed/cancelled completion
   and the following normally spaced retry, then restore real broker with on. Verify no
   early retry from GOT_IP, cleanup before replacement, final real connection and the
   same service/resource gates. This intentionally verifies bench override preservation
   and failed-attempt spacing, not normal-loss prompt eligibility (case 1 covers that).

Do not require a contrived secondary-SSID race or a ten-second successful handshake
on hardware. Deterministic host checks cover those control paths; field evidence will
assess whether the longer allowance helps real slow links. Record this residual risk.
No fresh full iPhone/SD/Live matrix and no throughput experiment.

After JP accepts retained results, deploy at JP's discretion and compare future ride
loss-to-IP, IP-to-BEGIN, phase durations, failed-to-next-BEGIN and loss-to-CONNECTED.
Compare similar episodes rather than treating different cellular conditions as paired
measurements. Report failures at the new caps and lease delays as well as successes.
Do not promise fewer WiFi drops. P004 image responsiveness design is next, not part of
this implementation. Any follow-up cap/policy tuning needs a separate decision.

## 9. Review and approval checkpoint

Claude's focused revision-2 check covers B1's >=60 s adoption-to-adoption stability
condition and B2's priority-independent network-number mapping, including their host
cases. The three nonblocking review points are accepted: cancelled-attempt backoff,
shutdown no-dispatch rather than no-credit, and host proof plus later field timing when
bench IP recovery cannot discriminate the scheduling change. JP's direction is recorded; this revision and its two-case scope await review
and implementation approval. No further product choices are required unless the review
finds a trade-off that changes this contract.

## Claude review - September 25, 2026 (revision 1, 5227a7a)

**Verdict: two blockers (B1, B2), both small specification fixes.** The rest checks out
against the current source. I accept Codex's evidence corrections to my proposal: six
(not seven) of nine quick IP returns, about 44.2 s rather than 30 s for 20:26, and the
image-gap counts (7 of 11 <= 1.4 s, 2 near 1.8-2.0 s, 2 near 5.2 s).

**Verified:**
- **Prompt credit and bench precedence.** `observedConnected` becomes true only when main
  adopts a real READY (net_module `takeResult` path). `intentionalDisconnect()` clears it
  for bench off and restore, so bench disconnects earn nothing. `restoreBenchMqtt` clears
  `benchFirstPending`, and dispatch clears both flags, so the priority order in section 3
  is sound and never double-subtracts. The 20:26 case (link up) dispatches in the same
  loop turn: `takeLoss` in `netMainTick` runs before `netCheckMqtt`, and `request()` only
  needs the loss consumed.
- **Connect-only timeout.** `socketTimeout` is read only by the CONNACK wait and
  `readByte`. The scope guard restores 5 before subscriptions and ONLINE use, and the MQTT
  phase's absolute `operationDeadline` still bounds per-byte waits.
  `setHandshakeTimeout(10)` affects only `ssl_starttls_handshake`, which yields
  `vTaskDelay(2)`. The handshake bio is nonblocking, so the TCP-lifetime 5000 ms is
  untouched. The budget arithmetic holds: after the maximum 15 s DNS plus the 0.1 s lease
  wait, 29.9 s remain against 27 s of phase allowances. STUCK 50 s exceeds ATTEMPT 45 s.

**B1 - an established session that keeps dropping would reconnect with no backoff
(blocker).** Section 3 grants a prompt attempt after every adopted-success loss, with no
minimum ONLINE time. If the broker accepts and then drops the session repeatedly, the
cycle becomes connect, loss, immediate retry, success, loss, and so on: TLS handshakes
back to back, with the media lease held most of the time. Causes include a duplicate
client ID kicking an older session (for example a HOME and a CAR board, or a lingering
session), an ACL or keepalive rejection after subscribe, or a flaky endpoint. Today's
15 s interval prevents this.
**Fix:** earn the prompt credit only if the session stayed ONLINE for a minimum time
(suggested 60 s, measured from READY adoption to loss adoption on main). Shorter sessions
fall back to the normal 15 s. Add host cases: a session shorter than the minimum gets no
credit; repeated short sessions keep 15 s spacing.

**B2 - P006 must return the network number, not the priority role (blocker).**
Section 5 says "select 1 for primary, 2 for secondary". MQTT profiles are numbered by
network: `ssid1` is 1 and `ssid2` is 2, as `connectToWiFi(primaryNetworkNum)` and the
late path `(WiFi.SSID()==ssid1)?1:2` use. Primary and secondary are *roles* set by
`WIFI_PRIORITY` (companion.ino:2179-2193). With priority 2, primary is `ssid2`, number 2.
The existing matcher (diagnostics_network.cpp:48) already returns
`primaryNumber`/`secondaryNumber`, which is correct.
**Fix:** specify SSID -> network number via `primarySsid -> primaryNetworkNum` and
`secondarySsid -> secondaryNetworkNum` (the primary check wins on equal names), and add a
host case with the priority-2 mapping. Today `WIFI_PRIORITY` is 1, so the wording happens
to coincide, but the HOME build P006 protects is exactly where this matters.

**Nonblocking:**
- **Flap mid-attempt still waits 15 s.** An attempt cancelled by a mid-attempt link loss
  stamps the normal 15 s, so F002-style flapping still waits after IP returns. That is an
  acceptable trade-off against storms; note it for field reading.
- **Shutdown.** A loss adopted during shutdown may backdate `lastMqttAttempt`, but
  `request()` refuses while `stop` is set. Have the host check assert "no dispatch" rather
  than "no credit".
- **Bench case 1** will often not discriminate the timing, as section 8 already says.
  Accepting host proof plus a later field episode is reasonable.

Once B1 and B2 are integrated, I only need to check those two changes. No further review
round is needed before JP's implementation approval.


## Revision 2 integration - September 25, 2026, Codex

Accepted both blockers from f5a25b7 at JP's request. Section 3 now requires >=60 s
from real READY adoption to loss adoption; each short session retains 15 s backoff.
Section 5 returns primaryNetworkNum/secondaryNetworkNum, including WIFI_PRIORITY=2
and primary-first equality. Section 7 adds boundary, repeated-short-session and
priority-2 host cases; these are requirements for implementation, not tests run now.
Section 8 adds a 65-second ONLINE preparation to the existing first bench case, not
a new case. All nonblocking review points are noted in sections 3 and 9.

Claude's revision-1 review above remains verbatim. Revision 2 awaits his focused check
and JP implementation approval. No firmware, build, flash or hardware test performed.

## Claude focused check - September 25, 2026 (revision 2, 79259bc)

**Verdict: B1 and B2 are resolved. No remaining blockers; ready for JP's implementation
approval.**

- **B1 resolved.** A main-owned `onlineAdoptedAtMs` is stamped only in the real READY
  adoption branch, after `acknowledgeReady` and a connected confirmation, and never on
  worker completion, GOT_IP or unadopted READY. Prompt eligibility requires Idle bench
  plus `observedConnected` plus at least 60000 ms from adoption to loss adoption. Each
  session is measured on its own, with no accumulation, so repeated short sessions keep
  15 s spacing and a reconnect storm cannot form. Boundary cases (0, 59999, 60000,
  > 60000 ms), delayed adoption and "reset on new success only" are required host checks.
  Bench precedence is unchanged. The 65 s preparation for bench case 1 is consistent.
- **B2 resolved.** Matching is primary SSID first, returning `primaryNetworkNum`, then
  secondary returning `secondaryNetworkNum`, which is the same semantics as
  diagnostics_network.cpp:48. Priority-2 host cases (including equal names returning 2)
  are required, and the harness must not hard-code primary = 1.
- **Nonblocking points** are recorded as accepted in sections 3 and 9.

Editorial nit, no review needed: section 9's sentence "this revision and its two-case
scope await review" ends without its object (presumably "and implementation approval").
