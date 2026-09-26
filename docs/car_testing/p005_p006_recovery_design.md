# P005/P006 - MQTT recovery timing and joined-network profile

Revision 1 - September 25, 2026. Author: Codex.
Status: design for Claude review, then JP implementation approval. No firmware changes,
builds or flashes. Reviewed against source at 092b32c on codex/car-improvements-p001.

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
These are counterfactual scheduling savings, not promised recovery times.

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

## 3. P005 A - one prompt attempt after established-session loss

At takeLoss adoption, snapshot observedConnected before changing it. Only a normal
real-session loss (BenchPhase::Idle and observedConnected true) backdates
lastMqttAttempt by MQTT_RECONNECT_INTERVAL. Continue emitting MQTT_LOST and clear
observedConnected exactly as today. No new credit counter is needed: this flag becomes
true only after main accepts a real READY, and the loss consumes it once.

Scheduling priority is explicit, mutually exclusive:

1. restorePending: retain existing immediate bench restoration.
2. benchFirstPending: retain existing five-second first test-attempt delay.
3. established normal loss: immediate eligibility.
4. Other cleanup/loss acknowledgements: retain normal 15-second stamp.

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
loss may earn another prompt attempt. Startup and the initial five-failure budget stay
unchanged. Shutdown and deliberate bench disconnects do not earn prompt retries.

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
Take one WiFi.SSID() snapshot while connected; compare with configured ssid1 and ssid2.
Select 1 for primary, 2 for secondary, 0 for neither. Primary wins if names are equal,
matching diagnostics_network's existing matcher. Never log SSID or broker credentials.

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

- Established loss earns one prompt eligibility; no startup, cleanup-only, revoked READY,
  intentional shutdown or bench disconnect credit. Failure/cancellation returns to 15 s.
  Repeated same-IP GOT_IP and flapping do not re-arm it; a new adopted success can.
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
- Execute selection with requested 2/actual primary, requested 1/actual secondary,
  matching profiles, equal configured names, unknown/empty and disconnected snapshots.
  Verify both call sites configure once with actual, deferred selection can recover,
  and repeat deferrals do not produce per-loop records. Assert no secret values in events.

Write the implementation handoff with changed files and host counts for Claude review.
No firmware compilation by either assistant. Because companion.ino will change, delete
only the selected profile's generated sketch before JP's later rebuild. Do not delete it
as part of this design-only work.

## 8. Essential bench scope and later field evaluation

Two cases maximum planned, issued separately after code review and JP's build. Reuse
P001 evidence for everything unchanged; expand only for a failure or unresolved gate.

1. **Normal hotspot loss and recovery after an established MQTT session.** Reconnect
   with media/retrieval idle and G-meter visible. Capture status and the resulting log.
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

Claude should focus on prompt-credit consumption/bench precedence, timeout restoration
on every return path, cooperative budget wording, and recognized-SSID selection with
late retry. JP's direction is recorded; this revision and its two-case scope await review
and implementation approval. No further product choices are required unless the review
finds a trade-off that changes this contract.
