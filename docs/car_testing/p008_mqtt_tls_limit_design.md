# P008 - MQTT TLS handshake allowance

Revision 1 - October 3, 2026. Author: Codex.
Status: approved by JP on October 3, 2026 after Claude clearance b3dc48a.
Implemented for Claude code review; no build or flash. Source baseline: 1489ec7 on main.

## Evidence and recommendation

F009 and Claude's review in [the field journal](field_journal.md) record four genuine
post-rejoin TLS failures at 10004 ms and successful handshakes up to 9708 ms.
Boots 87 and 93 subsequently succeed after the 15 s failure backoff. The current
10 s cap may terminate an almost-ready path. The logs cannot tell whether another
five seconds would have succeeded. Recommend a modest increase to **15 seconds**,
not 20 or an adaptive policy; preserve retry spacing and isolate this one change.
This does not prevent beacon losses or address P004's synchronous media setup.

## Values and source contract

Change only the three constants in src/net/net_worker.h and affected host checks:

| Budget | Current | Proposed |
| --- | ---: | ---: |
| TLS_MS | 10000 ms | 15000 ms |
| ATTEMPT_MS | 45000 ms | 50000 ms |
| STUCK_MS | 50000 ms | 55000 ms |

Keep DNS_MS=15000, SOCKET_MS=5000 (TCP and ongoing operations), MQTT_MS=10000
(CONNECT/CONNACK), SUBSCRIBE_MS=2000 and the 100 ms lease wait unchanged.
The phase total is 15 + 5 + 15 + 10 + 2 = 47 s; adding the 0.1 s lease leaves
2.9 s within the 50 s attempt budget for scheduling/other work. This preserves
the existing margin, rather than squeezing later phases to fund TLS.

net_worker.cpp already calls secure.setHandshakeTimeout(TLS_MS/1000) and
admitPhase(cmd, Phase::Tls, TLS_MS, result). Both must therefore use 15 s.
The attempt clock remains the original request/dispatch timestamp; DNS and lease
wait do not reset it. admitPhase requires enough remaining budget for the full
next phase. MQTT CONNECT still restores its ongoing socket timeout to 5 s on
every exit. No retry, protocol, subscription, DNS-slot or client ownership changes.

These are allowance and fault-detection budgets, not guaranteed hard wall-clock
completion bounds. startTLS is a synchronous native call on the worker; it does
not consult the application's epoch/operation guard while inside that call. A
link loss or shutdown invalidates the attempt immediately but cleanup can wait
until the call returns. Increasing TLS can extend that wait by about 5 s. Keep
post-call epoch validation, owner-only close, and lease retention until safe
cleanup/READY adoption. Do not free a client or kill the worker on timeout.
At 55 s the existing stuck fault invalidates the attempt and retains resources
and exclusion; this is fault containment, not forced socket cancellation or a
guaranteed recovery deadline. Preserve the existing fault/reboot recovery path.

## Single TLS slot, media and memory

DNS still runs before requesting the lease. Existing media/retrieval activity
prevents lease admission; there is no second concurrent TLS handshake. While
the lease is held, Latest/Live and download-mode entry (panel or USB command)
remain refused as mqtt_reconnecting. Here USB command means entry via log mode on,
not a new restriction on the raw USB log-transfer protocol. Preserve existing
arbitration in both directions. Nothing automatically queues a refused media request.

The lease spans TCP, TLS, MQTT exchange and subscriptions through READY adoption
or failure cleanup. Nominal phase allowances under it rise from 27 to 32 s,
plus scheduling/cleanup/adoption overhead. A failed TLS-only leg can hold it
about five seconds longer; the full 50 s attempt budget is not a promised
32 s media-unavailability bound. DNS itself does not own the slot.

UI navigation, G-meter and IMU remain on main and should stay responsive. The
worker absorbs waiting, but the user pays longer media exclusion on dead paths.
No new task, stack, TLS client or buffer. Keep the 20480-byte internal-largest
admission gate and existing memory sampling; holding memory longer is the cost,
not an authorized relaxation of the gate. F009 attempt-largest >=45044 and
stack >=7100 bytes are reference measurements, not predictions.

## Expected benefit and unchanged policy

A successful TLS phase in (10,15] s would directly demonstrate an attempt the
old cap would have cut off, potentially avoiding a 15 s backoff plus a new
connection. A dead path instead fails roughly five seconds later and starts
its unchanged 15 s backoff later. No promised outage reduction or claim that
the 93 s F009 episode would have recovered earlier: it still had a subsequent
TCP failure. WiFi rejoin time, P007 prompt link-return retries, the 60 s gate
for associated-WiFi losses, bench precedence, initial failure budget, shutdown
no-dispatch, memory/lease refusal and epoch cancellation stay unchanged.

## Checks and field validation after approval

One small implementation increment, followed by Claude code review before JP
builds. Update mqtt_owner.test.cjs and mqtt_recovery.test.cjs budget fixtures
without weakening existing assertions; keep every existing suite passing. Check:
- Constants and both TLS consumers agree on 15 s; other phase limits unchanged.
- Full phase allowance admitted exactly at the 50 s boundary, refused one ms
  later; preserve dispatch origin and sufficient MQTT/subscription allowance
  after near-limit DNS/TCP/TLS work.
- Stuck fault at 55 s, not before, counted once; held lease remains held.
- Existing cancellation, stale READY, retry/backoff, media and retrieval guards
  still pass. Host checks cannot prove native TLS timing or radio behavior.

Use ordinary rides, no new bench campaign. Record the flashed build boundary
and retain both archive/current evidence as needed. From existing
MQTT_CONNECT_END/MEM, SERVICE, retry and WiFi records compare:
- TLS >10 s successes (direct benefit); near-15 s failures and subsequent
  recovery (cost); preserve failed-phase and cancellation distinctions.
- Loss-to-IP, IP-to-BEGIN, DNS/TCP/TLS/MQTT and total attempt/outage times
  separately, including attempt_budget, worker_stuck or unusually late cleanup.
- Recovery UI/IMU gaps (target <100 ms, no recurring visible freezes), worker
  stack >=2048, internal-largest >=20480, no allocation failure/reset/log drops.
- Any observed delay/refusal using Latest/Live or entering retrieval while
  reconnecting. No interaction while driving is requested.

A quiet ride does not validate the longer cap. If no handshake exceeds 10 s,
report no demonstrated benefit yet, with the normal safety evidence. If >10 s
successes appear without regressions, propose acceptance to JP. Repeated extra
waiting without benefit or practical media/cancellation harm warrants reverting
all three constants to 10/45/50 s after review, not shortening backoff by stealth.

## Original decision request (approved October 3; see approval below)

Claude: review budget arithmetic, native-call cancellation limits and the longer
lease tradeoff. JP: approve or reject 15/50/55 s and ordinary-ride validation after
review. P004 and optional server keep-alive investigation remain separate and
deferred; this document does not authorize either.

## Claude review - October 3, 2026 (revision 1, d664ecf)

**Verdict: no blockers.** The design matches the current source and the F009 evidence.
JP can approve 15/50/55 s. Three small additions are recommended (A1-A3); none
changes the values.

**Verified against source:**
- **Budget arithmetic.** The DNS deadline is measured from `cmd.started`
  (net_worker.cpp:111), and `admitPhase` refuses any phase that cannot fit before
  `cmd.started+ATTEMPT_MS` (net_worker.cpp:192). Worst case is DNS 15 + lease 0.1 +
  TCP 5 + TLS 15 + MQTT 10 + subscribe 2 = 47.1 s, inside 50 s. Each phase is
  admitted well before its latest start: TLS needs to start by 35 s and starts by
  20.1 s at worst. The 2.9 s margin and the 5 s attempt-to-stuck gap are both the
  same as today (42.1/45/50). Phase overruns observed in F009 are a few ms
  (TCP 5002-5022, TLS 10004, MQTT 10002), so the margin is ample.
- **Consumers.** `TLS_MS` feeds exactly `secure.setHandshakeTimeout(TLS_MS/1000)`
  (line 229) and `admitPhase(cmd,Phase::Tls,TLS_MS,...)` (line 275). The stuck check
  (line 423) uses `STUCK_MS` only. The CONNECT socket-timeout scope (lines 25-31) is
  unaffected. 15 is a whole number of seconds, so the integer division is exact.
- **Lease span.** The lease is taken after DNS (line 260) and released at cleanup
  (line 325) or READY adoption (line 449). The nominal span going from 27 s to 32 s
  is correct.

**A1 - The cleanup-wait cost is narrower than stated.** The design says a link loss
"can extend that wait by about 5 s." In F009 all four cancellations ended 1-3 ms after
the beacon-loss record. Two were mid-handshake, at boot 90 TLS 7773 ms and boot 103
TLS 3640 ms. So in the field, link loss aborts the native call almost immediately.
The extra 5 s applies only to a dead path *without* a link event (the I005 pattern)
and to a genuine TLS timeout. Suggest saying so, so the risk is not overstated.
Keep the post-call epoch validation exactly as designed.

**A2 - Setup is a third consumer.** `initMQTT()` (companion.ino:2283-2289) makes up to
three attempts and waits on `netMqttBusy()` for each, running `runBackgroundTick()`
throughout. Its worst case grows by 3 x 5 s. The UI keeps running and every F009
startup TLS was <=709 ms, so the practical effect is nil, but the design should name
it. Shutdown and sleep do not wait on the worker (`netShutdown()` only invalidates),
so they are unaffected.

**A3 - Test fixture specifics.**
- `mqtt_owner.test.cjs`: lines 14 and 18 (fixtures and header strings), plus the
  boundary at line 30 and the stuck test at line 40, move to 50000/55000.
- `mqtt_recovery.test.cjs`: lines 206 and 211 move to the new values. Add **15000** to
  the allowance list at line 214, so the new TLS allowance itself is boundary-tested,
  not just the old 5/10/2 s ones.
- `network_diagnostics.test.cjs:48` checks the symbolic call and needs no change.

**Validation note.** F009 had four TLS failures at the cap in about 9.8 h, so the
direct benefit signal (a TLS success in (10,15] s) is rare. Suggest an explicit
horizon: evaluate after roughly the same coverage as F009, about 10 h of rides, and
report "no demonstrated benefit yet" if none occurs, as the design already allows.
No new telemetry is needed; `tls_ms` already shows it. A supporting observation:
successful F009 handshakes are dense just below the cap (7437, 7570, 7750, 8469,
9602, 9708 ms). That suggests the distribution continues past 10 s rather than
stopping there. It is consistent with a handshake waiting on TCP retransmission
backoff after the path reopens (hypothesis).

## Approval and review integration - October 3, 2026, Codex

JP approved P008 and authorized implementation after Claude's no-blocker review
b3dc48a. The three constants and host fixtures are implemented; code review is
required before JP builds.

A1: agree that observed link-loss cancellation was fast (1-3 ms in F009, including
two TLS calls). The extra waiting chiefly concerns a dead path without a link
event or a genuine TLS timeout. This observation is not a universal guarantee
that native calls abort immediately; retain epoch validation and safe cleanup.
A2: setup can make three attempts, so its theoretical worst case rises by 15 s
while runBackgroundTick keeps UI/IMU serviced. Shutdown/sleep do not wait on the
worker; the cleanup discussion above is not an added shutdown wait.
A3: updated both fixture sets, explicitly boundary-tested the 15000 ms TLS
allowance, and checked the instant before the 55000 ms fault boundary.
Evaluate ordinary-ride evidence after roughly 10 hours of comparable coverage;
no >10 s success means no demonstrated benefit yet, not automatic success.
