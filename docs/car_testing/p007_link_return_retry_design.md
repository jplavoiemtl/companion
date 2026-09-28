# P007 - Prompt MQTT retry after WiFi link loss

Revision 1 - September 27, 2026. Author: Codex.
Status: direction approved by JP; design for Claude review and subsequent JP
implementation approval. No implementation, build or flash authorized by this document.
Source reviewed: main at cb06706. Inputs: [F007 and Claude review](field_journal.md)
and [P005/P006](p005_p006_recovery_design.md).

## 1. Purpose and evidence

Remove avoidable retry waiting after a WiFi-caused MQTT loss or cancelled attempt,
including short ONLINE sessions. This improves recovery after link return; it does
not prevent beacon losses or establish their radio/hotspot cause.

F007 identifies four outages with about 11.5 seconds of idle time after IP restoration:
12:53:37, 12:54:33, 13:06:06 and 14:27:57. The third includes an attempt cancelled by
another link loss. Moving the following attempt earlier could remove that waiting;
actual outcomes and total outage savings cannot be predicted on a different network
schedule. A prompt attempt can itself be interrupted by another drop.

Keep the 60-second READY-adoption-to-loss-adoption stability gate for a loss without
a link-down observation. Keep 15 seconds after genuine link-up failures. Preserve
P005 timeouts, the initial five-counted-failure budget, bench precedence, worker
ownership, cleanup, cancellation, shutdown no-dispatch and all admission protections.

## 2. Source checks and scope

The change belongs in src/net/net_module.cpp and tools/tests/mqtt_recovery.test.cjs
(with existing owner/service harness adaptations only where necessary).
No net_worker implementation/header, MQTT protocol, sketch or WiFi scan changes.

Current boundaries:
- netLinkEvent forwards fixed metadata to mqttowner::linkEvent. CONNECTED,
  DISCONNECTED, LOST_IP and STOP currently forward false; GOT_IP forwards true.
  Therefore counting every false call as a physical loss would be wrong.
- netMainTick adopts Result first, then takeLoss. A revoked READY can produce a
  cancelled result plus a cleanup-only loss. Each currently resets lastMqttAttempt.
- netCheckMqtt calls request; only an accepted request creates an attempt. The owner
  rejects dispatch while link-down, stopping, busy, leased, faulted or with unconsumed
  result/loss mailboxes. Prompt eligibility must not bypass these checks.
- Successful real READY adoption sets observedConnected and onlineAdoptedAtMs.
  Intentional disconnect clears them. Repeated GOT_IP does not invalidate a healthy
  owner connection. Preserve all these properties.

## 3. Definition of link-caused and fixed metadata

Use a small net-module observer protected by its own portMUX, shared only between
WiFi callback and main. Store a usable-link boolean (initially false) and uint32_t
linkDownSerial (initially zero). A transition from usable=true to false increments
the serial once. Repeated false events while down do not increment it. A true event
marks usable=true, without incrementing anything. A DHCP GOT_IP while already true
cannot earn eligibility. CONNECTED while already down is not a second loss.

Here 'link' means the owner's usable-IP lifecycle, including LOST_IP/STOP, not solely
radio association. The existing false forwarding remains unchanged. No event reason
heuristic, RSSI threshold, socket inspection, scan or callback logging is added.

Inside netLinkEvent, hold the observer lock while updating this metadata AND forwarding
the existing mqttowner::linkEvent call. This call only takes the owner's metadata lock;
it performs no allocation or network operation. Main snapshots the observer under its
lock, then releases it before other owner calls. Lock order is observer -> owner only;
never acquire observer while holding the owner lock. This prevents main seeing a new
serial before the corresponding owner invalidation has been published. Do not hold
this lock across request(), logging, LVGL, or any potentially blocking work.

Main owns an accepted-attempt baseline serial/id, an adopted-session baseline serial,
and one linkRetryPending boolean. Each baseline has an explicit validity flag; zero
is a valid serial. No queue of link cycles or accumulated retry credits.

Define link-caused for scheduling as:
- Adopted session loss: linkDownSerial differs from its session baseline.
- Cancelled attempt/revoked READY: a matching accepted attempt exists, the adopted
  result is cancelled (including existing stale-epoch reclassification), and the
  serial differs from that attempt's baseline.

This is an observed link disruption overlapping the lifecycle, not proof the radio
failure preceded a broker failure internally. A broker loss followed by WiFi loss
before main adoption is treated as link-affected; without worker changes their exact
causal order is unavailable. Mere WiFi.status()==CONNECTED at adoption is insufficient:
a full down/up cycle may already have completed. An epoch change alone is insufficient:
bench/configuration/shutdown invalidation must not invent a WiFi cause.

## 4. Baseline lifecycle and retry decision

### Accepted dispatch

Snapshot the observer immediately BEFORE request(). If request refuses, leave the
current baselines and pending eligibility untouched. On acceptance, store that snapshot
as the new attempt baseline with the accepted id, clear linkRetryPending and clear the
old session baseline. Capture before request so a drop racing with dispatch is not
silently absorbed into the new baseline. A down/up cycle in this narrow call interval
is conservatively included in the attempt lifecycle; it can qualify a later cancellation,
never a genuine non-cancelled failure. Do not take a post-request baseline that could
hide a newly invalidated attempt. Existing attempt IDs and worker epochs remain intact.

### READY adoption

Snapshot the observer immediately before acknowledgeReady. Only after acknowledgement
and the existing connected confirmation succeeds, install that snapshot as the session
baseline and set observedConnected/onlineAdoptedAtMs as today. Clear pending eligibility
and retire the attempt baseline. A drop after the snapshot remains visible at loss.
If acknowledgement is revoked, retain the attempt baseline through cleanup instead;
do not bless an unadopted READY as an ONLINE session.

### Result adoption

Keep the existing stale-epoch normalization and failure counting. A matching cancelled
real attempt with a changed serial sets linkRetryPending only in normal Idle bench
mode. No changed serial means the existing 15-second cancellation backoff.
A genuine failed result (DNS/TCP/TLS/MQTT/lease or other non-cancelled result) clears
pending eligibility, invalidates the attempt baseline and retains 15 seconds from main
completion adoption, even if a
link event was observed elsewhere. Neither a previously granted prompt opportunity nor
an old cycle carries through a genuine failure. Do not change result.counted or the
initial give-up logic. In particular, P007 never resurrects giveUp.

A result initially marked ok can be revoked between the stale check and acknowledgement.
If connected confirmation then fails, do not create a session or grant stable credit.
Keep the attempt baseline for the ensuing cleanup-only takeLoss; it may earn link
eligibility there if its serial changed. No new worker acknowledgement is required.

### Loss / cleanup adoption

For an observed session, set pending eligibility if its valid baseline shows a link
disruption, regardless of ONLINE duration. Otherwise use the unchanged >=60000 ms
stability test. Clear the adopted-session validity with observedConnected afterward.
For cleanup without an observed session, do not apply the stability gate. Only a
retained matching attempt baseline (revoked READY), or an already pending link decision,
can preserve/grant link eligibility. Retire the attempt baseline at this cleanup.

A cancelled ordinary attempt may have only a Result. Retain its id/baseline until
next accepted dispatch, intentional reset or successful READY adoption. Any later
cleanup may only reaffirm the same single pending flag, never create a second credit.
A genuine failure retires the baseline immediately, as specified above. This keeps
revoked READY cleanup eligible without leaving genuine failures with reusable history.

Use one exclusive precedence when writing lastMqttAttempt at terminal adoption:
1. restorePending: immediate restoration, as today;
2. benchFirstPending: first test delay of 5000 ms, as today;
3. normal Idle mode and (linkRetryPending or valid stable-session credit): eligible now;
4. otherwise: 15000 ms.

Backdate the existing stamp once, never subtract two credits. A subsequent cleanup
acknowledgement must preserve linkRetryPending rather than overwrite it with 15 seconds.
Do not periodically rewrite the stamp from GOT_IP or from a main-loop counter comparison.
Eligibility survives refused dispatch/admission but is consumed by accepted dispatch.
A new true/false/true burst while waiting does not stack credits. A fresh disruption
during the next accepted attempt may qualify its cancellation as a new opportunity.

Intentional bench disconnect, profile reconfiguration and shutdown clear pending
eligibility and invalidate both baselines. Bench Outage/Restoring cannot earn P007
credit, including when a real WiFi drop occurs. Preserve restoration then first-test
precedence and existing bypassRateLimit callers. Shutdown still relies on the owner's
stop check for no-dispatch, including callbacks or late completions after shutdown.

## 5. Edge cases and attempt-rate bound

| Situation | Required result |
|---|---|
| Short session, down/up before loss adoption | Eligible immediately after cleanup; not 15 s |
| Short session, broker loss with no down event | 15 s; repeated short sessions never accumulate ONLINE time |
| Stable session, link stays up | Existing >=60 s prompt rule |
| Link drops during DNS/TCP/TLS/MQTT attempt | Existing cancellation/cleanup first, then prompt if link has returned |
| Cancellation adopted while still offline | Eligible stamp retained; request refuses until usable link |
| Several down events before one GOT_IP | One observed down transition; no extra credits |
| Several full cycles before adoption | Coalesce to one next eligible attempt |
| New attempt fails with link up | 15 s from result adoption; old credit consumed |
| Stale result without new link loss | No P007 credit |
| Revoked READY result followed by cleanup | One eligibility; cleanup cannot erase it or grant an additional attempt |
| New link cycle during idle backoff after a genuine failure | No standalone credit; preserve failed-attempt spacing |
| Fault/lease/media/retrieval/shutdown prevents progress | No bypass, forced cleanup, extra TLS client or unsafe resource release |

Serial comparison uses inequality, so UINT32_MAX -> 0 still detects a loss. Exactly
2^32 down transitions within one lifecycle would alias; that is outside realistic
operation, not a timer comparison. Do not order unsigned serials with greater-than.

Each P007 prompt opportunity requires a new observed usable-up -> down transition
relative to the relevant lifecycle, followed by usable link return before dispatch.
All cycles already observed before accepted dispatch are absorbed into that attempt's
baseline. Multiple observations/cleanup messages from the same lifecycle coalesce into
one flag. Thus P007 adds at most one prompt opportunity per observed link cycle, not
one per callback or per broker kick. It does NOT impose a numerical minimum interval
on a rapidly flapping physical link; F007's ~3.5 s recovery is evidence, not a guaranteed
lower bound. Only one attempt/cleanup can exist at a time. A failed link-up attempt
still waits 15 s, and link-up short-session kicks remain rate-limited as before.

## 6. Observability and resources

No new worker telemetry, protocol or periodic event. Add one small main-side
MQTT_RETRY_DECISION record at each terminal scheduling decision with:
source=loss|result|cleanup policy=restore|bench_first|wifi_return|stable|backoff
link_changed=0|1 pending=0|1 wait_ms=0|5000|15000.
This exposes policy, not inferred physical cause. Two records for cancelled READY and
cleanup are acceptable and bounded; none are emitted on every callback or loop turn.
Keep existing MQTT_CONNECT_END, SERVICE, MEM, LOST, GOT_IP and backoff_ms meanings.
The new record's wait is scheduled spacing from adoption, not a guaranteed BEGIN time.

No dynamic allocation, task, socket or stack change. Keep single-worker TLS ownership,
lease after DNS, 20480-byte internal-largest gate, media/retrieval refusal (including
USB), stop/cancellation retention and all P001/P005 deadlines unchanged. Prompt retry
can increase lease frequency during bursts; ordinary rides must assess this alongside
outage duration. It cannot promise shorter network calls or fewer WiFi drops.

## 7. Host checks and review handoff

Execute the actual changed main-side bodies through the existing source-adaptation
harness. Update the old blanket 'all mid-attempt cancellations wait 15 s' expectations
only for the new link-caused normal-mode cases; retain unrelated assertions.
Required checks:

1. Short real ONLINE session, down/up, loss adopted after return: wait zero. Repeat
   with link still down at adoption and confirm no dispatch until return.
2. Accepted attempt cancelled by link loss: prompt after cleanup; both Result-only and
   revoked READY plus cleanup orderings preserve one opportunity. Inject a drop between
   stale check and READY acknowledgement. No unadopted session gets stability credit.
3. Short ONLINE loss with unchanged serial: 15 s, including repeated broker kicks.
   Preserve 0/59999/60000/>60000 ms stability boundaries and adoption-based time origin.
4. Duplicate false events, CONNECTED during down, repeated GOT_IP, several full cycles,
   and delayed main adoption: no queued credits or per-event dispatch. Exercise drop
   around request and around READY snapshot/acknowledgement; serial wrap is detected.
5. Accepted prompt request consumes credit; subsequent genuine link-up failure and
   unrelated cancellation wait 15 s. Refused request retains eligibility. Old cycles
   before a new success cannot make its later link-up short loss prompt.
6. Bench restore > first test > normal policy, no double backdating, and no P007 credit
   during bench phases. Intentional/configuration invalidation clears baselines.
7. Shutdown no-dispatch; initial counted-failure budget unchanged, cancellations remain
   uncounted, giveUp is not revived. Preserve busy/link/lease/mailbox/fault request guards,
   media/retrieval arbitration, unsigned millis retry arithmetic and worker epoch checks.
8. Retry-decision records match scheduling and remain terminal-event-only. No secret
   values, network operations or logging under the observer lock; inspect lock order.

Run all existing host suites, including mqtt_owner, mqtt_service, mqtt_recovery,
media_admission and retrieval suites. No firmware build by Codex. Handoff to Claude
with changed files, test counts and the counter/READY/cleanup race coverage before JP
builds. This document does not authorize implementation yet.

## 8. Ordinary-ride validation and approval

No bench campaign. After Claude design review, JP implementation approval, code review
and JP's eventual build/flash, validate during ordinary rides. No induced disconnections
or interaction while driving. Compare link return to BEGIN, cancellation/cleanup to
BEGIN, failed-attempt spacing, loss-to-CONNECTED, policy records, service gaps and memory.
Use the existing gates: reconnect-only UI/IMU gaps <=100 ms, stack margin >=2048 bytes,
internal-largest >=20480 bytes, no new stuck fault/reset/logger loss. Separate media
pauses, power-off episodes and delayed admission from scheduling defects.

If no short-session link loss or cancelled attempt occurs, say the new path was not
exercised; a quiet ride does not prove P007 fixed WiFi. Further timeout/radio changes,
P004 media worker, NAT/socket survival experiments and shorter genuine-failure backoff
remain outside scope.

Claude review requested: counter publication/lock order, baseline race boundaries,
revoked READY double-adoption, the one-opportunity-per-cycle claim, and unchanged
bench/failure/shutdown protections. JP approved the direction only; implementation
awaits the reviewed design and explicit authorization.

## Claude review - September 27, 2026 (revision 1, a609655)

**Verdict: no blockers.** The design is correct against the current source. One
simplification is recommended (S1). JP can approve implementation with or without it.

**Verified:**
- **Link-event tracking.** net_module forwards CONNECTED, DISCONNECTED, LOST_IP and STOP as
  `false` and GOT_IP as `true` (net_module.cpp:176-183). Counting only usable true->false
  transitions gives one increment per F007 drop sequence (DISCONNECTED, then repeated
  no_ap_found/sta_leaving, then CONNECTED, then GOT_IP). A repeated GOT_IP while up is
  ignored, matching the owner's N2 rule (net_worker.cpp:385).
- **Race boundaries.**
  - A drop between the pre-request snapshot and `request()` is refused, because the owner
    sees `link=false` under its own lock.
  - A full down-then-up cycle in that gap only affects a later *cancellation*, never a
    genuine failure, as stated.
  - A drop between the stale check and `acknowledgeReady` makes the acknowledgement
    refuse. The revoked path then relies on the retained attempt baseline.
  - A cycle between the worker's READY and main adoption is caught by the existing stale
    reclassification. The worker also re-labels a mid-attempt failure as cancelled when
    its epoch changed (net_worker.cpp:298/302). So "genuine failure despite link loss"
    can only arise when the drop comes after main adoption, which is correct.
- **Revoked READY.** Until cleanup the owner keeps `lease` and `lossReady`, so `request()`
  cannot dispatch between the cancelled result and the cleanup loss. Keeping one pending
  flag across both adoptions, with cleanup forbidden to overwrite it with 15 s, gives
  exactly one opportunity.
- **One opportunity per cycle.** There is a single boolean, consumed by accepted dispatch
  and cleared by genuine failure, bench, reconfiguration and shutdown. No credits
  accumulate. Any serial change during an ONLINE session necessarily ended that session,
  because every `false` invalidates the owner epoch. So "serial changed" really does mean
  link-ended.
- **Preserved behaviour.** Bench precedence stays restore, then first test, then P007 or
  stable, then 15 s, with a single backdate. Bench phases never earn credit. Genuine
  failures keep 15 s. `giveUp` and counted-failure logic are untouched. Shutdown keeps
  relying on the owner's stop check. Request guards and admission are unchanged.

**S1 - put the counter in the owner instead of a second lock (recommended
simplification).** The owner's `linkEvent` already detects the usable transition under
its own mux. Adding `if(!up && status.link) ++status.linkDowns;` there, and exposing it
through `view()`, makes the serial atomic with the epoch bump:
- it removes the new portMUX, the nested critical section and the lock-order rule;
- it removes the "publish serial before invalidation" argument;
- it costs two worker lines, against the design's "no net_worker changes" scope.

The design's nested approach is also safe: different muxes, and the WiFi event task and
main both run on core 1 (`EventsCore=1`, `LoopCore=1`). But it is more to get right and
to test. JP or Codex's choice.

**Nonblocking:**
- **Lease frequency during bursts.** In a burst, attempts may run almost back to back
  (F007: losses 11-56 s apart, attempts 1-12 s), so Latest, Live and download entry are
  refused more often. The existing `IMAGE_REFUSED`/`LIVE_REFUSED reason=mqtt_reconnecting`
  events already measure this; include their count in ride analysis, as section 6
  implies.
- **`MQTT_RETRY_DECISION` volume.** At most two per terminal adoption, and none per event
  or loop turn. Fine as specified.
