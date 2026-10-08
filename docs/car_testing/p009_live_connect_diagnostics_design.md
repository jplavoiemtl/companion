# P009 - Live connection phase diagnostics

Revision 1 - October 8, 2026. Author: Codex.
Status: JP approved implementation October 8 after Claude clearance 718fff9 /
125fc18. Implementation complete, awaiting Claude code review; no build or flash.
Source baseline fd4f0b8. Inputs: F010 review a70fa42 and Pi-proxy addendum
8544ccb in [the field journal](field_journal.md). P008 is accepted by JP.

## Purpose and scope

The four October 8 failed Live starts never reached the Pi. A later session
reached it but stalled on the return path after two displayed frames. Diagnose
new connections to the Synology endpoint as DNS, TCP/setup or TLS; do not claim
to localize an established-response stall with connect timing.

Change behavior only inside Live ensureConnected, plus the minimum shared-client
type plumbing and diagnostic formatting needed below. No new worker/client,
retry, timeout change, still HTTPClient flow change, P004 or UI change. Keep
existing Live failure/return behavior, keep-alive fast path and HTTP response
timeout. Every new Live connection, including mid-feed reconnection, is covered.

## Verified core behavior: the shared-client hazard

The selected local profile still pins core 3.3.11. Its installed build cache is:
C:/Users/photo/AppData/Local/Arduino15/internal/esp32_esp32_3.3.11_b0d8b7bad2896d0b/
The packages directory contains 3.3.12; the profile target remains cached
3.3.11. The flag behavior is also unchanged in 3.3.12.

Checked 3.3.11 libraries/NetworkClientSecure/src/:
- NetworkClientSecure.h:31-34 makes sslclient and _stillinPlainStart protected.
  setPlainStart():89-91 only sets true; there is no public false setter.
- NetworkClientSecure.cpp:101-108 stop() does NOT reset the flag.
- connect(IP,port,host,CA,...):147-164 skips handshake when the flag is true;
  a failure calls stop(), leaving the flag true.
- startTLS():167-182 clears it only on SUCCESS. A failed handshake also calls
  stop() and returns before clearing it. Calling startTLS merely to clear a
  disconnected client is unsafe and is explicitly forbidden.
- write():234 uses send_net_data while plain-start remains enabled. A later
  still request could therefore skip TLS and send plaintext if the flag leaks.
- connect writes last_error on this attempt; startTLS does not. Reading
  lastError after a failed native startTLS does not yield a fresh handshake error.

### Required protection, not a second TLS client

Use a tiny MediaSecureClient subclass of WiFiClientSecure for the EXISTING
httpsClient object, with an explicit method clearing the protected plain-start
flag. No extra members, allocation, virtual overrides or copied TLS engine.
Add its header under src/image; change the object's declaration, the borrow
accessor return type, and vidClient's pointer type so Live can call the reset
method without casts. HTTPClient still receives the same object through the
normal base interface. Its connect/write/stop behavior is not overridden.
These mechanical type edits in image_fetcher.cpp/.h are necessary to protect
the shared object; they do not change the still-image request path.

A scoped guard in ensureConnected owns plain-start from just before it is set:
- No HTTP bytes may be written until startTLS returns success.
- TCP failure: capture its fresh error first; native connect has already closed
  the transport. Clear the flag before returning.
- TLS failure: native startTLS has already closed it. Clear the flag before
  returning, even though native startTLS failed to do so.
- Success: native startTLS clears the flag; guard verifies/normalizes false and
  preserves the encrypted connection for existing keep-alive use.
- Any early exit after TCP success but before calling startTLS must close that
  still-plaintext transport before clearing the flag. Track this explicitly in
  the scoped cleanup, so a future early return cannot hand a plaintext socket
  back as though it were encrypted. Do not redundantly stop again on failure
  paths where the native API already closed the socket.

Invariant at every return to the caller: plain-start is false; either the
connection is closed or its handshake succeeded. The connected fast path must
never accept an unexpectedly plain-start client as an encrypted reusable one:
close/reset that inconsistent state before a fresh attempt. Do not rely on
videoStreamStop or a future still call to repair the flag. No object replacement,
placement-new, downcast to a fictitious derived object or installed-core patch.

## Phase sequence and bounds

Retain current stop/CA/timeout setup and the one enclosing live_connect Span /
LiveTls probe. Time each call with monotonic milliseconds:
1. Network.hostByName(epHost, address), the same resolver used by the old
   hostname connect. On failure end with failed_phase=dns; do not set plain-start.
2. Arm the guard, setPlainStart(), then connect(address, epPort, epHost,
   remote_server_ca_cert, nullptr, nullptr). Keep setConnectionTimeout(5000).
3. Only after TCP/setup success, call native startTLS(); keep
   setHandshakeTimeout(5). Return success only after its success.

Keep epHost as the original hostname argument, never replace it with a printed
IP. start_ssl_client configures the CA and mbedtls_ssl_set_hostname with that
argument (ssl_client.cpp:309), preserving SNI and hostname verification. No
setInsecure, alternative address retry, certificate bypass or DNS cache change.

DNS has **no application-enforced deadline**, just as today. Network.hostByName
(NetworkManager.cpp:47 onward) calls blocking lwip_getaddrinfo and preserves
its IPv6 preference/fallback and literal-address behavior. We do NOT borrow the
MQTT worker's asynchronous resolver or impose its 15 s deadline. DNS is measured
separately; its underlying resolver limits are not a promised Live wall bound.

The IP connect performs TCP plus local TLS configuration/certificate parsing
before returning. Call the field tcp_ms for readability, but label a failure
**tcp_setup**, not proof of a remote TCP failure: local setup/allocation/CA
errors also arise here. tls_ms covers the subsequent native handshake call.

Required source comment beside ensureConnected:
"Blocking on main: DNS (no application deadline) + up to 5000 ms TCP wait +
5000 ms TLS handshake, plus local setup/scheduling overhead: about 10 s + DNS.
These are separate limits, not a hard 10 s total; this diagnostic does not
keep UI/IMU serviced during connection setup."

## Records and error freshness

Keep one NET_BEGIN/NET_END and one LIVE_CONNECT per actual connection attempt,
with the same operation/live ids, elapsed span, breadcrumb restoration and
LOOP_GAP attribution. Extend only the live_connect end format via a typed
Span end overload/helper; the existing generic Span::end and non-Live records
retain their current formatting and behavior. No duplicate END or new event type.

Append to both NET_END(kind=live_connect) and LIVE_CONNECT:
`dns_ms=... tcp_ms=... tls_ms=... phase_valid=... failed_phase=...`
Bits 1/2/4 mean DNS/TCP/TLS call was executed; unexecuted durations are zero,
not claims of an instantaneous successful phase. failed_phase is
none/dns/tcp_setup/tls. Preserve NET_END result/code and LIVE_CONNECT ok.

For these two Live records use the SAME captured error metadata:
`tls_queried=0|1 tls_code=<signed> tls_fresh=fresh|unavailable`
- DNS failure: no lastError query, code=0, unavailable.
- TCP/setup failure: query lastError immediately after connect returns, before
  reuse; its code was written by that connect. queried=1, fresh. The field's
  legacy name does not mean this is necessarily a TLS-handshake failure.
- TLS failure: native startTLS does not update last_error; queried=0, code=0,
  unavailable. This is **not** success/code-zero evidence. failed_phase=tls and
  ok=0 carry the failure. Fresh detailed TLS error is not available through this
  API. Do not report the preceding TCP result as a fresh TLS error.
- Success: no error query, code=0, unavailable (no error to report).

Do not fork the native startTLS wrapper or call internal handshake functions
solely to obtain a richer error code in this increment. That would add a core
dependency beyond the necessary flag reset. Preserve numeric errors where
available and be honest about unavailable native TLS detail. Change Live's
serial failure print to use the captured phase/error instead of querying stale
lastError again. Omit tls_text on the new Live format (numeric code plus phase);
other paths keep their existing text. No hostname, IP, URL, token or certificate
data in the new fields. Assert worst-case formatted fields remain <456 bytes.

## Host checks and next field reading

Add a focused Live-connect suite executing the real orchestration/guard logic
with a faithful sticky-flag mock, supplemented by source-contract checks:
- DNS failure skips TCP/TLS; TCP failure skips TLS; TLS failure and success;
  valid bits, elapsed durations, failed phase and matching event metadata.
- Failed TCP AND failed TLS followed by a normal still connect must perform
  TLS, never plaintext. Include stop() leaving the flag true in the mock.
- Inject an early exit between TCP and TLS; the guard closes and clears.
  Success/reused encrypted connection remains open; inconsistent plain state
  never passes the connected fast path. No HTTP write before TLS success.
- Exact original hostname/CA reaches the IP overload; 5000 ms/5 s unchanged;
  DNS called once using the same resolver, no timeout/retry change.
- TCP/setup error fresh; deliberately stale error after startTLS must NOT
  appear as fresh. Cleanup must not depend on DIAG_ENABLED; guard always active.
- One END per attempt, breadcrumb/span cleanup, maximum-width record sizing;
  existing still-image/network diagnostic and media admission checks unchanged.
Run all existing host suites. They cannot prove real TLS/CA behavior; Claude
reviews core assumptions and the shared-client regression before JP builds.

After approval and implementation, JP builds; ordinary rides supply evidence.
No fault-injection bench campaign is proposed. Record the build boundary.
Next failure reads as: dns -> resolver; tcp_setup -> socket/TCP/local secure
setup (use fresh error where available); tls -> handshake/verification after
successful TCP setup. Neither a generic -1 nor a near-limit duration proves
a precise remote cause. Success then response timeout still requires downstream
logs, not a connection-timeout change. Existing aggregate spans stay comparable.

Claude review required before JP decides on implementation, especially the
subclass/guard invariant and truthful unavailable handshake error reporting.

## Approval and implementation interpretation - October 8, 2026

JP approved the reviewed design. Implemented as described in the
[code-review handoff](p009_implementation_handoff.md). A1-A3 below are accepted:
the TLS timer starts at the handshake; the protected member exists in both
cached 3.3.11 and rollback 3.1.3; loss of fresh handshake detail is explicit.
Neither profile has been compiled by Codex. Compatibility is source-inspected,
not a build result, and depends on the protected member name remaining available.

A4 needs a narrower field interpretation. A near-5000 ms TLS failure is
consistent with exhausting the handshake allowance, but does not prove the
Synology sent no answer: partial progress or a return-path problem can also
exhaust it. A fast failure can be local setup/verification, peer rejection,
link loss or another transport failure; timing alone does not distinguish them.
Apply the same caution to tcp_setup, which includes local TLS configuration.
Use phase, duration, any fresh code and server evidence together. The original
review is retained below; this clarification governs subsequent field reading.

## Claude review - October 8, 2026 (revision 1, 306edde)

**Verdict: no blockers.** The core claims are correct, the subclass/guard is the
right minimal fix for the shared-client hazard, and the error reporting is honest.
JP can approve implementation. Four additions follow (A1-A4); none changes the
design's shape.

**Verified against the installed 3.3.11 core**
(`internal/esp32_esp32_3.3.11_*/libraries/NetworkClientSecure/src`):
- **Plain-start flag.** `setPlainStart()` only sets it (`.h:89-91`).
  `stop()` never clears it (`.cpp:101-108`). The IP `connect()` skips the
  handshake when the flag is set and calls `stop()` on failure (`.cpp:147-164`).
  `startTLS()` clears it only after success and calls `stop()` first on failure
  (`.cpp:167-182`). `write()` sends plaintext while it is set (`.cpp:234`).
  The hazard is real: without the guard, a failed Live TCP or TLS attempt would
  make the next still request's HTTP bytes go out unencrypted.
- **Identical connection path.** Today's `connect(host,port)` is literally
  `Network.hostByName()` followed by the same IP overload with the hostname
  (`.cpp:138-145`). The split therefore uses the same resolver, SNI and hostname
  verification (`ssl_client.cpp:309`).

**A1 - The TLS limit is unchanged by the split.** The handshake timer starts inside
`ssl_starttls_handshake()` (`ssl_client.cpp:330-336`), not in `connect()`. TLS
keeps exactly its 5 s, and time spent in TCP is not charged to it, the same as
in the combined call. Worth stating, since the design asserts unchanged limits.

**A2 - The rollback core is compatible.** 3.1.3 (`internal/esp32_esp32_3.1.3_*`)
has the same protected `_stillinPlainStart` (`.h:30-34`) and the same `stop()`.
The subclass therefore compiles on both the default 3.3.11 profile and the
rollback profile. The implementation should confirm both compile, or at least
note that the subclass depends on this protected member name.

**A3 - The split gives up the old handshake error code; acceptable.** In the
combined call, a handshake failure did write `last_error` (`.cpp:156`). Every
F009/F010 failure nevertheless logged `-1 "Generic error"`, unknown freshness:
that `-1` is both the socket-failure and the handshake-timeout return
(`ssl_client.cpp:130-160, 336`), so it never separated the phases. Phase timing
is worth more than that code. Record this trade explicitly.

**A4 - Add a duration rule to "next failure reads as".**
- `failed_phase=tls` with `tls_ms` about 5000 means a handshake timeout (no answer).
- `tls_ms` well under 5000 means the handshake was actively refused or failed
  verification.
- Likewise, `tcp_setup` at about 5000 ms is a connect timeout. A short
  `tcp_setup` is a refusal/reset or a local setup error, which its fresh code
  separates.
- This needs no extra logging and answers October 8's question directly: no
  answer from the Synology versus an active rejection.
