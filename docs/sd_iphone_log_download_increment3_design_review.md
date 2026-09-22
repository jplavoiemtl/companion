# Increment 3 design - Claude review

Reviewer: Claude. Date: September 22, 2026. Subject:
[increment 3 design](sd_iphone_log_download_increment3_design.md) at `b1efdb7`, against
[spec](sd_iphone_log_download_spec.md) revision 3 and the installed toolchain.

**Verdict: the architecture is right and it closes all three of the spec's provisional
problems. Not yet ready for implementation approval: one spec contradiction and one
workflow risk need JP's decision, and four items need specifying rather than discovering
during implementation.**

## The central decision is correct, and verified

Dropping the cross-task `shutdown(fd)` scheme in favour of cancellable transport overrides
is the right call, and every API it depends on exists in the installed bundle. Checked in
`Arduino15/packages/esp32/tools/esp32s3-libs/3.3.11`:

- `esp_idf_version.h` gives **5.5.5** - the design's correction of 5.5.1 is right.
- `httpd_open_func_t (httpd_handle_t, int sockfd)` and the `open_fn` config field exist
  (`esp_http_server.h:137`, `:253`), so overrides can be installed before parsing.
- `httpd_sess_set_recv_override` (`:798`), `httpd_sess_set_send_override` (`:818`) and
  `httpd_sess_set_pending_override` (`:838`) all exist.
- The `HTTPD_SOCK_ERR_FAIL / _INVALID / _TIMEOUT` contract is documented at `:703-705` with
  the semantics the design relies on.
- `MSG_DONTWAIT` is defined as `0x08` in the installed lwIP `sockets.h:272`.

Against spec section 5's three unresolved problems:

1. **Descriptor reuse** - removed by construction. No descriptor leaves HTTP execution, so
   there is no check-then-act window to lose. This is a stronger answer than the lifetime
   guarantee the spec asked for, because it removes the need for one.
2. **`httpd_stop()` blocks wherever it is called** - answered by the lifecycle worker. The
   component's stop path wakes the server task through the control socket, which is what
   makes a blocked `select()` interruptible; cancellable overrides are what make a blocked
   *handler* unwind so that stop can complete. Both halves are needed and both are present.
3. **Missing release needs a visible error and recovery** - answered by staged
   `RETRIEVAL_STUCK` with `stage=handler|reader_release|server_stop` plus an on-screen
   notice, while retaining the handle and exclusion rather than freeing on a timer.

The discipline elsewhere is also right: no HTTP callback into `diagreader::view/progress/
release`, writer cancellation A independent of transport release B, sticky stop per
lifecycle, refusing to advertise a 500 ms stop guarantee, and leaving `diagnosticsClose()`'s
bounded caller wait and `false`-on-timeout contract untouched.

## D1 - the idle-reset rule contradicts the approved spec

The design states: *"Favicon, status/result display, unknown paths and rejected methods
perform no SD work and do not trigger inventory refresh or reset idle time."*

Spec section 4, line 171: *"Any HTTP request resets it, including the last-result view and
favicon, as does a panel touch."*

That is a direct contradiction, in a document that opens by saying it "does not silently
amend the spec". It also reverses a point Codex itself made during the increment 2 review
("It is still an HTTP request under JP's agreed idle reset rule").

The design's position is defensible - the deadline should measure user presence, and an
idle Safari tab re-requesting a favicon should not hold the mode open indefinitely. The
practical difference is small, since the listing `GET` resets the clock either way and the
case that differs is an otherwise-idle tab. But **this is a change to a decision JP made,
so it needs JP's explicit agreement and a spec edit**, not a sentence in a design document.
Whichever way it goes, section 4 and this design must say the same thing before
implementation starts.

## D2 - workflow risk: the capability collides with how evidence is captured

The design proposes revealing the capability through a serial `log mode url` command, with
"that deliberate credential display is the sole serial exception and must be redacted from
captures sent back".

**This project commits console captures to git.** `docs/bench_data/*/console.txt` are
tracked files, and every bench case so far has been handed over as a raw console dump. A
single unredacted capture publishes the token into repository history, where removing it
means rewriting history. Manual redaction as the only control is the weakest possible
control for the one secret in the design.

Two alternatives avoid the exception entirely:

- **Show it on the panel.** The design already introduces a main-owned custom LVGL object
  for the stuck-teardown notice, so on-screen text is available in this increment. The URL
  and token on the panel need no serial path and no redaction discipline.
- **Defer the capability to the increment that first serves file bodies.** Increment 3
  serves a listing of names and sizes and refuses `/f/...` with 503. The exposure it
  protects against - other hotspot clients - is thin when nothing can be downloaded, and by
  the time bodies are served the entry screen decision is closer.

Also worth stating plainly for JP: a capability carried as `?k=` lands in Safari's history
and is synced with it. That is acceptable for a short-lived per-entry token, but it should
be a stated consequence rather than a discovered one.

## D3 - `httpd_sess_set_pending_override` is not addressed

The design installs send and receive overrides and claims cancellation covers "all input".
It does not mention the pending override, which the installed header exposes at `:838`.

The component uses the pending function in its session loop to decide whether a socket has
data worth dispatching. Leaving it at the default means one path still touches the socket
outside the cancellation check, and it means a cancelled session can still be selected and
dispatched before a handler sees the flag. That may be harmless - the handler refuses
immediately - but it is exactly the kind of gap that should be a stated decision rather than
an omission. State whether pending is overridden, and if not, why the default is safe.

## D4 - per-socket deadline state has no stated owner or reset point

The design specifies an absolute 5 s header budget "starting with the first received byte"
and a 5 s output budget "beginning with its first send". An override receives only
`(hd, sockfd, buf, len, flags)`, so both clocks require **per-socket state keyed by
descriptor**, and descriptors are reused across sessions.

`open_fn` is the natural place to allocate and reset that state, and the design already
uses it to install overrides, but none of this is stated. Specify the storage (a fixed
array sized to `max_open_sockets` plus reserved), the reset point, and what happens if a
socket is seen without a prior `open_fn` call.

## D5 - the listing response buffer is unspecified

The HTTP task is provisionally 4096 internal bytes, and a listing can carry up to 256
entries. Nothing states where the response is formatted, in what size chunks, or out of
which memory. That single decision touches the stack budget, the 20480-byte largest-block
gate and the chunking behaviour all at once. It should be a number in the design: buffer
size, PSRAM or internal, and whether the listing is emitted in one response or chunked.

## D6 - the stuck-teardown notice needs a parent and a clearing rule

"A main-owned persistent on-screen error notice using a custom LVGL object" is the right
approach and correctly avoids `ui/`. But an object created on the active screen disappears
when the user navigates. For a notice that must survive until reboot, `lv_layer_top()` is
the natural parent, since it persists across screen loads. State the parent, and state what
clears it - a late completion should remove it, and only a reboot otherwise.

## D7 - verify the enforced-close mechanism before approval, not during

The design says one response per socket is enforced "via the HTTP task's close path after
response completion (handler failure return where supported by the component; verify it
closes without emitting another body)".

That verification belongs before implementation approval, because the answer determines
whether a second request is servable at all, which in turn interacts with
`max_open_sockets=3` and the decision that second requests queue. It is a short read of the
installed component source, and the design already cites the right file. Settle it now.

## D8 - the worker's retained internal stack is a JP decision, not a provisional note

"One lazy-created persistent worker, 4096-byte internal stack ... keep it dormant after
OFF" means 4 KB of internal RAM is retained for the rest of the boot after the first entry,
in the budget that is already the binding constraint. The design is honest that this must
appear in the server-off regression and memory gates, which is right, but it presents as
provisional what is really a trade:

- retain 4 KB internal permanently, avoiding task deletion and recreation races; or
- create per entry and self-delete after publishing completion, paying the race risk the
  design explicitly wants to avoid; or
- place the worker stack in PSRAM, as the SD writer's already is, subject to the same
  unverified-lwIP-from-external-stack caveat - noting the worker itself calls only the
  component lifecycle API and does little socket work of its own.

Put the three in front of JP with a recommendation rather than deferring.

## Smaller points

- `max_open_sockets=3` plus three reserved internal sockets against the installed
  `LWIP_MAX_SOCKETS=16` is correct arithmetic, and the design is right to call it capacity
  rather than a memory claim.
- The claim that the polling interval is not `STALL_MS`, and that header bytes must never
  count as body progress, is exactly the distinction increment 4 will need. Keep it.
- "Do not return zero as a retry signal" is the right reading of the documented contract.
- The inventory design - writer-owned, published/staging pair, pins, preemption by a USB
  reservation, no append pause, `inventory pending` rather than an invented empty card - is
  sound and keeps USB untouched.

## Readiness

**Not yet.** D1 and D2 need JP's decision; D3, D4, D5, D6 need specifying; D7 needs a short
verification against the installed source. None is architectural - the hard part is already
right - so a short revision should reach approval.

Once resolved, fold the approved choices into spec sections 4, 5, 7, 9 and 12 as the design
proposes, and note that the idle-reset outcome must be written into section 4 explicitly
either way.
