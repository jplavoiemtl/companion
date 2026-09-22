# Increment 4 - Claude code review

Reviewer: Claude. Date: September 22, 2026. Subject: `ae919fb` against `de0b4e8`,
[increment 4 handoff](sd_iphone_log_download_increment4.md) and
[spec](sd_iphone_log_download_spec.md) revision 6.

**Verdict: no blocking defects. Ready for JP's build.** One recommendation that would turn a
silent corruption class into a detected one, and three low notes.

## Partial-send CRC accounting is correct

This was the thing to get right, and it is. I traced the full path because the dangerous
failure here is not a crash - it is a CRC that agrees with a corrupt file.

The mailbox publishes a chunk copy with an **absolute** offset. The HTTP task sends from
`state.data + (result.bytes - state.offset)`, so bytes already accepted are never resent,
and it reports a **cumulative** count. `diagtransfer::progress()` enforces monotonicity and
bounds it by `box.offset + box.length`. The writer then consumes only the delta:

```cpp
if (bytes>r.sentBytes && !diagreader::progressBytes(readerGeneration,size_t(bytes-r.sentBytes),at))
```

and `progressBytes()` credits CRC over exactly that prefix, advances `sentBytes`, reduces
`pendingBytes` and `memmove`s the unsent tail to the front. The delta can never exceed
`pendingBytes`, because `box.offset + box.length` is invariant under partial credit -
`sentBytes` rises exactly as `pendingBytes` falls - so `progress_refused` cannot fire
spuriously.

The mailbox is republished **only** once the whole chunk is acknowledged
(`if (s.length && bytes<s.offset+s.length) return;`), which is what keeps the unshifted copy
consistent with the HTTP task's own offset arithmetic. That is the invariant the whole
scheme rests on, and both halves honour it.

`httpd_send`'s semantics match the assumption. The installed header documents the return as
"Number of bytes that were sent successfully", with negative `HTTPD_SOCK_ERR_*` values, so
accumulating positive returns and treating `n<=0` as failure is correct.

**The body deadline is a no-progress clock, not an absolute one** - the distinction is
implemented inside the override, where partial writes also refresh it:

```c
if (n > 0) { if (io->bodyOutput) io->outputAt=nowMs(); return n; }
```

Header output keeps the absolute 5 s budget because `bodyOutput` is still false. I checked
this specifically: an absolute body deadline would have aborted any archive slower than 5 s,
which is entirely plausible over a phone link.

## Cancellation and release ordering

- **Every path releases.** After `diagtransfer::request()` succeeds there is no early return
  in `download()`; metadata failure, header failure, body failure, cancellation and success
  all fall through to the bounded close wait, `release(id,result)` and the END records. No
  reservation leak.
- **Release survives cancellation, by identity.** `release()` is accepted while
  `box.reserved && box.id==id && !box.released` regardless of `cancellation`, matching the
  invariant established in increment 1: an invalidated generation rejects *progress* but
  must still accept its matching *release*, or the buffer could never be freed.
- **Waits are on HTTP only.** The 5 s metadata wait and the 5 s close wait both run on the
  HTTP task; the writer never waits for the network, and `diagnosticsClose()` gains no wait.
  A pending close posts release anyway with `writer_close_pending` and raises lifecycle
  failure rather than force-freeing.
- **Lock order is consistent.** `request()` is the only nesting - transfer then reader - and
  `tick()` reads `acknowledged` under the transfer lock, exits it, then calls
  `progressBytes()`. `close()` calls the reader outside the transfer lock. No inverse
  nesting anywhere.
- **Stop and prune take the same close path.** `stop()` initialises state through
  `diagreader::start()` under an already-set `cancellation`, and `start()` checks
  `stopReason()` before any `stat`/`open`, so no SD is acquired during shutdown. `beforePrune`
  cancels and closes before unlink.
- `close("ok")` cannot race the body loop into a wrong result: the writer only sees
  `sentBytes==fileSize` after the final acknowledgement, by which point the HTTP loop has
  already exited on `result.bytes==result.expected`.

## Terminal-writer cleanup ownership

`offlineTick()` → `finishRelease()` requires `closed && released` and then calls
`diagreader::release()`, which refuses while `reader>=0 || paused`. Since `closed` implies
`closeReaderAndResume()` already ran, the fallback frees **only already-closed storage and
performs no SD work**. It is reached from `diagnosticsUsbOfflineTick()`, which main calls
only when `writerRunning()` is false, so it cannot overlap writer access. The extension is
sound and matches spec revision 6.

## USB compatibility

The adapter dispatches `HttpArchive` and returns early, and while `diagtransfer::busy()` it
pumps the transfer instead of the USB phase machine - but **still calls `controlTick()`**, so
USB status and error output stay responsive during an HTTP download. Framing, 144-byte
chunks, line progress, CRC, `STALL_MS`, `CURRENT_MS`, queue-pressure abort and release logic
are untouched, and the 16 connection/pacing and 12 logger-gate checks pass unmodified.
Archive transfers skip `hooks.begin`, so no `USB_GET_BEGIN` is emitted for an HTTP request -
correct, since HTTP emits its own records.

**202 host checks pass** across ten suites, counts confirmed by re-running them.

## Recommendation - the writer's CRC is computed and then discarded

Two independent CRCs exist over what must be byte-identical data: the HTTP task's `crc` over
what `httpd_send` accepted, and the writer's `r.crc` via `progressBytes()`. Only the HTTP one
reaches `HTTP_GET_END`.

They should always agree. If they ever disagreed it would mean the mailbox copy and the
reader buffer had diverged - which is precisely the corruption class a CRC over
"what we sent" cannot detect, because the CRC would faithfully describe a wrong byte stream.
Carrying the writer's CRC in `HTTP_GET_END` as a second field, or comparing them at close and
recording a mismatch, converts that from silent to caught. One field, and it makes the
strongest invariant in this increment self-checking.

Not a defect - I could not find a path where they diverge. It is cheap insurance on the one
thing that would be hardest to notice in the field.

## Note 1 - the terminal fallback's safety is by discipline, not by type

`readerGeneration` and `started` are plain non-atomic variables written by the writer and
read by main's fallback. The comment documents the handover, and the call-site gating makes
it correct today. But the design document's own rule - "ESP32 is 32-bit: protect 64-bit
generations/timestamps too" - is satisfied here only because the two tasks never run
concurrently, which is an invariant enforced elsewhere in the file. An assertion on the
writer lifecycle inside `offlineTick()`, or routing the fallback through the existing
snapshot lock, would make the argument local instead of distributed.

## Note 2 - `view()` copies 232 bytes under a critical section on the hottest path

`View` contains the 144-byte chunk plus ten 64-bit fields, and it is returned by value from
inside `portENTER_CRITICAL`. That happens on every body-loop iteration, every wait poll and
every writer tick. Each copy is short, but it is the most frequently executed
interrupt-disabled region added by this increment. A narrower accessor for the poll path -
offset, length, closed - would remove most of it without changing the design.

## Note 3 - watch `http_margin` first in gate A

`download()` adds meaningful depth to the 4096-byte HTTP task stack: a `Status`, a `Result`,
and a `View` copy live simultaneously with the component's parse frame. Nothing here is
unbounded, but nothing is measured either. `HTTP_GET_MEM` reports `http_margin`, which is the
right instrumentation - it is the number to read first when gate A runs, before the CRC
comparison.

## What this review does not establish

Source simulations with mocked platform calls; **nothing has been compiled**. No lwIP
timing, no throughput, no memory margin and no real partial-send behaviour is evidenced - a
real network is what produces partial sends at all, and gate A is the first time this code
meets one. `companion.ino` is unchanged, so no generated-sketch deletion is required.

## Recommendation

Build and flash, then run timing gate A on one small immutable archive. The comparison that
matters most is the exported Safari file against the card bytes, since that is the only check
that spans the whole path rather than the device's own view of it.
