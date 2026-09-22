# Increment 3 - Claude code review

Reviewer: Claude. Date: September 22, 2026. Subject: `dcac29f` against `2a9a9f6`, the
revised [design](sd_iphone_log_download_increment3_design.md) and
[spec](sd_iphone_log_download_spec.md) revision 4.
Handoff: [increment 3](sd_iphone_log_download_increment3.md).

**Verdict: no blocking defects. Ready for JP's build.** Two low maintenance notes, neither
worth holding the build for.

## The double-close finding is real - verified upstream

This was the item to check hardest, and it holds. Fetched from IDF **v5.5.5** source:

`httpd_sess.c`, `httpd_sess_new()`:

```c
if (ret != ESP_OK) {
    httpd_sess_delete(hd, session);   // closes the fd
    return ret;
}
```

`httpd_main.c`, `httpd_accept_conn()`:

```c
if (ESP_OK != httpd_sess_new(hd, new_fd)) { goto exit; }
...
exit:
    close(new_fd);
```

So returning `ESP_FAIL` from `open_fn` closes the descriptor **twice**. On this firmware
that is not theoretical: MQTT, the shared TLS client, lwIP and SD descriptors are all
created from other tasks, so a descriptor number freed by the first close can be handed to
another task before the second close destroys it. Rejecting a connection the obvious way
would have been a latent cross-subsystem descriptor corruption. Good catch.

**The mitigation is correct, and its load-bearing assumption checks out.** `rejectSession()`
returns `ESP_OK` and queues `httpd_sess_trigger_close()`, so exactly one close happens and
the component owns it. That only works if the session is findable from inside `open_fn`, so
I verified the ordering in `httpd_sess_new()`:

```c
session->fd = newfd;
session->handle = (httpd_handle_t) hd;
...
hd->hd_sd_active_count++;
if (hd->config.open_fn) { esp_err_t ret = hd->config.open_fn(hd, session->fd);
```

`fd` is assigned and the session counted **before** `open_fn` is invoked, and
`httpd_sess_trigger_close()` looks up by fd through `httpd_sess_get()`. So the lookup
succeeds, the close is queued, and `ESP_ERR_NOT_FOUND` - which would have tripped
`setFailure("session_close_queue")` and torn down the mode on every rejected connection -
cannot occur on this path.

The ordering inside `openSession()` is also right: **both overrides are installed before
any rejection check**, so even a rejected session has fail-closed I/O if the component
touches it before the queued close runs. `session()` returns null on missing context, fd
mismatch or stale generation, and both callbacks map null to `HTTPD_SOCK_ERR_FAIL`.

## Enforced close - also verified upstream

`httpd_uri.c` confirms the handler contract the design relies on:

```c
if (uri->handler(req) != ESP_OK) {
    ESP_LOGW(TAG, LOG_FMT("uri handler execution failed"));
    return ESP_FAIL;
}
```

No `httpd_resp_send_err`, no second body. Returning `ESP_FAIL` after a complete
`httpd_resp_send` closes the connection cleanly, which is exactly what `handle()` does on
every route.

## Lifecycle and cancellation

- **Admission is atomic.** `HandlerGuard` tests `ready && !cancel && generation` and
  increments the handler count inside one critical section, closing the check-then-enter
  gap. The destructor decrements unconditionally when admitted.
- **Teardown ordering is correct and each step is justified**: `diaginventory::stop()` →
  drain handlers → `httpd_stop()` → `dispose()` (requires writer quiescence, matching
  generation and zero pins) → free the page → publish `Off`. The page is freed only after
  `httpd_stop()` returns, so no handler can be formatting into it.
- **A failed stop parks rather than guesses.** `httpd_stop() != ESP_OK` sets
  `Phase::Failed`, keeps the handle, and the worker suspends forever. `stopped()` then
  never returns true, so main holds `STOPPING` and media exclusion until reboot - which is
  the design's stated recovery, not a silent leak.
- **Re-entry after a failed stop is refused.** `start()` returns false unless
  `phase == Off`, so a parked worker cannot be re-notified into a hang. I checked this
  specifically because a parked task consuming notifications would otherwise strand the
  mode in `STARTING`.
- **Cancellation reaches blocked I/O.** Both overrides re-check `cancelled()` each pass and
  return `HTTPD_SOCK_ERR_FAIL` - not `TIMEOUT`, which the parser may retry. The comment
  says so explicitly, and it is the right distinction.
- **Deadlines are absolute and separated.** Header time runs from first byte, output from
  first send, and `headerComplete` is set at admission so the header clock stops once a
  handler owns the request.

## Cache ownership and USB preservation

- `pin()` and `unpin()` are non-blocking critical sections, so **a handler can never block
  a teardown by waiting on a pin**. I checked this because the worker's
  `while (snapshot().handlers) nap();` would otherwise be an unbounded wait.
- `formatPage()` unpins **before** any network send, and unpins on the overflow path too -
  the `unpin` sits after the loop, which exits early on `ok == false`. No pin leak on a
  truncated page.
- The HTTP path performs **zero** SD calls: a grep for `opendir|readdir|stat|fstat|open|
  read|close` in `diagnostics_http.cpp` returns 0. Favicon, 404, 405, `/f/` 503 and
  `/result` all avoid SD and the reader reservation entirely.
- The only `diagreader` call on the HTTP task is `busy()`, a locked scalar read used to mark
  the listing stale. That stays within the design's rule, which named
  `view/progress/release` as the writer-only surface.
- USB changes are confined to one writer-only preemption hook. Framing, chunk sizes,
  deadlines, CRC, queue-pressure abort and release logic are untouched, and the existing 16
  connection/pacing plus 12 logger-gate checks still pass unmodified.

## Spec conformance

Activity resets on `/` and `/result` only - `HandlerGuard(io, method == GET && content_len
== 0 && (listing || result))` - with favicon, unknown paths and rejected methods excluded.
That matches spec revision 4 exactly. `idleExpired()` decides and publishes cancellation
under one lock, so a late request cannot revive a `STOPPING` session.

The stuck notice is parented to `lv_layer_top()`, survives screen loads, and is cleared
correctly: `notice(releaseWarned)` is the last statement in the tick, **after** the
`Stopping → Off` transition clears `releaseWarned`, so a late recovery removes the label in
the same tick. I traced the early `if (mode == Mode::Off) return;` to be sure the clearing
call is still reached on the recovery tick. It is.

**172 host checks pass** across nine suites, counts confirmed by re-running them.

## Note 1 - `idleExpired()` mutates state behind a predicate name

It sets `cancel` and `ready` when it returns true. That is deliberate and correct - the
atomic decide-and-cancel is what prevents a late request reviving `STOPPING` - but the name
reads as a pure query. A future caller adding it to a status display would silently cancel a
live session. A name like `takeIdleExpiry()` would make the side effect impossible to miss.

## Note 2 - the worker polls every 100 ms for the whole session

```c
while (!cancelled(generation)) { ulTaskNotifyTake(pdTRUE,pdMS_TO_TICKS(100)); sampleWorker(); }
```

That is ~3000 wakeups across a five-minute session, on a low-priority task, purely to
resample a high-water mark that only ever decreases and has already peaked during
`httpd_start()`. A longer interval, or relying on the stop notification plus one final
sample, would cost nothing in evidence. Not a defect; it just adds scheduler noise to a
system where loop gaps are measured.

## What this review does not establish

Source simulations with mocked platform calls. **Nothing has been compiled**, and this
increment adds two translation units, the first `xTaskCreateStaticPinnedToCore` outside the
SD writer, and the first `esp_http_server` usage in the project - all places a compile or
link error would surface first. No RTOS scheduling, lwIP timing or memory margin is
evidenced.

`companion.ino` is unchanged, so the generated-sketch deletion is **not** required this
time; a clean build remains the fallback if the new translation units are not picked up.

## Recommendation

Build and flash. The first bench case carries an extra job beyond its own gate: it is the
deterministic check on the lifecycle worker's PSRAM stack making the firmware's first lwIP
calls, since `httpd_start()` creates the listener and control sockets on the worker. If
`httpd_start()` misbehaves, move the worker to an internal stack through review and rebuild
rather than debugging PSRAM and lwIP interaction.
