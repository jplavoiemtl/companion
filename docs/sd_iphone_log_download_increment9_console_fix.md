# Increment9 console busy-refusal correction - for Claude review

September24. JP's first USB-during-HTTP case exposed an actual console defect.
Firmware correctly refused with busy. The USB page treated refusal before BEGIN as a
reason to send global log abort, cancelling HTTP, then disconnected after8 seconds
because HTTP completion does not produce the USB abort confirmation it expected.
Full evidence and device cleanup timestamps are in the bench document's latest entry.

## Change

In tools/sd_log_browser.html, protocolLine settles a pre-BEGIN busy request locally
with Device: busy. No abort is sent and no abort timer is installed. Busy after BEGIN
still leaves the accepted USB transfer alone. Explicit cancel and malformed/CRC error
handling are unchanged. Firmware and historical plan are untouched.

The new tools/tests/sd_log_browser.test.cjs regression executes the real page: refused
request resolves false, no file saved, only log get17 sent, no8-second abort timer,
connection retained and later successful USB retry. Existing started-transfer busy and
damaged-data abort-barrier tests remain. Test harness exposes timer entries to assert
against the abort timer rather than ordinary UI timers. The regression failed on the
old implementation; all20 browser tests now pass. Also16 connection/pacing and12 logger
gate checks pass,48 total. No firmware build or flash; no claim of hardware pass yet.

## Review focus and next action

Check the pre-BEGIN ownership distinction and preservation of existing USB error paths.
After Claude clears the diff, JP disconnects/reloads the corrected local USB console,
reconnects with DTR=true/RTS=false, and repeats only increment9 USB-during-HTTP.
Expected busy must leave HTTP running and the console connected. No board rebuild or
flash. Reverse-direction case remains pending; no later feature implementation here.

## September24 - Claude review cleared, supplied by JP

No blocking issues; ready for the single bench repeat. Claude independently reports
48 passing checks and confirms the new regression fails with the old page. Ownership,
post-BEGIN busy, explicit cancellation and all existing error/abort-barrier paths were
reviewed as preserved. Keep the separate commented busy branch for clarity.

Deferred protocol note: errors carry no request ID. A delayed busy reply from an earlier
command could theoretically settle a newer pre-BEGIN request. No practical reproduction
identified in this review; pre-existing, not worsened by the fix. Revisit on a future
protocol change, not an additional bench gate now.

Repeat only USB-during-HTTP after disconnecting/reloading/reconnecting the local console.
Require busy refusal, no automatic TX log abort, console connected beyond8 seconds,
and normal HTTP completion. Export this repeat's Safari archive21 to PC Downloads as
well as the later USB current.log so the reported HTTP CRC can be checked against the
actual saved phone bytes, as requested by the review. Previous procedure did not require
this export; this clarification adds evidence to the same repeat, not another test.
Increment9 remains pending; no firmware rebuild/flash.
