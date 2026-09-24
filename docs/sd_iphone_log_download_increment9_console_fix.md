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
