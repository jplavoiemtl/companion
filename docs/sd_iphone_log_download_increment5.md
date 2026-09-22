# Increment 5 - representative 2 MiB archive

Status: approved by JP on September 22 after accepting increment 4. Timing gate B passed; explicit increment 5 acceptance pending.
No firmware changes, build or flash required. The accepted 6144-byte HTTP-stack build
already streams any managed archive with a fixed 144-byte mailbox and uint64 byte counts.
The handler runs until the frozen expected length; CURRENT_MS applies only to current.log.
The five-second no-progress and mode idle guards remain unchanged. This is validation of
larger-file behavior, not a new transfer implementation. Increment 6 remains unapproved.

## One next bench case - timing gate B

Use archive 21, last listed at 2097146 bytes (six bytes below 2 MiB). With USB power,
companion on the iPhone hotspot and web console DTR=true/RTS=false:

1. Capture status, log status and log list. Confirm archive-00000021.log is still
   2097146 bytes. If absent or different, stop and send the list for selection.
2. With retrieval mode OFF, log get 21 to retain the USB reference; require CRC OK.
3. log mode on; await ACTIVE. On iPhone Safari open the reported URL and download
   archive 21. Keep Safari foreground and phone awake for this normal successful case.
   Do not start any other transfer until completion.
4. Capture the last-result page and export the actual saved Safari file unchanged to PC.
5. Capture log mode status and status; log mode off, then log mode status to confirm OFF
   and server=off. Download current.log via USB, then capture log status.

Send the full console, USB archive reference, exported Safari file, last-result capture
and current.log (local Downloads paths suffice). No stopwatch or PC hotspot membership
needed. Device timestamps establish transfer timing; the actual saved file establishes
phone-side integrity. Do not infer Safari save duration from device send completion.

Evaluate all gate A metrics: exact saved bytes, both CRCs and independent byte comparison;
request/metadata/first/last/close/release times, maximum and terminal progress gaps,
append state, queue high-water/drops, internal free/largest, writer and HTTP stack margins,
clean mode exit and append continuation. Add sustained throughput over this representative
archive and assess whether progress gaps or errors grow with duration. Successful archive
throughput does not justify changing CURRENT_MS=120000 or STALL_MS=5000 and does not prove
current.log pause safety. Controlled failure/slow-client work remains in later increments.

After passing gate B, request JP's increment 5 acceptance and explicit increment 6 approval
before implementing current.log HTTP snapshot retrieval. Any resulting firmware change
requires Claude review before JP builds/flashes. The deferred configuration host assertion
can accompany later code work; it is not needed to run this unchanged-firmware case.


## Gate B result - September 22

JP reports pass; independently verified Safari/USB bytes equal (2097146), CRC 0B301A04,
dual CRC match, 41.482 s request-to-release, maximum gap 42 ms. HTTP margin 2524, reported
internal largest at least 34804, zero drops, appends continuing, clean recorded exit 102 ms.
See bench record for details and qualifications. No remaining evidence for this case.
Await explicit increment 5 acceptance and increment 6 implementation approval.
