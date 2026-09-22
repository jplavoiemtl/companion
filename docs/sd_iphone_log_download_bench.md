# iPhone log retrieval - bench cases and results

## September 22, 11:31 - increment 3 corrected admission passes; first case completion pending

Reviewed checkpoint 767cbbc; JP reports the test passed. Evidence: console attachment
265f1cd4-4897-4190-9b44-89cca49a461e/Pasted text.txt and pasted laptop curl output.
Boot 108, same profile amoled-1-8-core-3-3-11, normal diagnostic switches.

- STARTING at 11:31:31.764, ACTIVE at 11:31:31.768. Worker PSRAM placement and internal
  TCB confirmed; worker margin 2656 bytes, HTTP margin 1688 bytes after requests.
- Admission reaches accepted=2, then 3; rejected=0, last_reject=none, error=none.
  This hardware run verifies the corrected dual-stack admission on this connection path.
- Laptop favicon GET returns HTTP/1.1 204 No Content, Content-Length: 0,
  Connection: close, Cache-Control: no-store and Referrer-Policy: no-referrer.
- Logger ready, drops=0, high=7, slow=0; internal_min=88804, reported internal_largest=49140.
  Probe largest_min reaches 47092, above the 20480-byte gate. No reset in this capture.
- USB current transfer succeeds while mode remains ACTIVE: 210225 bytes, 2.13 seconds,
  CRC OK. Later current_size=210388, confirming 163 bytes of append growth;
  active=0, paused=0, queue=0/16, result=ok. One 5 ms USB link loss recovered within grace.

The requested manual stop was not executed: at 11:33:20.347 the command was literally
`Run log mode off`, rejected as unknown. At 11:33:31 mode was still ACTIVE. No OFF/server=off
observation follows. This is a procedure correction, not a demonstrated shutdown defect.
JP's message ends with "Then I got this in the web UI:" but the UI detail is missing;
listing-name comparison and Last result content cannot yet be independently marked complete.
The downloaded file itself was not supplied in this turn; CRC evidence is the console report.

Continue only the outstanding portion of this same case, without rebuilding: capture
log mode status; if already OFF through idle timeout, enter mode again; issue exactly
log mode off, wait two seconds, then log mode status, status and log status. Request the
missing web UI detail and whether listing/Last result matched expectations. No repeat
USB download needed on this evidence. Increment 3 gate remains incomplete; increment 4
unapproved. No firmware changes or assistant build/flash.


Running bench record for the wireless retrieval feature, one case at a time.
Design: [Codex review](sd_iphone_log_download_review.md) and
[Claude review](sd_iphone_log_download_review_claude.md). Historical draft:
[sd_iphone_log_download_plan.md](sd_iphone_log_download_plan.md).
Branch `iphone-log-retrieval`. Results are appended newest first as cases complete.

Status: design review closed on September 20, 2026 with six corrections accepted.
**Case 1 passed on September 20, 2026**, with transfer timing deliberately deferred. No
wireless firmware exists for this feature yet. The USB-only extraction is now implemented
and reviewed by Claude at `0ba72e5`; its host checks and issued first bench gate are recorded in the
[increment 1 handoff](sd_iphone_log_download_increment1.md). All five issued USB regression gates and the restored normal-build handoff passed on
September 21. JP accepted increment 1 and approved increment 2 on September 21. Increment 2 is
reviewed by Claude; its first hardware entry/exclusion/exit gate passes below. Historical acceptance-pending statements below predate that decision.

---

## Increment 3 first server case - September 22, 11:13-11:18 - INCOMPLETE / HTTP RESET

Evidence: attachment `0851f9d1-399c-4c56-8385-630ab4d31280/Pasted text.txt`, boot 106,
plus JP's untimestamped verbose curl transcript in the conversation. Curl established
TCP to 172.20.10.2:80, sent GET /favicon.ico, then received reset before response headers.
The curl transcript cannot be aligned precisely to individual mode-status timestamps.
No iPhone listing result or completed HTTP 204 is evidenced; the whole gate is not passed.

At 11:13:30 mode goes STARTING -> ACTIVE in 4 ms, worker_external=1, tcb_internal=1,
worker_min=2656. At 11:17:53 mode remains ACTIVE, idle_ms=262655, error=none,
http_min=0. At 11:18:36 it is OFF/idle_timeout/server=off, worker_min=2304. Same boot,
no reset in the capture. Thus first httpd_start and stop on the PSRAM worker succeeded;
there is no reason from this result to invoke the internal-stack fallback. This is not
proof of all PSRAM/lwIP operations. HTTP minimum staying zero supports failure before a
successfully admitted session; it is not a recorded rejection reason. Probe minima shown
remain above the 20480-byte gate. Initial logger status has drops=0 and one historical
slow flush (113459 us); there is no full post-case logger status or current.log download.

### Code defect and proposed correction, awaiting Claude review

Installed sdkconfig enables CONFIG_LWIP_IPV6. IDF 5.5.5 httpd_server_init therefore uses
PF_INET6 for its listener, serving IPv4 through mapped addresses. The implementation's
getsockname buffer was sockaddr_in and admission required AF_INET. This rejects valid
IPv4-mapped local addresses, consistent with the reset and http_min=0. Runtime family was
not captured by the old firmware, so this explains a verified code defect rather than
claiming a directly measured rejection reason for every curl attempt.

Correction uses sockaddr_storage and validates length/family, accepting native IPv4 or
exact ::ffff:IPv4 only when the final IPv4 bytes equal the hotspot STA address. Native
IPv6, non-mapped addresses, truncated addresses and other local addresses stay refused.
Status now retains accepted/rejected counters and last rejection reason per mode entry,
without emitting per-request SD records. No capability or native-IPv6 service is added.
Five new source-body checks cover these cases. No firmware build/flash by Codex.
Claude must review before JP rebuilds. Resume this same first case after review; no
additional case or increment is approved by this failed attempt.

---

## Increment 2 accepted - September 22, 2026

JP explicitly accepted increment 2 after gate 11, including the preceding proposal to
defer battery-only entry refusal and brief pending-handover entry refusal hardware checks
until controlled bench coverage before car deployment. All eleven issued cases passed;
the two deferred cases remain unproven on hardware. No additional test or flash is needed
for this acceptance. Earlier pending-acceptance statements below are historical.

Next: prepare increment 3's lifecycle/descriptor ownership design for Claude review,
along with provisional socket budget and capability decision required by spec section 12.
Increment 3 implementation remains unapproved and requires JP's separate authorization.

---

## Increment 2 gate 11 - 2026-09-22, 08:37-08:41 - PASS

Remote MQTT notification exclusion and still-to-Live recovery, boot 104; JP confirms pass.
Evidence: attachment `17e8481e-f4a5-4863-880e-f1d3f4ee314c/Pasted text.txt`, Downloads
`104-current (1).log`.

ACTIVE at 08:37:52; remote refusal at 08:38:11 is persisted as MQTT_IMAGE
result=ignored_download_mode seq=133, with no IMAGE_BEGIN/LIVE_BEGIN for that attempt.
The intermediate ACTIVE query after refusal was omitted; no exit is recorded until the
explicit off command at 08:38:45, followed by exit completion seq=135 and OFF status.
Fresh notification at 08:39:22 yields IMAGE_BEGIN trigger=mqtt, accepted MQTT_IMAGE,
and IMAGE_END displayed/HTTP 200, expected=received=34207. LIVE_BEGIN seq=146 has
trigger=motion_handover. LIVE_END seq=157 is duration/failure=none/HTTP 200,
**165 frames / 60368 ms = 2.73 fps**. This is functional recovery, not a paired FPS gate.

Download **1827146 bytes**, 16.92 s, browser CRC OK; local length matches and computed
CRC32 **1298A90B**. Post-Live full status: ready, drops=0, error=none, stack_min=2920,
internal_min=38032, internal_largest=26612 > 20480. After download, USB active=0,
paused=0, result=ok, queue=0/16, drops=0, no retained USB failure; current_size=1827313
exceeds the downloaded snapshot by 167 bytes, confirming append continuation. Same boot,
no unexpected reset visible; JP reports the test passed. No firmware changes.

### Acceptance coverage proposal - awaiting JP, not a change to the approved spec

All eleven issued increment 2 cases pass. Section 11 of the spec still calls for battery-only
entry refusal and entry refusal during the brief pending motion handover. Neither has been
observed on hardware. Host source simulations cover usb_power_required and display_pending;
the completed-display pending guard and real VBUS-loss exit have hardware evidence, but
these do not establish the two missing cases.

Recommend accepting increment 2 with those two hardware checks explicitly deferred to a
controlled bench procedure before car deployment, using later entry UI or an approved
bounded fixture if necessary. Do not silently mark them passed or assume a timed manual
command catches the short gap. JP must approve this coverage exception; otherwise design
one targeted case at a time. No further bench case is issued pending that decision.
After acceptance, propose resolving increment 3 descriptor lifetime and lifecycle teardown
ownership with Claude review before JP explicitly authorizes implementation. Increment 3
remains unapproved. Historical draft unchanged.

---

## Increment 2 gate 10 - 2026-09-22, 08:15-08:16 - PASS

History-image Back exclusion/recovery, boot 104; JP confirms pass. Evidence: attachment
`b34e9571-c1bc-4456-abce-1462e7212b7c/Pasted text.txt`, Downloads `104-current.log`.

Mode ACTIVE at 08:15:09.614; Back at 08:15:23 refused download_mode, with persisted
IMAGE_REFUSED trigger=history_back seq=37 and no IMAGE_BEGIN for the refused request.
Status remains ACTIVE. Exit requested 08:15:44.897; saved exit completion seq=42 precedes
allowed IMAGE_BEGIN seq=44 at 08:15:53.402. The explicit OFF query was omitted, but the
saved ordering confirms completed exit. IMAGE_END seq=49: displayed, HTTP 200,
34586 expected/received bytes, total_ms=1362. JP reports normal operation.

Current download **1795870 bytes**, 16.59 s, browser CRC OK; local size matches and
computed CRC32 **12B27089**. Before download: ready, drops=0, error=none, queue=0/16,
internal minimum=43708, largest=31732 > 20480, stack minimum=3208. No post-download
resource status was captured; these are pre-download observations. No unexpected reset
or stall is visible in this capture. This is a new sitting/boot, not a paired comparison
with yesterday. No firmware changes.

### Next single case issued: remote notification exclusion and handover recovery

Same build/no flash, USB and hotspot connected, DTR=true/RTS=false, browser test switches
off, dashboard idle. Clear console, send status then log mode on/status; require ACTIVE.
Use the normal entrance-camera system to send one real MQTT latest-image notification
(not movement of the companion itself). Expect refusal with no still or Live, then mode
status remains ACTIVE. Send log mode off/status; require OFF. Wait at least 15 seconds
after any preceding image display, then trigger a fresh camera notification. Expect the
normal brief still followed by Live; allow the full cycle to finish. Capture status and
log status, download current with CRC OK, then log status again to check unpaused/idle
and drops. Send console, downloaded file and visual observations. If a notification
cannot be triggered or received, report that rather than substituting a local button.

This covers real MQTT admission and successful still-to-Live handover after exit; it does
not prove entry refusal during the short pending-handover gap or battery-only entry.
Those coverage decisions remain open before increment 2 acceptance. Increment 3 remains
unapproved and requires lifecycle/descriptor decisions before implementation.

---

## Increment 2 gate 9 - 2026-09-21, 15:19-15:22 - PASS

Direct Live exclusion while ACTIVE, boot 103; JP confirms pass. Evidence: attachment
`6c60d715-acac-423c-93a6-00c7814708ea/Pasted text.txt`, Downloads
`103-current (9).log`.

ACTIVE at 15:19:26; button at 15:19:40 refused download_mode. Persisted LIVE_REQUEST
seq=338 is refused, with no LIVE_BEGIN for that attempt; subsequent status remains ACTIVE.
Mode exit completes seq=340 before the next LIVE_BEGIN id=4. The explicit OFF query was
omitted, but persisted completion establishes ordering. Allowed Live ends reason=duration,
failure=none, http_code=200, 183 frames/60397 ms = **3.03 fps** (not a paired benchmark).

Download **1224471 bytes**, browser CRC OK, 11.63 s; local length matches, computed
CRC32 **56240213**. Pre-download status ready, drops=0, error=none, queue=0/16,
internal minimum=34304 and largest=24564 > 20480, stack minimum=2920. Largest retained
block is lower than the prior 26612 observation; no leak or cause is inferred from these
boot-retained minima. No unexpected reset/stall visible. Same boot, normal flags.

### Next single case issued: history-image Back exclusion and recovery

Same build/no flash, dashboard, USB/hotspot connected, browser test switches off. Mode
on/status -> ACTIVE. Press the dashboard history-image Back button (not screen-return
navigation) once: expect Back refused: download mode, no image/loading screen. Mode
status stays ACTIVE. Mode off/status -> OFF. Press the same history-image Back button
once; expect normal older-image retrieval. After it displays, return to dashboard via
normal navigation; status/log status, then current download (CRC OK). Send console/file
and screen observations. If the server has no older image, report that result rather
than counting successful rendering. This covers the remaining local still caller;
remote MQTT/handover and battery-entry evidence still need an explicit coverage decision
before increment 2 acceptance. Increment 3 unapproved; no new firmware changes.

---

## Increment 2 gate 8 - 2026-09-21, 15:16-15:18 - PASS

Explicit mode exit during current USB transfer, boot 103; JP confirms pass. Evidence:
attachment `a71bd6ce-3153-4649-9902-a05ad1a5be00/Pasted text.txt`, Downloads
`103-current (8).log`. Earlier OFF status in this capture preceded the omitted on command;
JP then sent on successfully at 15:16:14.930. No defect was observed at initial setup.

Current transfer started 15:16:30.875; off sent 15:16:32.853, followed by STOPPING and
Device: aborted. Persisted order: mode stopping seq=322 up_ms=3070134; USB_GET_END
seq=323 up_ms=3070138, bytes=206208, duration_ms=1990, result=aborted; mode exit ok
seq=324 up_ms=3070158. This supports cleanup before completed mode exit; record spacing
is not a separately instrumented release/close latency measurement.

Subsequent mode status OFF/release_stuck=0, USB active=0/paused=0/result=aborted,
queue=0/16, drops=0, no retained failure. Retry **1217047 bytes**, browser CRC OK,
11.45 s; local size verified, calculated CRC32 **AFD46D5A**. Same boot. The only full
resource status is before the test (largest=26612, stack_min=2920); no post-retry memory
or status observation is claimed. Retry file preserves cancellation evidence and
records appended afterward. No firmware changes.

### Next single case issued: direct Live refusal while mode ACTIVE

Same build/no flash, USB/hotspot connected, dashboard idle, test switches off. Clear
console; mode on/status must be ACTIVE. Press Live once: expect Live refused: download
mode, no loading screen/frames/navigation. Send mode status, still ACTIVE. Mode off/status
must reach OFF. Press Live once and allow the full normal cycle to complete. Then
status/log status and current download (CRC OK). Send console/file and whether the first
press stayed on dashboard and the second played normally. Earlier live_busy gate tested
the opposite admission direction; this covers the direct Live guard while mode is active.
Remaining coverage is to be reviewed before increment 2 acceptance; increment 3 unapproved.

---

## Increment 2 gate 7 - 2026-09-21, 15:09-15:11 - PASS

USB-power-loss exit without reboot, boot 103; JP confirms pass. Evidence: attachment
`515d0129-20ce-43e5-b398-2ad7932114ae/Pasted text.txt`, Downloads
`103-current (7).log`.

ACTIVE confirmed before unplug. Host device removal 15:10:15.546, available again
15:10:23.350. Persisted boot-103 mode exit usb_power_lost/stopping at up_ms=2692876
and ok at 2692880. Post-reconnect status confirms OFF/reason=usb_power_lost,
release_stuck=0; same boot 103/continuous uptime. No explicit off command or reboot
explains this exit. Record spacing is not a physical VBUS-to-exit latency measurement.
Movement occurred while handling the board; power loss is the recorded exit cause.

Download **1209578 bytes, browser CRC OK**, 11.29 s; local length matches, calculated
CRC32 **54FDFD6B**. Pre-download status ready, drops=0, error=none, queue=0/16,
internal largest=26612 > 20480, stack minimum=2920, continued append growth. Initial
USB active=1 on reconnect belongs to automatic listing; subsequent status is idle.
No unexpected reset/stall visible. This is idle-mode VBUS exit, not battery-entry
refusal or power loss during a transfer. No firmware changes.

### Next single case issued: explicit mode exit during current USB retrieval

Same build/no flash, USB/hotspot connected, DTR=true/RTS=false, browser test switches off.
Capture status and mode on/status (ACTIVE). Pretype log mode off in the command box but
do not send yet. Download current.log; while progress is advancing (about two seconds
into the current roughly 11-second transfer), send that command. Expect STOPPING,
Device: aborted, no saved partial file. Send mode status and log status after the abort:
require OFF, release_stuck=0, active=0, paused=0, result=aborted. Stop on mismatch.
Download current normally without re-entering mode (CRC OK); status/log status. Send
console and successful file. If download finished before off arrived, report that timing;
it did not exercise cancellation. This validates main exit -> writer cleanup/release ->
OFF and subsequent reuse. Other admission/exclusion coverage remains for final review;
increment 3 unapproved.

---

## Increment 2 gate 6 - 2026-09-21, 15:04-15:07 - PASS

Wi-Fi loss/recovery retains mode and timer, boot 103. JP confirms pass. Evidence:
attachment `e7de58ac-b669-4db7-9842-c4d7fbc826e1/Pasted text.txt`, Downloads
`103-current (6).log`.

ACTIVE entry 15:04:45.934; offline status confirms Wi-Fi OFFLINE/MQTT DISCONNECTED
and mode ACTIVE/link=down/idle_ms=41560. Persisted RETRIEVAL_LINK down at up_ms=2386751,
up at 2420597: **33846 ms** observed link-down interval, not a reconnect latency measured
from hotspot re-enable. Wi-Fi/MQTT recover; mode ACTIVE/link=up/idle_ms=70408, consistent
with original entry (no reset). Explicit off completes OFF with release_stuck=0.

Download **1203556 bytes**, browser CRC OK, 11.39 s; local size verified, computed
CRC32 **08231D6E**. Last pre-download status ready, drops=0, error=none, queue=0/16,
internal largest=26612 > 20480, stack minimum=2920, no USB losses/retained failure;
same boot, ongoing append growth. This tests mode/link policy, not HTTP listener recovery
(no server exists) or HTTPS readiness. No firmware changes.

### Next single case issued: observed USB power loss exits mode

Same build, no flash, battery connected/charged and hotspot available. No media/downloads
in progress. Explicit DTR=true/RTS=false and all browser switches off. Clear console;
send mode on/status and confirm ACTIVE, then status/log status to record boot. Physically
unplug USB for about five seconds, then reconnect promptly (before the existing 30-second
power-loss grace). Do not send mode off. Reconnect Chrome, preserving console output,
and send mode status: expect OFF/reason=usb_power_lost/release_stuck=0. Send status/log
status, then download current once with CRC OK. Send console/file and whether board stayed
on without reboot. If boot changed, report it; OFF after a restart alone cannot prove
power-triggered mode exit. Use persisted RETRIEVAL_MODE exit evidence to evaluate the gap.
This is idle-mode power loss, not transfer interruption or battery-only entry admission.
Increment 2 acceptance remains pending; increment 3 unapproved.

---

## Increment 2 gate 5 - 2026-09-21, 14:52-15:01 - PASS

Panel-touch idle reset, boot 103; JP confirms pass. Evidence: attachment
`4456aa00-915b-4ccb-8e5b-46c8ec8ea528/Pasted text.txt`, Downloads
`103-current (5).log`.

Entry up_ms=1657076 (14:52:59.836). Touch is visible at host 14:54:59.687; mode status
at 14:55:04.092 shows ACTIVE/idle_ms=4417, proving reset. Persisted idle exit at
up_ms=2076925 (14:59:59.685) is **419849 ms after entry**, approximately five minutes
after touch. The original-deadline status query was omitted; the persisted later exit
plus reset observation establish the intended behavior without a repeat. Host touch and
device record timestamps are not an exact same-clock latency measurement. Movement
occurred shortly after entry without exiting mode; no extra panel touch is visible.

Final mode OFF/idle_timeout/release_stuck=0. Download **1190003 bytes**, browser CRC OK,
11.26 s; local length matches and calculated CRC32 **BA9A10A9**. Pre-download status
ready, zero drops/errors, queue=0/16, largest block=26612 > 20480, stack minimum=2920,
no USB loss/retained failure. Same boot, Wi-Fi/MQTT connected; appends continue.

### Next single case issued: Wi-Fi loss/recovery retains mode and idle age

No flash; same build, USB connected, dashboard idle, no board touches/media/downloads.
Use the actual hotspot/AP currently serving the companion; serial off is MQTT-only and
cannot test Wi-Fi loss. Clear console, mode on -> ACTIVE, mode status capture. Disable
that hotspot for about 30 seconds; send status and mode status, expecting WiFi disconnected
and ACTIVE/link=down. If Wi-Fi remains connected (for example another configured network),
stop and report that instead of claiming a loss. Re-enable hotspot, wait for status to
confirm Wi-Fi/MQTT recovery, then mode status: ACTIVE/link=up, idle_ms continuing forward
rather than near zero. Complete within five minutes of entry so idle expiry is not a
confound; report delay/expiry rather than re-entering silently. Mode off/status -> OFF;
status/log status and one current download with CRC OK. Send console/file and note any
touch or power interruption. No iPhone wireless download/server exists at this increment.
Other power/admission gates remain pending; increment 3 unapproved.

---

## Increment 2 gate 4 - 2026-09-21, 14:42-14:49 - PASS

Five-minute idle expiry, boot 103; JP confirms pass. Evidence: attachment
`5b0a46a7-973b-4b61-8a7a-da2f2a5d6ac5/Pasted text.txt`, Downloads
`103-current (4).log`.

Entry seq=149 up_ms=1053110. Intermediate mode status is ACTIVE/idle_ms=121654.
Persisted idle_timeout stopping and ok records seq=165/166 are both at up_ms=1353119:
**300009 ms after entry**. Thus the status request did not reset idle and exit occurred
independently before the final status query, which confirms OFF/release_stuck=0.

Download **1175630 bytes, browser CRC OK**, 11.06 s; local length matches, calculated
CRC32 **63123CC4**. Pre-download status ready, drops=0, error=none, queue=0/16;
internal largest=26612 > 20480, writer stack minimum=2920, no USB losses or retained
failure, same boot and Wi-Fi/MQTT connected. No screen touch visible during the interval.
No post-download status supplied; no stronger resource claim made. This is idle with no
transfer; future HTTP archive-mid-transfer expiry remains a separate later gate.

### Next single case issued: panel touch resets idle deadline

Same build, no flash; USB/hotspot connected, dashboard idle, all browser switches off.
Clear console, send mode on and confirm ACTIVE. At about two minutes after entry, tap
once on an unused dashboard area (not a media/navigation button), then immediately send
mode status: require ACTIVE and idle_ms near zero. Do not touch again. At five minutes
ten seconds from original entry, send mode status: require still ACTIVE, idle_ms about
190000. At five minutes ten seconds from the touch, send mode status: require OFF,
reason=idle_timeout, release_stuck=0. Use one clock/timer and note both entry/touch times.
If the touch does not reset idle, stop and send the capture. After expected expiry, send
status/log status and download current (CRC OK). Send console/file and any extra touch
or connection interruption. This is one touch-reset case; other link/power/admission
gates remain pending. Increment 3 unapproved.

---

## Increment 2 gate 3 - 2026-09-21, 14:37-14:39 - PASS

Active-Live entry refusal, boot 103. JP confirms pass. Evidence: attachment
`64484ff4-6ea6-4ef4-9541-e79a146d7cea/Pasted text.txt` and Downloads
`103-current (3).log`.

Live started 14:37:24; mode on at 14:37:30.561 refused live_busy, state OFF.
Live continued to duration completion: persisted LIVE_END failure=none, http_code=200,
173 frames / 60197 ms = **2.87 fps**, no performance comparison claimed. Mode entry
succeeded after completion; off then status confirmed OFF, release_stuck=0. Persisted
mode events confirm refusal, later entry and exit completion.

Download **1164408 bytes**, browser CRC OK, 10.96 s; local length verified, computed
CRC32 **11A657C5**. Pre-download status ready, drops=0, error=none, queue=0/16,
internal largest=26612 > 20480, stack minimum=2920. Same boot, no unexpected reset/stall
visible. Last resource status precedes retrieval. This proves Live-active entry refusal,
not a paired FPS benchmark or direct Live refusal while mode ACTIVE.

### Next single case issued: five-minute idle exit

Same build, no flash. Keep USB and hotspot connected, dashboard idle, no media/downloads,
no panel touch, browser test switches off. Send mode off/status, clear console, then mode
on and confirm ACTIVE. Start timing from that successful on. At about two minutes send
only log mode status: expect ACTIVE and idle_ms near 120000 (status must not reset it).
At five minutes ten seconds from entry send mode status: expect OFF/reason=idle_timeout,
release_stuck=0. Do not send mode off before observing that result. Then status/log status
and one current download (CRC OK); send console/file and report any panel touch or USB/
hotspot interruption. Persisted timestamps, not manual timing alone, determine expiry.
This tests idle expiry without touch; touch reset and link/power cases remain separate.
Increment 2 acceptance pending, increment 3 unapproved.

---

## Increment 2 gate 2 - 2026-09-21, 14:32-14:34 - PASS

Pending-display admission, boot 103. JP confirms the test passed, including the screen
behavior. Evidence: attachment `b9e78bbd-c14b-42aa-801f-55e98c6f1381/Pasted text.txt`
and `C:/Users/photo/Downloads/103-current (2).log`.

Latest completes at 14:32:56.699 (31713 image bytes, 1492 ms). On at 14:33:00.243 is
OFF/refused/display_pending, before screen unload at 14:33:10.268. After navigation,
on at 14:33:27.579 succeeds ACTIVE/ok. Off reports STOPPING; persisted seq=95 confirms
exit result=ok at up_ms=496900 even though final mode status was omitted. No stuck-release
record is needed for this ordinary exit. The saved log contains the refusal and re-entry.

Current download **1155516 bytes, browser CRC OK**, 10.89 s; local file size matches,
computed CRC32 **9FA07F97**. Last pre-download full status: ready, drops=0, error=none,
queue=0/16, internal largest=31732 > 20480, writer stack minimum=2920, no USB loss or
retained failure. Same boot, Wi-Fi/MQTT connected; no unexpected reset/stall visible.
No post-download status was supplied, so those resource observations precede retrieval.
This establishes completed-image display admission, not the brief motion handover gap.

### Next single case issued: refuse entry during Live

Same build, no flash. Dashboard, Wi-Fi/MQTT connected, DTR=true/RTS=false, browser test
switches off. Send mode off/status and confirm OFF. Start Live normally. While frames
are visibly updating (about five seconds in), send log mode on; expect OFF/refused/live_busy,
and Live should continue normally. Let the full cycle finish; return to dashboard if
needed. Send mode on (ACTIVE/ok), off, then mode status (OFF). Send status/log status,
download current once with CRC OK, save console. Send console/file and whether Live
continued normally. If command arrives after Live finishes, report the timing; that does
not exercise this refusal gate. Other increment 2 gates remain pending; increment 3 is
unapproved. No firmware change or assistant build/flash in this review.

---

## Increment 2 gate 1 - 2026-09-21, 14:25-14:29 - PASS

Entry/exclusion/exit with reordered steps, boot 103. JP reports the test worked, with
Latest-in-mode tested last. Source checkpoint `5ed5ed1` includes Claude's approval of
corrections `3b9d1cc`. Evidence: attachment
`1a8641d2-e734-4b53-aab7-a12d76035ede/Pasted text.txt` and Downloads
`103-current.log` / `103-current (1).log`.

- Initially OFF; on -> ACTIVE/ok at 14:26:20; repeated on refused not_off. Off ->
  STOPPING, then OFF confirmed; repeated off reports already_off.
- Latest with mode OFF succeeded: IMAGE_BEGIN id=1, IMAGE_END ok/displayed, 31713 image
  bytes, total_ms=1365 in persisted record. UI_ACTION processed belongs to that request.
- Second on -> ACTIVE at 14:28:52; Latest at 14:28:56 printed Image refused: download
  mode. Persisted seq=63 is IMAGE_REFUSED trigger=latest reason=download_mode, with no
  IMAGE_BEGIN or UI_ACTION processed for that press. Thus "worked" means exclusion
  worked, not that a new image was admitted while ACTIVE. No firmware correction needed.
- Both exits have persisted stopping then ok records. Second exit completes at up_ms
  237138; no RETRIEVAL_STUCK appears. Record spacing is not a measured close deadline.
- USB downloads: 1145827 and **1148832 bytes**, both browser CRC OK. Second file size
  verified locally, calculated CRC32 **BA86BA29**; it includes the final refusal/exit.
  Two complete downloads plus size growth establish ongoing logging in this interval.
- Last full status (before final refusal/download) is ready, hooks=0, drops=0, error=none,
  queue=0/16, largest internal block=31732 > 20480, stack minimum=3208. No final full
  status after the second download was supplied; no stronger final-memory claim is made.
  Same boot throughout, no unexpected reset/stall visible. Normal fixture lines absent.

### Next single case issued: refuse entry during completed-image display

Same build, no flash. DTR=true/RTS=false; all browser switches off. On dashboard send
log mode off, log mode status (OFF), status. Press Latest and wait for the image to finish
loading. While that image remains displayed (within its normal one-minute display window),
send log mode on. Expect OFF/refused/display_pending, with the image still displayed.
If image_busy appears, wait for completion and retry while still on the image; if it already
returned to the dashboard, report that timing rather than treating entry as a failure.
Use the normal navigation control to return to dashboard (not the history-image Back
request). Send log mode on: expect ACTIVE/ok, then off and mode status: OFF. Capture
status/log status, download current with CRC OK, send console/file and screen observations.
This tests the pending-display admission guard and its release on navigation, not remote
motion handover or active Live. Those remain separate cases; increment 3 unapproved.

---

## Increment 1 normal-build handoff - 2026-09-21, 10:37-10:39 - PASS

JP reports the test ran fine. Evidence: attachment
`e7d8aa31-31c1-4ddf-9802-fe572e2382d7/Pasted text.txt` and
`C:/Users/photo/Downloads/101-current.log`.

- Boot 101, hooks=0; fixture, USB TEST and USB GATE status lines absent. Source flags
  confirmed: logging=1, fixture=0, hooks=0, PSRAM writer=1; working tree clean.
- **103377 bytes, browser CRC OK**, 1.04 s. Local file length matches and computed
  CRC32 is **7F7FCDC7**. Numeric CRC is local; browser CRC OK supplies device-END
  verification. Current size later reaches **104680** (+1303 beyond snapshot), same
  generation 21/newest 20, no rotation. Writes 35 -> 41.
- Final ready, active=0, paused=0, result=ok, queue=0/16, drops=0, error=none;
  no retained transfer failure, USB losses or unexpected reset in capture. Wi-Fi/MQTT
  remain connected. Internal largest block **51188** exceeds 20480; internal minimum
  92004; writer stack minimum **2920**, matching the earlier normal-build cases.

**Increment 1 is ready for JP's explicit acceptance.** Five scoped regression cases
(normal current, cancellation/reuse, queue protection, selected-archive prune, orderly
close/restart) plus this normal-build restoration check pass. Host checks and Claude
review remain as recorded; no new code changes require their repetition. In-flight
shutdown timing and delayed stale acknowledgements were not measured on hardware;
the prior documented coverage limits and review deferrals remain. No additional bench
case is requested now. No increment 2 implementation or approval is implied.

Next action: JP may accept increment 1 and explicitly approve increment 2. Proposed
increment 2 is mode state/admission with USB command entry and no server, per spec;
implementation goes to Claude review before JP builds/flashes, with bench gates issued
one at a time. Historical draft remains untouched.

---

## Increment 1 USB gate 5 - 2026-09-21, 10:30-10:33 - PASS

Orderly shutdown and restart, boots 98 -> 99, fixture-enabled build. JP reports the test
ran fine. Evidence: attachment `281557af-8003-4e4e-b338-fbd7ae3d18b0/Pasted text.txt`,
`C:/Users/photo/Downloads/98-current.log` and `99-current.log`.

- Before: 78888 bytes, browser CRC OK, local CRC32 **6AF52D0E**. After: 86435 bytes,
  browser CRC OK, local CRC32 **FF9C81B8**. The first **78888 bytes match exactly**.
- USB removal at host 10:31:06.494 is expected for this test. Persisted boot 98 records:
  POWER_DECISION action=shutdown moving=0 usb=0 idle_ms=88833 at 10:31:37.471;
  SESSION_END reason=shutdown pending=0 at 10:31:37.531. Boot 99 records reset=power_on,
  context=append. This establishes orderly close and append preservation, not an exact
  diagnosticsClose latency: clocks and record intervals are not a caller-wait measurement.
- Post-restart CRC transfer succeeds; later current_size=87730 (1295 beyond snapshot).
  Same file generation 21/newest 20, archives=7; no rotation. Final ready, active=0,
  paused=0, result=ok, queue=0/16, drops=0, error=none, no USB losses or retained failure.
  Wi-Fi/MQTT connected. Internal largest block 51188 > 20480, internal minimum 94412,
  writer stack minimum 2984. Startup sd_max_us=159198 is reported, not treated as a
  transfer stall; slow write count=0.

All five issued increment 1 bench cases now pass. This close case is idle shutdown,
not in-flight close or deep-sleep repetition; host tests cover session shutdown and
previous accepted platform evidence remains applicable. Full acceptance belongs to JP.

### Next single case issued: restore normal build and verify configuration

Codex restored only the temporary fixture define from 1 to its tracked default 0;
DIAG_ENABLED=1, DIAG_TEST_HOOKS=0 and DIAG_WRITER_STACK_PSRAM=1 remain unchanged.
JP builds/flashes amoled-1-8-core-3-3-11 (no generated-sketch deletion). Connect Chrome
with explicit DTR=true/RTS=false and all test switches off. Send status/log status,
download current once (CRC OK), wait 70 seconds, then send both status commands again.
Send saved console and current.log. Verify hooks=0, fixture/test/gate status lines absent,
ready, no drops/errors, USB idle/unpaused, CRC success and append growth. This is the
normal-configuration handoff check, not a repeat of the fixture suite. After reviewing it,
present increment 1 for explicit acceptance and ask for increment 2 approval separately
or together. No increment 2 implementation has begun.

---

## Increment 1 USB gate 4 - 2026-09-21, 09:40-09:43 - PASS

Selected synthetic archive pruning, same boot 97/fixture-enabled build. JP reports normal
operation. Evidence: attachment `3496d409-128d-4ea3-9f04-89cfa38a2bb2/Pasted text.txt`
and `C:/Users/photo/Downloads/97-current (1).log`.

- Fixture creation completed with archive=00000022, bytes=2097152, result=ok. Two
  earlier list requests returned unavailable while creation was active; this is the
  documented guard, not a logger failure. After creation, list count was 9/newest=22.
- `log test prune 22` armed and fired once; result=pruned, outcome=pruned. Device
  returned pruned. Saved current records show archive-00000022.log completion with
  bytes=1584, duration_ms=61, result=pruned, followed by USB_PRUNE_TEST synthetic=true.
- After removal: count=8, newest=20, archives=7, pruned=1 (was 0), fixture result=
  pruned_by_test. These counts return to the pre-fixture inventory; the capture does
  not include individual FILE lines for a byte-by-byte inventory comparison.
- USB active=1 in the refresh snapshots corresponds to the concurrent listing. Later
  standalone status confirms active=0, paused=0, result=pruned before the retry.
- Normal current retry: **67600 bytes, CRC OK**, 0.68 s. Local size matches, computed
  CRC32 **F33D63CB**. Browser CRC OK verifies device-END comparison; numeric CRC is local.
  Later size=68902; ready, active=0, paused=0, result=ok, zero drops/errors, same boot,
  no new stall/reset, no USB link losses. Largest block=51188, stack minimum=2984.

This passes the production removal-helper regression for an actively read disposable
archive; it does not repeat retention threshold selection. Close coverage remains pending.
No firmware edit/build/flash by Codex. JP's fixture=1 change stays local and uncommitted.

### Next single case issued: orderly shutdown close and restart

Same build, no flash. Keep hotspot available and Live stopped. Save status/log status
and a fresh CRC-checked current.log as the pre-shutdown reference. Unplug USB from the
battery-equipped unit, leave stationary and untouched, and allow up to two minutes for
automatic Shutdown/screen-off. Do not force it with the power button; report a failure
to shut down. After power-off, reconnect USB and Chrome with explicit DTR=true/RTS=false.
Retain both console segments. Send status/log status, download current (CRC OK), wait
70 seconds, then send both status commands again. Send both saved files and the console,
plus observed shutdown behavior. Compare prefix preservation, prior SESSION_END
reason=shutdown pending=0, new boot append, later growth and zero drops/errors.
This is orderly idle shutdown/restart, not a measurement of an in-flight network or USB
cancellation deadline. Existing host tests separately cover shutdown during a session.
After reviewing close evidence, restore the normal fixture=0 build before final handoff;
no increment 2 work without explicit approval.

---

## Increment 1 USB gate 3 - 2026-09-21, 09:37-09:39 - PASS

Queue-pressure abort and retry, boot 97, fixture=1/hooks=0/PSRAM writer=1.
JP reports the test ran fine. Evidence: attachment
`7450c206-1d9e-47d3-b307-6c15820bc770/Pasted text.txt` and
`C:/Users/photo/Downloads/97-current.log`.

- `log test queue` armed; transfer fired once, added=8, queued_at_test=8/16,
  result=injected. Device returned logger_busy, browser reported the expected error.
- Saved retry contains all **eight USB_QUEUE_TEST records**, boot 97 seq=38..45,
  followed by seq=46 `USB_GET_END bytes=1584 duration_ms=54 result=logger_busy`.
  Thus the actual queued evidence was saved after abort, not merely reported injected.
- Retry without rearming: **61227 bytes, CRC OK**, 0.61 s. Local file size matches;
  independently calculated CRC32 **23E3FFE5**. Numeric CRC is local; browser CRC OK
  establishes the device-END comparison.
- Final gate retains fired=1, added=8, queued_at_test=8/16, result=injected,
  outcome=logger_busy; armed/active=none. USB active=0, paused=0, result=ok, queue=0/16.
  Logger ready, drops=0, error=none, no new stall/reset in capture. Queue high=9 is
  compatible with the completion record after the half-full trigger, not an overflow.
- current_size grows beyond the snapshot to 62520; same generation 21/newest 20,
  rotations=0. Largest internal block 51188 exceeds 20480; internal minimum 91928;
  stack minimum 3176 -> 2984 in this fixture build. No USB link losses.
- JP omitted the intermediate status between abort and retry. The persisted abort
  record, all eight records, successful retry and retained gate outcome substantiate
  cleanup/reuse; no intermediate status observation is claimed.

No firmware changes by Codex. JP's local DIAG_USB_TEST_FIXTURE=1 edit remains uncommitted
and untouched for the next case; tracked default stays zero. Prune and close regression
gates remain pending, and increment 2 remains unapproved.

### Next single case issued: prune an actively read disposable archive

No rebuild/flash; same fixture-enabled boot/build. Explicit DTR=true/RTS=false,
all browser test switches off; no Live. Capture status/log status. Send `log test file`,
wait for result=ok and note its new archive number N. Refresh files and preserve the
list. Send `log test prune N` using only that newly created synthetic archive number,
then download that archive. Expect Device: pruned and no saved partial file.
Refresh files and capture status/log status: only the fixture disappears, pruned rises
by one, result=pruned and outcome=pruned, logger ready and drops=0. Stop on mismatch.
Download current.log normally (CRC OK), then status/log status. Send full saved console
and that current.log. This exercises the production reader-close-before-unlink path;
it does not retest retention threshold selection. No separate fixture deletion is needed.

---

## Increment 1 USB gate 2 - 2026-09-21, 09:29-09:32 - PASS

Cancellation and reuse, same boot 95 and normal build. JP reports the test ran fine.
Evidence: attachment `cf8ccfce-6e10-4863-a073-1abd773a8e1b/Pasted text.txt` and
`C:/Users/photo/Downloads/95-current (1).log`.

- Deliberately damaged browser data caused `log abort` at 09:30:11.269; device replied
  `@@ERR reason=aborted` at .281 and browser reported abort confirmed. The 12 ms is
  host-observed command/response timing, not an isolated writer close measurement.
- Following status: active=0, paused=0, bytes=1296, result=aborted, queue=0/16.
  Retry snapshot contains `USB_GET_END bytes=1296 duration_ms=56 result=aborted`, seq=64.
- Normal retry: **41702 bytes, CRC OK**, 0.43 s. Local file has exactly 41702 bytes;
  independently computed CRC32 **E936A9DC**. Browser CRC OK establishes device-END
  comparison; the numeric CRC is calculated locally, not exposed in the capture.
- Post-retry current_size=41862 at 09:30:56, then 43139 at 09:32:07/14: **1277 bytes
  growth**. Same generation 21/newest archive 20, no rotation. Final active=0, paused=0,
  result=ok, ready, drops=0, error=none; no unexpected reset/stall in the capture.
- Internal largest block remains 51188 bytes, internal minimum 91952, writer stack
  minimum 2920. USB losses=0 for both new transfers; Wi-Fi/MQTT remain connected.

This proves cancellation cleanup and subsequent reservation reuse on hardware, not delayed
stale acknowledgements (those remain host-test coverage). Queue, prune and close gates
remain pending. Increment 2 is unapproved.

### Next single case issued: queue-pressure abort and normal retry

Use the existing fixture build: JP sets `DIAG_USB_TEST_FIXTURE=1` in
`src/diagnostics/diagnostics_config.h`, keeping DIAG_TEST_HOOKS=0, DIAG_ENABLED=1 and
DIAG_WRITER_STACK_PSRAM=1, then builds/flashes amoled-1-8-core-3-3-11. No generated-sketch
deletion is needed. This is existing bench instrumentation, not new firmware logic.
The checked-in default remains zero; restore it after fixture regression work.

Chrome: explicit DTR=true/RTS=false, all browser test switches off, no Live, Wi-Fi/MQTT
connected. Capture status and log status, send `log test queue` (expect armed=queue),
download current once (expect logger_busy and no partial save), then status/log status.
Require fired=1, result=injected, outcome=logger_busy, queued_at_test=8/16, added>0,
zero drops, and USB idle/unpaused. Stop on any mismatch. Download current again without
rearming (CRC OK), then status/log status. Send full saved console and successful file
so injected records and the terminal event can be checked. No later case issued yet.

---

## Increment 1 USB gate 1 - 2026-09-21, 09:23-09:25 - PASS

JP reports the test ran fine and the file downloaded. Checkout at review: `41dd46d`,
firmware implementation `7976818`, reviewed by Claude at `0ba72e5`.
Evidence: console attachment `bd3d1894-af58-4bd2-8264-ae1f7f796181/Pasted text.txt`
and local download `C:/Users/photo/Downloads/95-current.log` (09:23:31).

- Browser reports **32598 bytes, CRC OK**, 0.37 s, decoded 87865 B/s. Independent local
  file inspection confirms 32598 bytes and computes CRC32 **59978670**. The capture hides
  raw BEGIN/D/END frames, so that numeric CRC is a local calculation; browser CRC OK is
  the evidence for its device-END comparison. The snapshot ends with `USB_GET_BEGIN`,
  as expected; its own completion record cannot be in the frozen snapshot.
- Same **boot 95**, current generation 21, newest archive 20 throughout; rotations=0.
  Before transfer current_size=32438. At 09:23:45/50, post-transfer size=33890;
  at 09:24:52 size=34898; at 09:24:57 size=35032. Later growth **1142 bytes** confirms
  append continuation with no rotation ambiguity. The first post-transfer status was
  about 15 seconds after completion; the later samples still establish the gate.
- Post-transfer and final USB state: active=0, paused=0, bytes=32598, result=ok;
  queue=0/16, drops=0, failure valid=0. Logger ready, error=none, errno=0; writes 35 -> 45.
  No reset or new stall is visible in this capture. Wi-Fi and MQTT remain connected.
- One transient USB connection observation: losses=1, max_loss_ms=3, pending=0;
  recovered within the unchanged 1000 ms grace, with successful CRC completion.
- Internal largest block **51188 bytes**, above the **20480-byte** gate; internal minimum
  91952 bytes. Writer PSRAM stack minimum moved 3208 -> **2920 bytes** and then remained
  there in the later sample. This is one transfer, not a repeated-resource stability test.

The issued case requested DTR=true/RTS=false; opening is outside this cleared capture,
so those settings are procedural, not independently visible here. No build transcript
was supplied; running behavior is hardware evidence, not a compilation-log review.
No firmware changes, assistant builds or flashes in this result review. Queue-pressure,
prune-conflict, cancellation and close regression gates remain pending, one case at a
time. Increment 2 remains unapproved.

---

## Case 1 result - 2026-09-20, 18:21-18:55 - PASS, timing deferred

**Revised September 20 after Codex's evidence review (`6c3148e`).** The first write-up
stated several inferences as observations. Findings are now split into what was observed
and what was concluded from it. Corrections are marked; nothing in the pass verdict
changed, but four conclusions were withdrawn or narrowed.

**Rig.** Windows PC on the `iphone-jp` hotspot at `172.20.10.13/28`, phone at
`172.20.10.1`, Ethernet left connected throughout. Python 3.13.9
`python -m http.server 8000 --bind 172.20.10.13`, serving a scratch directory, never the
repository. The companion was powered and connected to the same hotspot for the whole
session. iPhone 13 Pro running **iOS 26.6.2**.

**Evidence available.** Server access log, `netstat` socket snapshots, a continuous ping
log, and JP's reports of what the phone displayed. **No request-header capture, no packet
trace and no Files screenshots.** Request headers were never recorded, which bounds several
findings below. Nothing here is packet-level proof.

**Verdict.** The listing loaded in Safari and files saved to the Files app at sizes
consistent with what was served. The case passes on its stated criterion.

### Observed

| # | Observation | Evidence |
|---|-------------|----------|
| O1 | Safari on the **hotspot-host phone** reached a **Wi-Fi client** of that hotspot and loaded the listing. | `172.20.10.1 - - [18:24:05] "GET / HTTP/1.1" 200` |
| O2 | One `GET /favicon.ico` accompanied the first listing load; the later listing load at 18:27:01 was not followed by another. | `"GET /favicon.ico HTTP/1.1" 404` at 18:24:05 |
| O3 | The same 262,144 bytes served as `application/octet-stream` produced a save prompt; served as `text/plain` it rendered inline as text. | `sample.log` versus `sample.txt`, identical bytes, differing `Content-Type` |
| O4 | Two distinct phone source ports appeared in `TIME_WAIT` after a download, on two occasions. | `:51950`/`:51952`, later `:51955`/`:51956` |
| O5 | Every logged request line used `GET`. No `HEAD` appeared. | Server access log, all entries |
| O6 | Files displayed `sample.bin` as **262 KB** and `big.bin` as **2.1 MB**, consistent with 262,144 and 2,097,152 bytes served. | JP's reports from the Files app |
| O7 | One interrupted transfer displayed on the phone as **"713 KB of 2.1 MB"**. | `big.log`, interrupted by a link loss |
| O8 | A 2 MiB body was served without server error, and a local fetch of the same file returned all 2,097,152 bytes. | Access log; local `Invoke-WebRequest` |

### Concluded, with its strength

- **C1 (from O1), strong.** The network path the feature depends on exists: the phone can
  reach an HTTP server running on a device attached to its own hotspot. **Not tested:**
  traffic between two tethered clients, and - the case that actually ships - reaching the
  **ESP32's** server rather than a laptop's.
- **C2 (from O2), moderate.** Safari requests a favicon. **Withdrawn: "once per session".**
  One non-repeat does not establish a caching rule. The firmware requirement does not
  depend on the frequency: **routing must acquire the reader reservation only for requests
  that need it.** A favicon may consume a socket; it must not take the session, touch SD or
  overwrite the last-transfer result. Under JP's rule it still counts as an HTTP request
  for the five-minute idle reset.
- **C3 (from O3), strong for MIME, untested for disposition.** Serving a log as
  `text/plain` puts it on screen instead of into Files, so the firmware sends
  `application/octet-stream`. **`Content-Disposition` was never exercised** - the Python
  server does not send it - so the attachment header is a **design decision**, taken
  because the filename carries the expected size and transfer ID, not a tested result.
  This evidence does **not** show that `text/plain` plus an attachment header would fail
  to download.
- **C4 (from O4), withdrawn as stated.** `TIME_WAIT` is a post-close state; two ports prove
  two **recently closed** connections, not two concurrent sockets, nor a purpose, nor "two
  per download". Two sequential requests preceded each snapshot and fully account for it.
  **The earlier claim that `max_open_sockets = 2` "would have been wrong" is withdrawn** -
  that setting counts clients and permits two, and nothing observed disproves it. Keep the
  2-3 proposal as a prudent start, measure live connection occupancy on firmware, and never
  tie a retrieval session to a TCP accept.
- **C5 (from O5), split.** *No `HEAD`* is supported for the observed request lines.
  ***No `Range`* is not established and the claim is withdrawn.** Verified independently:
  Python 3.13.9's `http/server.py` contains no `Range`, `Accept-Ranges` or `If-Range`
  handling at all, and `log_request` is called by `send_response()` and records only the
  request line and status. A `Range` request would therefore have been ignored, answered
  `200` with the full body, and left no trace. The honest statement: **observed downloads
  completed against a server with no resume support, so version 1 keeps full-body `200`
  behaviour** - not that the client never asks. Capture real `Range`/`If-Range` headers in
  firmware evidence.
- **C6 (from O6, O8), narrowed.** A 2 MiB transfer is feasible over this path. The phone
  sizes are **rounded Files displays, not byte-exact verification**; the local fetch
  validates the laptop's path, not the saved iPhone copy. Byte-exact confirmation still
  requires the exported-Safari-file gate. *Unit correction:* 262,144 bytes is **256 KiB**
  and 524,288 is **512 KiB**; the first write-up said 262 KiB and 524 KiB.
- **C7 (from O7), narrowed.** One interruption was visibly partial against `Content-Length`.
  This does **not** establish that every truncation is detected, nor what object Files
  finally retained. The deliberate truncated-response gate and the Safari-export comparison
  remain necessary.

Also noted: a logged `200` is written when the response starts, so **server status is not
evidence of a completed body**. Exact saved size and content are the stronger evidence.

### What this requires of the firmware

- **`application/octet-stream` with `Content-Disposition: attachment`** (C3).
- **SD-free, reservation-free handling of incidental requests** such as favicon (C2), with
  bounded servicing of a second request while a transfer streams - `max_open_sockets` alone
  does not make handlers concurrent.
- **Spare socket capacity above a single client** as a starting configuration, to be
  measured rather than assumed (C4).
- **Full-body `200` for version 1**, with an explicit, cheap and deliberate policy for
  `HEAD` and for a `Range` request that a future iOS may send (C5).
- `Content-Length` on every response, which is what made O7 visible.

### Two procedural findings

- **A Files "error" is not necessarily a failed download.** `big.bin` showed
  "Your device couldn't connect to the server" yet appeared at full size; Files was failing
  to *preview* an unrecognised binary, which it labels a "MacBinary archive". JP identified
  this. An earlier reading of it as a transfer failure was wrong and briefly motivated a
  `Range` hypothesis that C5 now shows the evidence cannot settle either way.
  **Confirm by size, not by whether the file opens** - and prefer an exact size to a
  rounded display.
- **The hotspot was absent from a PC scan while the companion stayed associated.** Treat
  this as a rig observation on this phone and date, not a universal rule about iOS.

### Timing: deliberately deferred

The 2 MiB duration was not obtained. Three attempts were interrupted by the PC losing the
hotspot (`ConnectionAbortedError 10053`, 12 ping timeouts, `netsh wlan` reporting
`disconnected`), and finally the PC could see the SSID but could not associate.

**Correction to the first write-up, which called this "the test rig".** The cause was not
established. The companion staying associated shows only that there was no total outage
affecting all clients; it does not identify what dropped the PC, and the SSID being absent
from a scan is hotspot-side behaviour. What can be said is narrower: **none of it involves
firmware, which does not exist yet**, and it is not a measurement of the feature.

Deferral stands. The number governing the 120 second `current.log` limit and the 5 second
no-progress abort is **ESP32-to-phone** behaviour, and average throughput alone would not
set either bound - the overall limit follows throughput, while stall safety depends on the
longest no-progress interval, queue occupancy and cancellation latency. The ESP32 is the
actual server but is **not yet proven to be the sole bottleneck**. For reference only, a
local fetch of the same 2 MiB file ran at roughly 1.2 MB/s. The firmware timing gates are
specified in [the implementation spec](sd_iphone_log_download_spec.md).

### Limits of this result

A laptop's Python server is not `esp_http_server`: response construction, timeouts, socket
limits and concurrency are all untested here. Findings are Safari's behaviour on one phone
running iOS 26.6.2, on one date, over plain HTTP with no TLS, with a single client, and
**without request-header capture**. A future iOS can change C2 through C5. The firmware
must pass its own reachability, response-contract and transfer cases.

---

## Case 1 - iPhone reachability proof (no firmware)

**Why this runs first.** Every later case assumes Safari on the iPhone can open a page
served by a device sitting on that same iPhone's Personal Hotspot. Nothing in the project
proves that yet: the companion talks *out* to MQTT and HTTPS, which is the opposite
direction. If this fails, the whole feature needs a different transport, and finding that
out now costs an evening instead of a firmware cycle.

It also answers two design questions cheaply, before any code is written:

- **Content type.** Does Safari render a `.log` inline instead of offering a download? That
  decides whether `Content-Disposition: attachment` and
  `Content-Type: application/octet-stream` are mandatory or merely tidy.
- **Link speed.** How long does 2 MiB actually take over the hotspot? That is the number
  behind the 120 second `current.log` limit and the 5 second stall bound.

**Nothing on the companion is involved.** Do not flash anything. Leave the companion
powered and connected to the hotspot as usual, so the subnet carries its normal client.

### Before you start

Record the iPhone's iOS version: **Settings > General > About > Software Version**. It goes
in the result and is needed context for every Safari behaviour observed later.

### Steps

1. **Turn on Personal Hotspot** on the iPhone 13 Pro. Connect the Windows laptop to it over
   Wi-Fi (not USB tethering - the companion uses Wi-Fi and that is the path under test).

   **Foreground Settings > Personal Hotspot only to establish or re-establish the
   association**, then switch to Safari to run the case. The two cannot be open at once on
   the same phone, so do not treat "keep that screen open" as a standing instruction. If the
   laptop later cannot rejoin, return to that screen, reconnect, and switch back.

2. **Create a scratch folder with three test files.** Use the session scratch directory, not
   the repository - a default directory server exposes everything below it.

   ```powershell
   $d = "$env:TEMP\hotspot_test"
   New-Item -ItemType Directory -Force $d | Out-Null
   # 256 KiB text file, roughly a real current.log
   $line = ("A" * 127) + "`n"
   Set-Content -Path "$d\sample.log" -Value ($line * 2048) -NoNewline -Encoding ascii
   # 256 KiB binary file
   [byte[]]$b = 1..262144 | ForEach-Object { $_ % 256 }
   [System.IO.File]::WriteAllBytes("$d\sample.bin", $b)
   # 2 MiB binary file, the archive size
   [System.IO.File]::WriteAllBytes("$d\big.bin", (New-Object byte[] 2097152))
   Get-ChildItem $d | Select-Object Name, Length
   ```

   Note the three sizes it prints. Those are the numbers you compare against on the phone.

3. **Find the laptop's hotspot address.** `ipconfig` - look for the wireless adapter
   connected to the iPhone. The address is normally `172.20.10.x`, with the phone itself at
   `172.20.10.1`. Confirm the link with `ping 172.20.10.1`.

4. **Serve the folder, bound to that address only:**

   ```powershell
   cd $env:TEMP\hotspot_test
   python -m http.server 8000 --bind 172.20.10.X
   ```

   Use your actual address. Binding it keeps the server off every other interface.

5. **Windows Firewall will prompt.** Allow the profile Windows actually assigned to the
   hotspot adapter - check it first with `Get-NetConnectionProfile`, since a hotspot is
   commonly classified **Public** and a Private-only rule would then never apply. Keep the
   rule narrow (one port, the hotspot address, the hotspot subnet). Do not disable the
   firewall. Remove the rule when the case is finished - step 9.

6. **On the iPhone, open Safari** and go to `http://172.20.10.X:8000/`. The directory
   listing should appear.

7. **Tap each file in turn and record what happens.** This is the actual data of the case:

   - `sample.log` - does Safari **display it inline** as text, or offer a download? This is
     the content-type question.
   - `sample.bin` - expect a download prompt. Save it to Files.
   - `big.bin` - start it and **time it**. While it runs, **lock the phone** for about ten
     seconds, unlock, and see whether the download resumed, continued or failed. Then repeat
     the download and **background Safari** (swipe to home) for about ten seconds instead.

8. **Check the saved sizes** in the Files app (long-press a file > Info, or the list view's
   size column) against the three numbers from step 2.

9. **Stop the server** with Ctrl+C and remove the firewall rule:
   `Windows Security > Firewall & network protection > Allow an app through firewall`, find
   the Python entry created in step 5 and remove it.

### What to send back

- iOS version.
- The laptop's hotspot IP, and whether `ping 172.20.10.1` worked.
- Per file: inline or download, and the size shown in Files against the expected size.
- `big.bin` duration in seconds, plus what happened on screen lock and on backgrounding.
- Anything unexpected: prompts, warnings, stalls, or a listing that loaded slowly.

### Pass criteria

The case passes if the directory listing loads in Safari and `sample.bin` saves to Files at
exactly the expected size. Everything else on the list is observation that shapes the
design rather than a pass condition.

### If the page does not load

Check the firewall rule and the `--bind` address **before** concluding that the hotspot
isolates its clients. A `ping 172.20.10.1` that works while the browser fails points at the
laptop, not the network. Only after both are ruled out is client isolation the explanation -
and that would be a genuine finding worth stopping on, because it would rule out the whole
local-HTTP approach.

### Limits of this case

A laptop's Python server is not the companion's `esp_http_server`, and passing here does not
certify the firmware's own responses, timeouts or socket limits. It establishes the network
path and Safari's handling of the two content types on this specific iOS version, on this
date. The firmware still has to pass its own reachability case later.
