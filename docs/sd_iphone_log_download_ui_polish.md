# Retrieval UI polish before car deployment

JP requested these presentation changes after both extra pre-car gates passed.
He intends to use his existing car configuration, already exercised on the bench.
No configuration changes are made by this patch.

## Changes

- Shared HTTP listing and last-result HTML: explicit dark color scheme, #121212
  background, #ededed text, #8ab4f8 links and #c6a7f2 visited links. No external assets.
- Companion download screen: title, status, iPhone instruction and URL use the already
  enabled Montserrat 24 font, up from 20. Text is "On your iPhone".
- Remove the Latest/Live and five-minute explanatory labels. Their actual behavior
  stays unchanged. Top 64 pixels remain reserved; positions are 66, 105, 190 and 230.
  Status has space to wrap above the instruction; URL can wrap above the bottom button.
- Button dimensions, position and text styling remain unchanged. Larger fonts are
  applied to individual labels, not inherited from the screen by the button.
- No generated UI files or companion.ino changes. No generated-sketch deletion required.

## Verification and handoff

All 288 existing checks across 13 host suites pass. Only the UI test harness font
symbol table needed the available 24 px font; no assertions were removed or weakened.
git diff --check passes. No firmware build or flash performed. Host checks do not
establish visual appearance on hardware. No new low-impact appearance tests added.

Claude review requested before JP builds: inspect the two source-file changes and
harness symbol addition, particularly label spacing for wrapped status/URL, preserved
button styling, and bounded shared HTML formatting. After clearance, JP rebuilds the
same selected profile and performs one visual case: load companion download mode,
view Safari listing and last-result, then Stop. Capture both screens to assess text
fit and colors. Existing transfer-integrity and storage gates need not be repeated
for this styling-only patch absent a regression.
