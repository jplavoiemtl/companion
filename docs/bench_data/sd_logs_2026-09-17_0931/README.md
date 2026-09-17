# SD inspection after tail repair and touch wake

Read from E:\sdcard after JP's normal shutdown on 2026-09-17.
Raw .txt backups preserve the card bytes; manifest.json records SHA-256.

- Archive 12 is identical to the 08:52 backup.
- The original 4087-byte current prefix is unchanged. The 4143-byte prepared
  tail fixture also matches its recorded hash; firmware added a newline,
  then TAIL_RECOVERY action=added_newline in boot 35.
- Across both files, sequences are complete: boot 34 1–31, boot 35 1–27,
  boot 36 1–9, boot 37 1–13. The single invalid line is the deliberate fixture.
- Boot 36 records SESSION_END reason=deep_sleep pending=0.
- Boot 37 records reset=deep_sleep, reset_code=8, wake_code=2.
  JP confirmed touch alone woke the board before USB.
- Both breadcrumbs survive from boot 36: main sleep and writer sd_close.
- Boot 37 starts time=approx; SNTP restores synced with correction_ms=-118.
- Final boot-37 normal shutdown is 09:31:10.475-04:00, pending=0.

The short-tail recovery and deep-sleep evidence checks pass.
Power-loss durability and near-cap tail recovery remain outside these tests.
Fixture preparation after this backup is recorded separately in fixture.json.
