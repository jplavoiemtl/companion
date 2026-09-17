# SD evidence after small-limit recovery, 2026-09-17

Source: actual SD card at E:\sdcard\logs, confirmed by JP.
Only current.log was present. Backup preserves its exact bytes, including line endings.
Size: 472389 bytes. SHA-256: 0eb39ffc9594cdfcea686ae61ce79a9ffcdf7e2fcaf2577f3d4ab5f01a2b5efe.

- Generation 11 FILE_OPEN reason=empty_recovery is verified.
- Boot 32 sequences 1-836 and boot 33 sequences 1-10 are contiguous.
- Boot 33 appended after logger failure; CLOCK_SYNC at uptime 19372 ms.
- Final SESSION_END: 2026-09-17 08:28:58.218 -04:00, shutdown, pending=0.

After backup verification, original current.log was preserved as archive-00000011.log.
A fresh zero-byte current.log was exclusively created, flushed and closed for the
size-rotation retry. Expected next generation: 12. Safely eject before board insertion.
