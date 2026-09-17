# Rotation evidence and incomplete-tail fixture, 2026-09-17

Source: JP-confirmed actual SD card at E:\sdcard\logs.
The .log.txt backups preserve the original bytes before any fixture change.

| Original | Bytes | Boot-34 sequence range | SHA-256 |
|---|---:|---|---|
| archive-00000012.log | 8136 | 1-21 | 64188f3519d5659c26e2925837892a689b9bae805d6f8d9da35995a414388369 |
| current.log | 4087 | 22-31 | 819d2f01747f520ecc917ae6cd339a2975708785175e6a7ab451d4847dabf078 |

Verified: archive 12 is 8136 bytes (limit 8192), current has FILE_OPEN generation=13
reason=size, and all sequences 1-31 are contiguous across files. Final shutdown
is 2026-09-17 08:52:17.219 -04:00 with pending=0. Natural rotation content check passes.

After backup, appended only the literal fixture fragment shown in fixture.json, with
no newline, to the card current.log. Flushed and closed it. Header and original prefix
remain unchanged; archive 12 remains byte-identical. This is a controlled tail fixture,
not evidence of real power-loss durability. Expected next boot: append a newline and
TAIL_RECOVERY action=added_newline, retain generation 13 and archive 12.
