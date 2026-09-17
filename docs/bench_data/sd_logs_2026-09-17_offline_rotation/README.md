# Final offline rotation and preservation inspection — 2026-09-17

Read-only inspection of E:\sdcard after JP's boot-38 normal shutdown.
All ten files were backed up byte-for-byte; manifest.json records sizes and hashes.
The card was not modified.

| Managed file | Bytes | Boot-38 sequences |
|---|---:|---|
| archive-00000014.log | 8059 | 1–20 |
| archive-00000015.log | 7936 | 21–36 |
| archive-00000016.log | 7942 | 37–52 |
| current.log | 3325 | 53–61 |

All 61 records have time=unknown; sequences are complete with no duplicates.
FILE_OPEN for generation 14 has reason=empty_recovery.
Generations 15, 16 and 17 have reason=size at uptime 842440, 1682838
and 2523278 ms. All completed archives are below the 8192-byte test cap.
Archives 12 and 13 were pruned as expected and have prior project backups.
Final record: SESSION_END reason=shutdown pending=0, up_ms=2774316.

All six sentinel paths, sizes and SHA-256 hashes match the
../sd_logs_2026-09-17_0931/fixture.json manifest. This includes images-folder data,
unrelated text, the seven-digit name, suffix and case variants, and the child
of the archive-like directory. That directory still exists.

Offline size rotation, pruning, protected-file preservation and final close pass.
Stage 1 acceptance remains JP's decision; this inspection does not begin Stage 1B.
