# SD storage-test evidence, 2026-09-16

Source: JP supplied E:\sdcard\logs after boot 27 shutdown at 15:11:53 Montreal time.
Each .log.txt file is byte-identical to its original .log; only the backup filename differs.
Original files were read without modification. SHA-256 hashes verified after copying.

| Original file | Bytes | SHA-256 |
|---|---:|---|
| archive-00000003.log | 584 | 1946f745f45d6a0c8c5e1e34dd89f99df0093a584e55d1aa6a0353cdad215a7b |
| archive-00000004.log | 584 | 3cad99137c62aa7f5e8f778558626bc3ee0f889da15f47f990c5398d79fba09c |
| archive-00000005.log | 584 | c8fe643bcf249213b932b17f4d449cb12e57e6c2d995004ae85346d2cae8916a |
| archive-00000006.log | 6474 | 1aed5c20877d148e6516b4cbf25a23d4ddc4dd5806b5304c0e649501d8759da3 |
| archive-00000007.log | 3627 | f507b7baeb1253c5c77aa00689d94f5dcf7cc95a3cb0ee3ab8dd8050a680c1d6 |
| archive-00000008.log | 153 | 7696b278937443eb2fd2c5282ec2848aa1218372f5d4b7291f1884b97377e1d0 |
| current.log | 3650 | 452b85af2edbc65bdd45b10d49a8ad84127a492d80bafe21e5b49fdc606a3307 |

Archive 8 intentionally contains the 22-byte test prefix immediately followed by a shutdown record.
It is preserved unchanged, not repaired. Other files have valid common-field record prefixes.
Retained sequence ranges: boot 24: 19-35; boot 25: 1-12; boot 26: 1-14;
boot 27: 1-13. Boot 26 sequence 14 is embedded after the malformed prefix.
The missing earlier boot-24 records belonged to pruned archives.
