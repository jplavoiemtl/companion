# Evening SD evidence, 2026-09-16

Source: JP supplied F:\2026-09-16-evening\logs after boot-30 shutdown at 18:02:11 Montreal time.
The .log.txt files preserve the original .log bytes, with hashes verified after copying.
Original files were not modified. Archives 3 through 8 match the earlier 15:11 backup exactly.

| Original file | Bytes | SHA-256 |
|---|---:|---|
| archive-00000003.log | 584 | 1946f745f45d6a0c8c5e1e34dd89f99df0093a584e55d1aa6a0353cdad215a7b |
| archive-00000004.log | 584 | 3cad99137c62aa7f5e8f778558626bc3ee0f889da15f47f990c5398d79fba09c |
| archive-00000005.log | 584 | c8fe643bcf249213b932b17f4d449cb12e57e6c2d995004ae85346d2cae8916a |
| archive-00000006.log | 6474 | 1aed5c20877d148e6516b4cbf25a23d4ddc4dd5806b5304c0e649501d8759da3 |
| archive-00000007.log | 3627 | f507b7baeb1253c5c77aa00689d94f5dcf7cc95a3cb0ee3ab8dd8050a680c1d6 |
| archive-00000008.log | 153 | 7696b278937443eb2fd2c5282ec2848aa1218372f5d4b7291f1884b97377e1d0 |
| archive-00000009.log | 5447 | 43e7dcdd8e1da61ef14892cb3917ae7bed2ba60c26e9f0d3b911f6fb07a50c15 |
| current.log | 92055 | 48bf15790e1c50656fac1985763a67283b60f005d99e848665a4f0679720fa6c |

All retained sequence ranges are contiguous within each boot, including across files:
- Boot 24: 19 through 35.
- Boot 25: 1 through 12.
- Boot 26: 1 through 14.
- Boot 27: 1 through 13.
- Boot 28: 1 through 9.
- Boot 29: 1 through 148.
- Boot 30: 1 through 35.

Archive 8 contains the deliberate 22-byte malformed prefix followed by boot-26 shutdown.
The first record in current.log is generation 10, reason=new, NOT reason=empty_recovery.
The header-pause/reset test therefore demonstrates missing-file recovery; true empty-file
recovery remains unverified. Both deliberate reset classifications and retained breadcrumbs,
both DST offset transitions, SNTP restoration and final shutdown are present.
