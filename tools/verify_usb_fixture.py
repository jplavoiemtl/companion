"""Verify a downloaded synthetic USB archive without removing the SD card.

Usage: python tools/verify_usb_fixture.py [downloaded-file.log]
No argument prints the expected size, CRC32 and SHA256.
"""
import argparse
import hashlib
from pathlib import Path
import sys
import zlib

SIZE = 2 * 1024 * 1024


def reference():
    return b"".join(
        f"USB_TEST_FIXTURE line={number:08d} ".encode("ascii").ljust(63, b".") + b"\n"
        for number in range(SIZE // 64)
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("file", nargs="?", type=Path)
    args = parser.parse_args()
    expected = reference()
    print(f"Expected bytes={len(expected)} CRC32={zlib.crc32(expected):08X} "
          f"SHA256={hashlib.sha256(expected).hexdigest()}")
    if args.file is not None:
        try:
            actual = args.file.read_bytes()
        except OSError as error:
            print(f"FAIL: {error}", file=sys.stderr)
            return 1
        if actual != expected:
            print(f"FAIL: fixture differs; received {len(actual)} bytes", file=sys.stderr)
            return 1
        print("PASS: downloaded fixture matches every expected byte")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
