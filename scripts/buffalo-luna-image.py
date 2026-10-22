#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
#
# Build a firmware image for Buffalo routers on the Realtek "luna" SDK
# (RTL960x family, e.g. the WSR-1500AX2S) that the stock firmware's web
# updater accepts.
#
# The file is a 52-byte header followed by a plain tar archive. The stock
# web server checks the header, strips it and hands the archive to
# /etc/scripts/fwu_starter.sh, which verifies the MD5 of the fwu.sh found
# in the archive against md5.txt and then runs that script as root. The
# script in the archive does the actual installation.
#
# The header is a little-endian structure, stored byte-reversed and XORed
# with 0xff (so the structure's first field ends up last in the file):
#
#   u32 hdrcrc        bitwise NOT of the CRC32 of the encoded header
#                     without these 4 bytes (big-endian)
#   char "FIRMWARE"
#   char platform[4]  "520", NUL terminated
#   char version[4]   e.g. "113" for Ver.1.13, NUL terminated
#   char product[24]  e.g. "WRTT-363AX" (the WSR-1500AX2S), NUL padded
#   u32 length        of the archive
#   u32 crc32         of the archive

import argparse
import hashlib
import io
import os
import struct
import tarfile
import zlib

def encode(data):
    return bytes(b ^ 0xff for b in data[::-1])


def add_file(tar, name, data, mode, mtime):
    info = tarfile.TarInfo(name)
    info.size = len(data)
    info.mode = mode
    info.mtime = mtime
    info.uname = "root"
    info.gname = "root"
    tar.addfile(info, io.BytesIO(data))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--kernel", required=True, help="uImage")
    parser.add_argument("--rootfs", required=True, help="root filesystem")
    parser.add_argument("--script", required=True, help="fwu.sh to run")
    parser.add_argument("--product", required=True, help='e.g. "WRTT-363AX"')
    parser.add_argument("--version", required=True, help='3 digits, e.g. "113"')
    parser.add_argument("--platform", default="520")
    parser.add_argument("--fwu-ver", default="V4.0.0-000000",
                        help="content of the fwu_ver file")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    mtime = int(os.environ.get("SOURCE_DATE_EPOCH", "0"))

    kernel = open(args.kernel, "rb").read()
    rootfs = open(args.rootfs, "rb").read()
    # The stock updater writes the root filesystem through the MTD device
    # of its UBI volume, which needs whole flash pages.
    rootfs += b"\xff" * (-len(rootfs) % 4096)
    files = [
        ("fwu.sh", open(args.script, "rb").read(), 0o755),
        ("uImage", kernel, 0o644),
        ("rootfs", rootfs, 0o644),
        ("fwu_ver", (args.fwu_ver + "\n").encode(), 0o644),
        ("hw_ver", b"skip\n", 0o644),
        ("k_crc32", b"%08x\n" % (zlib.crc32(kernel) & 0xffffffff), 0o644),
        ("r_crc32", b"%08x\n" % (zlib.crc32(rootfs) & 0xffffffff), 0o644),
    ]
    md5 = "".join("%s  %s\n" % (hashlib.md5(data).hexdigest(), name)
                  for name, data, _ in files)
    files.append(("md5.txt", md5.encode(), 0o644))

    archive = io.BytesIO()
    with tarfile.open(fileobj=archive, mode="w", format=tarfile.GNU_FORMAT) as tar:
        for name, data, mode in files:
            add_file(tar, name, data, mode, mtime)
    payload = archive.getvalue()

    header = b"FIRMWARE"
    header += args.platform.encode().ljust(4, b"\0")
    header += args.version.encode().ljust(4, b"\0")
    header += args.product.encode().ljust(24, b"\0")
    header += struct.pack("<II", len(payload), zlib.crc32(payload) & 0xffffffff)
    header = encode(header)
    header += encode(struct.pack(">I", ~zlib.crc32(header) & 0xffffffff))
    assert len(header) == 52

    with open(args.output, "wb") as f:
        f.write(header)
        f.write(payload)


if __name__ == "__main__":
    main()
