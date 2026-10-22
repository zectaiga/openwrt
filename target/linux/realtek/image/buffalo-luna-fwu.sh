#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-only
#
# OpenWrt installer for Buffalo routers on the Realtek "luna" SDK, run as
# root by the stock firmware's updater (/etc/scripts/fwu_starter.sh) with
# the current directory holding the files it extracted:
#   $1  slot the stock updater wants to write (always 0)
#   $2  the firmware archive
#
# The bootloader verifies slot 0 at every boot (the first kernel_sz bytes
# of ubi_k0 against kernel0_crc, the first rootfs_sz bytes of ubi_r0
# against rootfs0_crc) and mirrors a valid slot 0 into slot 1. Like the
# stock updater, this writes slot 0 in place: the kernel into ubi_k0 and
# the root filesystem over the mounted ubi_r0 through its MTD device; the
# stock slot 1 copy is removed to make room for rootfs_data, which is
# created here as sysupgrade would.

img="$2"

mtd_of() {
	sed -n 's/^\(mtd[0-9]*\): .* "'"$1"'"$/\/dev\/\1/p' /proc/mtd
}

vol_id() {
	ubinfo -d 0 -N "$1" 2>/dev/null | sed -n 's/^Volume ID: *\([0-9]*\).*/\1/p'
}

vol_rm() {
	local id

	id=$(vol_id "$1")
	[ -z "$id" ] || ubirmvol /dev/ubi0 -n "$id"
}

check_md5() {
	local sum

	sum=$(md5sum "$1" | sed 's/ .*//')
	grep -q "^$sum  $1\$" md5.txt
}

active=$(nv getenv sw_active | sed 's/^.*=//')
if [ "$active" != 0 ]; then
	echo "The stock firmware is running from slot $active, expected 0"
	exit 1
fi

tar -xf "$img" uImage rootfs k_crc32 r_crc32 md5.txt || exit 1
check_md5 uImage || exit 1
check_md5 rootfs || exit 1

kernel=$(vol_id ubi_k0)
rootfs_mtd=$(mtd_of ubi_r0)
if [ -z "$kernel" ] || [ -z "$rootfs_mtd" ]; then
	echo "UBI volumes ubi_k0 or ubi_r0 not found"
	exit 1
fi

vol_rm ubi_r1
vol_rm rootfs_data
vol_rm rootfs

ubiupdatevol "/dev/ubi0_$kernel" uImage || exit 1
flash_eraseall "$rootfs_mtd" || exit 1
flashcp rootfs "$rootfs_mtd" || exit 1
ubimkvol /dev/ubi0 -N rootfs_data -m || exit 1

nv setenv kernel_sz "$(printf '%x' "$(wc -c < uImage)")"
nv setenv kernel0_crc "$(cat k_crc32)"
nv setenv rootfs_sz "$(printf '%x' "$(wc -c < rootfs)")"
nv setenv rootfs0_crc "$(cat r_crc32)"
nv setenv sw_version0 "OpenWrt"
nv setenv sw_commit 0
nv setenv sw_tryactive 2

rm -f uImage rootfs k_crc32 r_crc32 md5.txt
echo "OpenWrt installed into slot 0"
