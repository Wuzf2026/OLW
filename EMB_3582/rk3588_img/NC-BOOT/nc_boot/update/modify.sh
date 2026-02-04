#!/bin/sh

echo NC_BOOT_DIR: $NC_BOOT_DIR
echo NC_UPDATE_IMG_DIR: $NC_UPDATE_IMG_DIR
echo NC_ROOTFS_MNT: $NC_ROOTFS_MNT

[ ! -d "$NC_ROOTFS_MNT/etc/norco" ] && mkdir -p $NC_ROOTFS_MNT/etc/norco
date > $NC_ROOTFS_MNT/etc/norco/modify.log
cat $NC_BOOT_DIR/$NC_UPDATE_IMG_DIR/modify.sh | tee -a /$NC_ROOTFS_MNT/etc/norco/modify.log
