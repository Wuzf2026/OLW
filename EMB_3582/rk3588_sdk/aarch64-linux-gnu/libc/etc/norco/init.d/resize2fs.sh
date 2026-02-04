#!/bin/sh

NC_THIS_FILE=${BASH_SOURCE:-$0}

echo "start $NC_THIS_FILE"

for block in rootfs oem userdata; do
	BLOCK_DEV=$(realpath /dev/block/by-name/$block 2>/dev/null)
	[ ! -b "$BLOCK_DEV" ] && BLOCK_DEV=$(blkid | grep 'LABEL="'$block'"' | awk -F ':' '{print $1}')
	[ -b "$BLOCK_DEV" ] && resize2fs $BLOCK_DEV
done

echo "end $NC_THIS_FILE"
