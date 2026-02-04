#!/bin/sh

NC_THIS_FILE=${BASH_SOURCE:-$0}
NC_THIS_FILE_DIR=$(dirname $NC_THIS_FILE)
NC_UPDATE_VERSION=V1.1

nc_checksum()
{
	NC_PART=$1
	NC_PARTIMG_MD5SUM_FILE=$NC_THIS_FILE_DIR/md5sum.txt

	if [ ! -e "$NC_PARTIMG_MD5SUM_FILE" ]; then
		echo "not found: $NC_PARTIMG_MD5SUM_FILE"
		return 0
	fi

	if [ "$NC_PART" = modules ]; then
		NC_PARTIMG=modules.tar.gz
	else
		NC_PARTIMG=${NC_PART}.img
	fi

	NC_PARTIMG_MD5SUM=$(cat $NC_PARTIMG_MD5SUM_FILE | grep -w "$NC_PARTIMG" | awk '{print $1}')
	if [ -z "$NC_PARTIMG_MD5SUM" ]; then
		echo "no md5sum: $NC_PARTIMG"
		return 0
	fi

	NC_MD5SUM=$(md5sum $NC_THIS_FILE_DIR/$NC_PARTIMG | awk '{print $1}')
	if [ "$NC_MD5SUM" != "$NC_PARTIMG_MD5SUM" ]; then
		echo "md5sum error: $NC_MD5SUM: $NC_PARTIMG_MD5SUM"
		return 1
	fi

	return 0
}

nc_update_modules()
{
	if [ -e "$NC_THIS_FILE_DIR/modules.tar.gz" ]; then
		if nc_checksum modules; then
			echo "start update modules"
			tar -xpf $NC_THIS_FILE_DIR/modules.tar.gz -C /lib/modules
			if [ $? -eq 0 ]; then
				echo "update modules ok"
			else
				echo "update modules fail"
			fi
		else
			echo "checksum fail: $NC_THIS_FILE_DIR/modules.tar.gz"
		fi
	else
		echo "modules.tar.gz not exist"
	fi

	return 0
}

nc_update()
{
	for part in uboot boot recovery; do
		if [ -e "$NC_THIS_FILE_DIR/$part.img" ]; then
			if [ -b "/dev/block/by-name/$part" ]; then
				if nc_checksum $part; then
					echo "start update $part"
					dd if=$NC_THIS_FILE_DIR/$part.img of=/dev/block/by-name/$part
					if [ $? -eq 0 ]; then
						echo "update $part ok"
					else
						echo "update $part fail"
					fi
				else
					echo "checksum fail: $NC_THIS_FILE_DIR/$part.img"
				fi
			else
				echo "invalid partition: /dev/block/by-name/$part"
			fi
		else
			echo "$part.img not exist"
		fi
	done

	nc_update_modules

	sync

	return 0
}

main()
{
	echo "NC_UPDATE_VERSION: $NC_UPDATE_VERSION"

	nc_update
}

main $@
