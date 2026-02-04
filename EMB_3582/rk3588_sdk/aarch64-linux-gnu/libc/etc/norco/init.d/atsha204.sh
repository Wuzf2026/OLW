#!/bin/sh

NC_THIS_FILE=${BASH_SOURCE:-$0}

sleep 15

echo "start $NC_THIS_FILE"

if false; then
	:;
else
	i2c_id=
	for file in `ls -1 /sys/bus/i2c/drivers/new_mcu/*-0038/name 2>/dev/null`; do
		if cat $file | grep -wq "new_mcu"; then
			i2c_id=$(echo $file | cut -d '/' -f 7 | cut -d '-' -f 1)
			break;
		fi
	done

	for file in `ls -1 /sys/bus/i2c/drivers/atsha204-i2c/*-0064/name 2>/dev/null`; do
		if cat $file | grep -wq "atsha204-i2c"; then
			if [ -n "$i2c_id" ]; then
				rm -rf /var/log/atsha204_client.*
				atsha204_client -d $i2c_id -a 0x38 1>/dev/null 2>&1 &
			else
				rm -rf /var/log/atsha204_client.*
				atsha204_client 1>/dev/null 2>&1 &
			fi
			break;
		fi
	done
fi

echo "end $NC_THIS_FILE"
