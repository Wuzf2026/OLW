#!/bin/sh

NC_THIS_FILE=${BASH_SOURCE:-$0}

sleep 10

echo "start $NC_THIS_FILE"

if false; then
	:;
else
	for file in `ls -1 /sys/devices/platform/*.i2c/i2c-*/*-007d/name 2>/dev/null`; do
		if cat $file | grep -wq "mcu"; then
			count=1
			while [ $count -le 5 ]; do
				mcu_init_arm 1>/dev/null 2>&1 &
				sleep 5
				mcu_check=$(cat /sys/devices/platform/*.i2c/i2c-*/*-007d/N79E8132A |grep "ccc = 1")
				if [ -n "$mcu_check" ]; then
					break;
				else
					count=$((count + 1))
				fi
			done
			break;
		fi
	done
fi

echo "end $NC_THIS_FILE"
