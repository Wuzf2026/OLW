#!/bin/sh

NC_THIS_FILE=${BASH_SOURCE:-$0}

# 打开风扇
turn_on()
{
	echo 1 > /sys/bus/platform/drivers/gpio-consumer/fan-ctrl/value
}

# 关闭风扇
turn_off()
{
	echo 0 > /sys/bus/platform/drivers/gpio-consumer/fan-ctrl/value
}

echo "start $NC_THIS_FILE"

if [ ! -e /sys/bus/platform/drivers/gpio-consumer/fan-ctrl/value ]; then
	echo "fan-ctrl: no fan ctrl"
	echo "end $NC_THIS_FILE"
	exit 0
fi

if [ ! -e /sys/class/thermal/thermal_zone0/temp ]; then
	turn_on
	echo "fan-ctrl: no temp ctrl"
	echo "end $NC_THIS_FILE"
	exit 0
else
	turn_on
	FAN_ON=1
	sleep 10
fi

THRESHOLD=60000
while :; do
	TEMPERATURE=`cat /sys/class/thermal/thermal_zone0/temp`
	if [ -n "$TEMPERATURE" ] && [ -n "$THRESHOLD" ]; then
		if [ $TEMPERATURE -gt $THRESHOLD ]; then
			if [ "$FAN_ON" != 1 ]; then
				turn_on
				FAN_ON=1
			fi
		else
			if [ "$FAN_ON" != 0 ]; then
				turn_off
				FAN_ON=0
			fi
		fi
	fi

	sleep 10
done
