#!/bin/sh

NC_THIS_FILE=${BASH_SOURCE:-$0}

nc_unlock()
{
	rmdir $NC_LOCK_DIR
}

nc_lock()
{
	NC_LOCK_DIR=/tmp/norco.lock

	while ! mkdir $NC_LOCK_DIR 1>/dev/null 2>&1; do sleep 1; done
}

set_on_time()
{
	nc_lock

	[ -e "/etc/norco/timer-power.conf" ] && . /etc/norco/timer-power.conf

	if [ -n "$POWER_ON_TIME" ]; then
		if [ $(date -d "$(date +%H:%M)" +%s) -lt $(date -d "$POWER_ON_TIME" +%s) ]; then
			TIMESTAMP=$(date -d "$POWER_ON_TIME today" +%s)
		else
			TIMESTAMP=$(date -d "$POWER_ON_TIME tomorrow" +%s)
		fi

		echo "[NC-POWER]: set power on time: $POWER_ON_TIME"
		echo 0 > /sys/class/rtc/rtc0/wakealarm
		echo $TIMESTAMP > /sys/class/rtc/rtc0/wakealarm
		if [ $? -ne 0 ]; then
			echo "[NC-POWER]: set power on time to rtc fail"
		fi
	fi

	if [ -n "$POWER_OFF_TIME" ]; then
		echo "[NC-POWER]: set power off time: $POWER_OFF_TIME"
	fi

	nc_unlock
}

turn_off()
{
	nc_lock

	[ -e "/etc/norco/timer-power.conf" ] && . /etc/norco/timer-power.conf

	if [ -n "$POWER_OFF_TIME" ]; then
		if [ "$(date +%H:%M)" = "$POWER_OFF_TIME" ]; then
			echo "[NC-POWER]: power off now: $POWER_OFF_TIME"
			sync
			sleep 1
			sync
			poweroff -f
		fi
	fi

	nc_unlock
}

echo "[NC-POWER]: start $NC_THIS_FILE"

while :; do
	set_on_time

	for i in $(seq 1 120); do
		turn_off
		sleep 30
	done
done

echo "[NC-POWER]: end $NC_THIS_FILE"
