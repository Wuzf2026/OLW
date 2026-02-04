#!/bin/sh

mcu_getdir()
{
	for file in $(ls -1 /sys/bus/i2c/drivers/new_mcu/*-0038/name 2>/dev/null); do
		if cat $file | grep -wq "new_mcu"; then
			MCU_DIR=${file%/*}
			break;
		fi
	done

	if [ -z "$MCU_DIR" ]; then
		echo "get mcu dir fail"
		exit 1
	fi
}

mcu_power_on()
{
	CFG_POWERON_MODE=$(cat $MCU_DIR/cfg_poweron_mode)
	if [ "$CFG_POWERON_MODE" != "65535" ]; then
		echo "cfg_poweron_mode change form $CFG_POWERON_MODE to 65535"
		echo 65535 > $MCU_DIR/cfg_poweron_mode
	else
		echo "cfg_poweron_mode already 65535"
	fi
}

mcu_power_not_on()
{
	CFG_POWERON_MODE=$(cat $MCU_DIR/cfg_poweron_mode)
	if [ "$CFG_POWERON_MODE" != "39593" ]; then
		echo "cfg_poweron_mode change form $CFG_POWERON_MODE to 39593"
		echo 39593 > $MCU_DIR/cfg_poweron_mode
	else
		echo "cfg_poweron_mode already 39593"
	fi
}

mcu_set_poweron_delay()
{
	CFG_POWERON_DELAY=$(cat $MCU_DIR/cfg_poweron_delay)
	if [ "$CFG_POWERON_DELAY" != "$MCU_POWERON_DELAY" ]; then
		echo "cfg_poweron_delay change form $CFG_POWERON_DELAY to $MCU_POWERON_DELAY"
		echo $MCU_POWERON_DELAY > $MCU_DIR/cfg_poweron_delay
	else
		echo "cfg_poweron_delay already $MCU_POWERON_DELAY"
	fi
}

MCU_POWERON=1
MCU_POWERON_DELAY=500

sleep 12
mcu_getdir

if [ "$MCU_POWERON" = "1" ]; then
	mcu_power_on
	mcu_set_poweron_delay
else
	mcu_power_not_on
fi
