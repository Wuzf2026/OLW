#/bin/sh

NC_THIS_FILE=${BASH_SOURCE:-$0}

echo "[NET-LED]: start $NC_THIS_FILE"

NET_IFACE="wlan0 "
NET_LED_PATH="/sys/class/leds/net-led"

if [ -d "$NET_LED_PATH" ]; then
	echo "[NET-LED]: use $NET_IFACE"
	echo $NET_IFACE > $NET_LED_PATH/device_name
	echo 1 > $NET_LED_PATH/link
	echo 1 > $NET_LED_PATH/tx
	echo 1 > $NET_LED_PATH/rx
else
	echo "[NET-LED]: net led not found"
fi

echo "[NET-LED]: end $NC_THIS_FILE"
