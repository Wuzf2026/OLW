#!/bin/sh

NC_THIS_FILE=${BASH_SOURCE:-$0}
NC_MODEM_VERSION=V1.5

nc_modem_log()
{
	if [ $MODEM_NETWORK_STATE_UNCHANGED = "0" ]; then
		echo "[NC-MODEM]: $@"
	fi
}

nc_modem_log_file()
{
	UART_RECEIVE_FILE=$1

	if [ -e "$UART_RECEIVE_FILE" ]; then
		if [ $MODEM_NETWORK_STATE_UNCHANGED = "0" ]; then
			while IFS= read -r line; do
				echo "[NC-MODEM]: $line"
			done < "$UART_RECEIVE_FILE"
		fi
	fi
}

nc_modem_get_device()
{
	MODEM_USB_BUS=""
	MODEM_USB_DEVICE=""
	MODEM_USB_ID=""
	MODEM_DEVICE_NAME=""

	LSUSB_OUT_FILE=$(mktemp)
	lsusb > $LSUSB_OUT_FILE

	while IFS= read -r line; do
		USB_BUS=$(echo $line | awk '{print $2}')
		USB_DEVICE=$(echo $line | awk '{print $4}' | tr -d ':')
		USB_ID=$(echo $line | awk '{print $6}')

		MODEM_FOUND=1

		if echo $USB_ID | grep -qE "2dee:4d54"; then
			MODEM_DEVICE_NAME="$MODEM_DEVICE_NAME LCM720"
		elif echo $USB_ID | grep -qE "2dee:4d4[123]"; then
			MODEM_DEVICE_NAME="$MODEM_DEVICE_NAME SLM3XX"
		elif echo $USB_ID | grep -qE "05c6:f601"; then
			MODEM_DEVICE_NAME="$MODEM_DEVICE_NAME SLM750"
		elif echo $USB_ID | grep -qE "2c7c:0904"; then
			MODEM_DEVICE_NAME="$MODEM_DEVICE_NAME EC200G"
		elif echo $USB_ID | grep -qE "2c7c:0900"; then
			MODEM_DEVICE_NAME="$MODEM_DEVICE_NAME RM500U"
		elif echo $USB_ID | grep -qE "2c7c:7001"; then
			MODEM_DEVICE_NAME="$MODEM_DEVICE_NAME RM500K"
		elif echo $USB_ID | grep -qE "2c7c:0800"; then
			MODEM_DEVICE_NAME="$MODEM_DEVICE_NAME RM500Q"
		elif echo $USB_ID | grep -qE "2dee:4d52"; then
			MODEM_DEVICE_NAME="$MODEM_DEVICE_NAME SRM810"
		elif echo $USB_ID | grep -qE "1e0e:9001"; then
			MODEM_DEVICE_NAME="$MODEM_DEVICE_NAME SIMCom"
		elif echo $USB_ID | grep -qE "05c6:|2c7c:"; then
			MODEM_DEVICE_NAME="$MODEM_DEVICE_NAME EC20"
		else
			MODEM_FOUND=0
		fi

		if [ "$MODEM_FOUND" = "1" ]; then
			MODEM_USB_BUS="$MODEM_USB_BUS $USB_BUS"
			MODEM_USB_DEVICE="$MODEM_USB_DEVICE $USB_DEVICE"
			MODEM_USB_ID="$MODEM_USB_ID $USB_ID"
		fi
	done < "$LSUSB_OUT_FILE"

	rm -f $LSUSB_OUT_FILE

	nc_modem_log "MODEM_USB_ID: $MODEM_USB_ID"
	nc_modem_log "MODEM_DEVICE_NAME: $MODEM_DEVICE_NAME"

	return 0
}

nc_modem_get_net_iface()
{
	MODEM_NET_IFACE=""
	MODEM_QMI_DEVICE=""
	MODEM_AT_DEVICE=""
	MODEM_RESET_DEVICE=""
	MODEM_DEVICE_INDEX=""
	local INDEX=1

	for USB_ID in $MODEM_USB_ID; do
		USB_BUS=$(echo "$MODEM_USB_BUS" | awk -v j=$INDEX '{print $j}')
		USB_DEVICE=$(echo "$MODEM_USB_DEVICE" | awk -v j=$INDEX '{print $j}')
		DEVPATH="/sys"$(udevadm info --query=path --name=/dev/bus/usb/$USB_BUS/$USB_DEVICE)

		NET_IFACE=$(ls -1 $DEVPATH/*/net | head -n 1)
		if [ -n "$NET_IFACE" ]; then
			MODEM_NET_IFACE="$MODEM_NET_IFACE $NET_IFACE"
		else
			MODEM_NET_IFACE="$MODEM_NET_IFACE NULL"
		fi

		AT_DEVICE=$(ls -1d $DEVPATH/*/tty* | awk -F '/' '{print $NF}' | sort | head -n 1)
		if [ -n "$AT_DEVICE" ]; then
			MODEM_AT_DEVICE="$MODEM_AT_DEVICE $AT_DEVICE"
		else
			MODEM_AT_DEVICE="$MODEM_AT_DEVICE NULL"
		fi

		QMI_DEVICE=$(ls -1 $DEVPATH/*/usbmisc | head -n 1)
		if [ -n "$QMI_DEVICE" ]; then
			MODEM_QMI_DEVICE="$MODEM_QMI_DEVICE $QMI_DEVICE"
		else
			MODEM_QMI_DEVICE="$MODEM_QMI_DEVICE NULL"
		fi

		RESET_DEVICE=""
		DEVICE_INDEX=""
		for file in $(ls -1 /proc/device-tree/reset-modem*/usb-bus-port 2>/dev/null); do
			DTS_USB_BUS=$(cat $file | tr -d '\0' | awk -F '-' '{print $1}')
			DTS_USB_PORT=$(cat $file | tr -d '\0' | awk -F '-' '{print $2}')
			if echo $DEVPATH | grep -qE "$DTS_USB_BUS"; then
				if [ -z "$DTS_USB_PORT" ] || echo $DEVPATH | grep -qE "\-${DTS_USB_PORT}$"; then
					RESET_DEVICE=$(echo $file | awk -F '/' '{print $4}')
					DEVICE_INDEX=$(echo $RESET_DEVICE | awk -F '-' '{print $3}')
					break;
				fi
			fi
		done

		if [ -n "$RESET_DEVICE" ]; then
			MODEM_RESET_DEVICE="$MODEM_RESET_DEVICE $RESET_DEVICE"
		else
			MODEM_RESET_DEVICE="$MODEM_RESET_DEVICE reset-modem"
		fi

		if [ -n "$DEVICE_INDEX" ]; then
			MODEM_DEVICE_INDEX="$MODEM_DEVICE_INDEX $DEVICE_INDEX"
		else
			MODEM_DEVICE_INDEX="$MODEM_DEVICE_INDEX 1"
		fi

		INDEX=$((INDEX+1))
	done

	nc_modem_log "MODEM_DEVICE_INDEX: $MODEM_DEVICE_INDEX"
	nc_modem_log "MODEM_NET_IFACE: $MODEM_NET_IFACE"

	return 0
}

nc_modem_ip_test()
{
	local NET_IFACE=$1

	MODEM_IP=$(ip -4 addr show $NET_IFACE | grep 'inet ' | awk '{print $2}' | cut -d'/' -f1)
	if [ -n "$MODEM_IP" ]; then
		nc_modem_log "$NET_IFACE: ip exist: $MODEM_IP"
		return 0
	else
		nc_modem_log "$NET_IFACE: none ip"
		return 1
	fi

	return 0
}

nc_modem_ping_test()
{
	local NET_IFACE=$1 DEVICE_INDEX=$2

	for ping_ip in $(eval echo \$MODEM_PING_IP_$DEVICE_INDEX); do
		for i in $(seq 1 3); do
			sleep 1
			ping -I $NET_IFACE -c 1 -W 3 $ping_ip 1>/dev/null 2>&1
			if [ $? -eq 0 ]; then
				nc_modem_log "$NET_IFACE: ping $ping_ip ok"
				return 0
			fi
		done
	done

	nc_modem_log "$NET_IFACE: ping $(eval echo \$MODEM_PING_IP_$DEVICE_INDEX) fail"
	return 1
}

nc_modem_at_query()
{
	local AT_DEVICE=$1

	nc_modem_log "AT_DEVICE: $AT_DEVICE"

	UART_RECEIVE_FILE=$(mktemp)

	uart -D $AT_DEVICE -r > $UART_RECEIVE_FILE &
	SUB_PID=$!

	sleep 2
	uart -D $AT_DEVICE -s -d 'at+cpin?\r\n'

	sleep 2
	uart -D $AT_DEVICE -s -d 'at+csq\r\n'

	sleep 2
	uart -D $AT_DEVICE -s -d 'at+cops?\r\n'

	sleep 5
	{ kill -9 $SUB_PID; } 1>/dev/null 2>&1

	nc_modem_log_file $UART_RECEIVE_FILE
	rm -f $UART_RECEIVE_FILE

	return 0
}

dial_LCM720()
{
	local NET_IFACE=$1 AT_DEVICE=$2 DEVICE_INDEX=$3

	AT_DEVICE="/dev/"$(echo $AT_DEVICE | awk '{ match($0, /[0-9]+$/); prefix = substr($0, 1, RSTART-1); number = substr($0, RSTART); new_number = number + 0; print prefix new_number; }')
	OPTION_APN=$(eval echo \$OPTION_APN_$DEVICE_INDEX)

	pkill -f "quectel-CM -i $NET_IFACE $OPTION_APN"
	sleep 1
	quectel-CM -i $NET_IFACE $OPTION_APN 1>/dev/null 2>&1 &

	nc_modem_at_query $AT_DEVICE

	return 0
}

dial_SLM3XX()
{
	local NET_IFACE=$1 AT_DEVICE=$2 DEVICE_INDEX=$3

	AT_DEVICE="/dev/"$(echo $AT_DEVICE | awk '{ match($0, /[0-9]+$/); prefix = substr($0, 1, RSTART-1); number = substr($0, RSTART); new_number = number + 0; print prefix new_number; }')

	uart -D $AT_DEVICE -s -d 'at+cgact=1\r\n'

	nc_modem_at_query $AT_DEVICE

	return 0
}

dial_SLM750()
{
	local NET_IFACE=$1 AT_DEVICE=$2 DEVICE_INDEX=$3

	AT_DEVICE="/dev/"$(echo $AT_DEVICE | awk '{ match($0, /[0-9]+$/); prefix = substr($0, 1, RSTART-1); number = substr($0, RSTART); new_number = number + 0; print prefix new_number; }')
	OPTION_APN=$(eval echo \$OPTION_APN_$DEVICE_INDEX)

	sleep 5

	pkill -f "quectel-CM -i $NET_IFACE $OPTION_APN"
	sleep 1
	quectel-CM -i $NET_IFACE $OPTION_APN 1>/dev/null 2>&1 &

	nc_modem_at_query $AT_DEVICE

	return 0
}

dial_EC200G()
{
	local NET_IFACE=$1 AT_DEVICE=$2 DEVICE_INDEX=$3

	AT_DEVICE="/dev/"$(echo $AT_DEVICE | awk '{ match($0, /[0-9]+$/); prefix = substr($0, 1, RSTART-1); number = substr($0, RSTART); new_number = number + 0; print prefix new_number; }')

	{ killall dhclient; } 1>/dev/null 2>&1
	{ killall udhcpc; } 1>/dev/null 2>&1
	uart -D $AT_DEVICE -s -d 'AT+QNETDEVCTL=1,3,1\r\n'
	sleep 3
	udhcpc -i $NET_IFACE &
	dhclient $NET_IFACE &

	nc_modem_at_query $AT_DEVICE

	return 0
}

dial_RM500U()
{
	local NET_IFACE=$1 AT_DEVICE=$2 DEVICE_INDEX=$3

	AT_DEVICE="/dev/"$(echo $AT_DEVICE | awk '{ match($0, /[0-9]+$/); prefix = substr($0, 1, RSTART-1); number = substr($0, RSTART); new_number = number + 2; print prefix new_number; }')

	{ killall dhclient; } 1>/dev/null 2>&1
	{ killall udhcpc; } 1>/dev/null 2>&1
	uart -D $AT_DEVICE -s -d 'AT+QNETDEVCTL=1,3,1\r\n'
	sleep 3
	udhcpc -i $NET_IFACE &
	dhclient $NET_IFACE &

	nc_modem_at_query $AT_DEVICE

	return 0
}

dial_RM500K()
{
	local NET_IFACE=$1 AT_DEVICE=$2 DEVICE_INDEX=$3

	AT_DEVICE="/dev/"$(echo $AT_DEVICE | awk '{ match($0, /[0-9]+$/); prefix = substr($0, 1, RSTART-1); number = substr($0, RSTART); new_number = number + 1; print prefix new_number; }')

	UART_RECEIVE_FILE=$(mktemp)
	uart -D $AT_DEVICE -r > $UART_RECEIVE_FILE &
	SUB_PID=$!
	sleep 10
	uart -D $AT_DEVICE -s -d 'AT+QAPNACT=1,"internet","default"\r\n'
	sleep 5
	{ kill -9 $SUB_PID; } 1>/dev/null 2>&1
	CID=$(cat $UART_RECEIVE_FILE | grep -oE "PDN ACT [0-9]+,[0-9+]" | cut -d ' ' -f 3 | cut -d ',' -f 1)
	if [ -z "$CID" ]; then
		return 0
	fi

	uart -D $AT_DEVICE -r > $UART_RECEIVE_FILE &
	SUB_PID=$!
	sleep 2
	uart -D $AT_DEVICE -s -d 'AT+CGPADDR='$CID'\r\n'
	sleep 5
	{ kill -9 $SUB_PID; } 1>/dev/null 2>&1

	IP_ADDR=$(cat $UART_RECEIVE_FILE | cut -d ',' -f 2 | grep -oE "[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}")
	if [ -z "$IP_ADDR" ]; then
		return 0
	fi

	rm -f $UART_RECEIVE_FILE

	sleep 3
	ifconfig $NET_IFACE ${IP_ADDR} up
	ip route add default dev $NET_IFACE table main
	systemd-resolve --interface $NET_IFACE --set-dns 119.29.29.29 --set-dns 223.5.5.5

	nc_modem_at_query $AT_DEVICE

	return 0
}

dial_RM500Q()
{
	local NET_IFACE=$1 AT_DEVICE=$2 DEVICE_INDEX=$3

	AT_DEVICE="/dev/"$(echo $AT_DEVICE | awk '{ match($0, /[0-9]+$/); prefix = substr($0, 1, RSTART-1); number = substr($0, RSTART); new_number = number + 3; print prefix new_number; }')
	OPTION_APN=$(eval echo \$OPTION_APN_$DEVICE_INDEX)

	pkill -f "quectel-CM -i $NET_IFACE $OPTION_APN"
	sleep 1
	quectel-CM -i $NET_IFACE $OPTION_APN 1>/dev/null 2>&1 &

	nc_modem_at_query $AT_DEVICE

	return 0
}

dial_SRM810()
{
	local NET_IFACE=$1 AT_DEVICE=$2 DEVICE_INDEX=$3

	AT_DEVICE="/dev/"$(echo $AT_DEVICE | awk '{ match($0, /[0-9]+$/); prefix = substr($0, 1, RSTART-1); number = substr($0, RSTART); new_number = number + 2; print prefix new_number; }')

	sleep 5
	uart -D $AT_DEVICE -s -d 'AT+CGDCONT=1,"IPV4V6","ctnet"\r\n'
	sleep 5
	uart -D $AT_DEVICE -s -d 'AT^NDISDUP=1,1\r\n'

	nc_modem_at_query $AT_DEVICE

	return 0
}

dial_SIMCom()
{
	local NET_IFACE=$1 AT_DEVICE=$2 DEVICE_INDEX=$3

	AT_DEVICE="/dev/"$(echo $AT_DEVICE | awk '{ match($0, /[0-9]+$/); prefix = substr($0, 1, RSTART-1); number = substr($0, RSTART); new_number = number + 2; print prefix new_number; }')
	OPTION_APN=$(eval echo \$OPTION_APN_$DEVICE_INDEX)

	sleep 5

	pkill -f "quectel-CM -i $NET_IFACE $OPTION_APN"
	sleep 1
	quectel-CM -i $NET_IFACE $OPTION_APN 1>/dev/null 2>&1 &

	nc_modem_at_query $AT_DEVICE

	return 0
}

dial_EC20()
{
	local NET_IFACE=$1 AT_DEVICE=$2 DEVICE_INDEX=$3

	AT_DEVICE="/dev/"$(echo $AT_DEVICE | awk '{ match($0, /[0-9]+$/); prefix = substr($0, 1, RSTART-1); number = substr($0, RSTART); new_number = number + 3; print prefix new_number; }')
	OPTION_APN=$(eval echo \$OPTION_APN_$DEVICE_INDEX)

	sleep 5

	pkill -f "quectel-CM -i $NET_IFACE $OPTION_APN"
	sleep 1
	quectel-CM -i $NET_IFACE $OPTION_APN 1>/dev/null 2>&1 &

	nc_modem_at_query $AT_DEVICE

	return 0
}

nc_modem_reset()
{
	local NET_IFACE=$1 RESET_DEVICE=$2

	nc_modem_log "$NET_IFACE: reset: $RESET_DEVICE"

	if [ -e /sys/bus/platform/drivers/gpio-reset-consumer/$RESET_DEVICE/reset ]; then
		echo 1 > /sys/bus/platform/drivers/gpio-reset-consumer/$RESET_DEVICE/reset
	else
		nc_modem_log "$RESET_DEVICE: none exist"
	fi

	return 0
}

nc_modem_dial()
{
	local INDEX=1 NETWORK_STATE_UNCHANGED=1

	if [ -n "$MODEM_USB_ID" ]; then
		MODEM_NOT_FOUND_TIMES=0
	else
		MODEM_NOT_FOUND_TIMES=$((MODEM_NOT_FOUND_TIMES+1))
		if [ $MODEM_NOT_FOUND_TIMES -gt $MAX_MODEM_NOT_FOUND_TIMES ]; then
			if [ -n "$TEST_OK_NET_IFACE" ]; then
				TEST_OK_NET_IFACE=""
				MODEM_NETWORK_STATE_UNCHANGED=0
			else
				MODEM_NETWORK_STATE_UNCHANGED=1
			fi
		fi

		return 0
	fi

	for USB_ID in $MODEM_USB_ID; do
		DEVICE_NAME=$(echo "$MODEM_DEVICE_NAME" | awk -v j=$INDEX '{print $j}')
		NET_IFACE=$(echo "$MODEM_NET_IFACE" | awk -v j=$INDEX '{print $j}')
		AT_DEVICE=$(echo "$MODEM_AT_DEVICE" | awk -v j=$INDEX '{print $j}')
		QMI_DEVICE=$(echo "$MODEM_QMI_DEVICE" | awk -v j=$INDEX '{print $j}')
		RESET_DEVICE=$(echo "$MODEM_RESET_DEVICE" | awk -v j=$INDEX '{print $j}')
		DEVICE_INDEX=$(echo "$MODEM_DEVICE_INDEX" | awk -v j=$INDEX '{print $j}')

		if [ -n "$NET_IFACE" ] && [ "$NET_IFACE" != "NULL" ]; then
			if nc_modem_ip_test $NET_IFACE && nc_modem_ping_test $NET_IFACE $DEVICE_INDEX; then
				FAILED_NET_IFACE=$(echo "$FAILED_NET_IFACE" | sed "s/ $NET_IFACE//g")
				if echo $TEST_OK_NET_IFACE | grep -q "$NET_IFACE"; then
					MODEM_NETWORK_STATE_UNCHANGED_NET_IFACE="$MODEM_NETWORK_STATE_UNCHANGED_NET_IFACE $NET_IFACE"
				else
					TEST_OK_NET_IFACE="$TEST_OK_NET_IFACE $NET_IFACE"
					MODEM_NETWORK_STATE_UNCHANGED_NET_IFACE=$(echo "$MODEM_NETWORK_STATE_UNCHANGED_NET_IFACE" | sed "s/ $NET_IFACE//g")
				fi
			else
				if echo "$DIALED_NET_IFACE" | grep -qvE "$NET_IFACE"; then
					nc_modem_log "dial: $NET_IFACE"
					[ ! -d /run/systemd/resolve ] && { mkdir -p /run/systemd/resolve; }
					eval dial_${DEVICE_NAME} $NET_IFACE $AT_DEVICE $DEVICE_INDEX
					DIALED_NET_IFACE="$DIALED_NET_IFACE $NET_IFACE"
				else
					FAILED_NET_IFACE="$FAILED_NET_IFACE $NET_IFACE"
				fi

				if [ "$(echo "$FAILED_NET_IFACE" | grep -o "$NET_IFACE" | wc -l)" -ge $MAX_NETWORK_FAILED_TIMES ]; then
					nc_modem_reset $NET_IFACE $RESET_DEVICE
					DIALED_NET_IFACE=$(echo "$DIALED_NET_IFACE" | sed "s/ $NET_IFACE//g")
					FAILED_NET_IFACE=$(echo "$FAILED_NET_IFACE" | sed "s/ $NET_IFACE//g")
					WAIT_TIME=2
					if echo $TEST_OK_NET_IFACE | grep -q "$NET_IFACE"; then
						TEST_OK_NET_IFACE=$(echo "$TEST_OK_NET_IFACE" | sed "s/ $NET_IFACE//g")
						MODEM_NETWORK_STATE_UNCHANGED_NET_IFACE=""
					fi
				fi

				if echo $TEST_OK_NET_IFACE | grep -q "$NET_IFACE"; then
					MODEM_NETWORK_STATE_UNCHANGED_NET_IFACE=$(echo "$MODEM_NETWORK_STATE_UNCHANGED_NET_IFACE" | sed "s/ $NET_IFACE//g")
				else
					MODEM_NETWORK_STATE_UNCHANGED_NET_IFACE="$MODEM_NETWORK_STATE_UNCHANGED_NET_IFACE $NET_IFACE"
				fi
			fi
		else
			MODEM_NETWORK_STATE_UNCHANGED_NET_IFACE="$MODEM_NETWORK_STATE_UNCHANGED_NET_IFACE $NET_IFACE"
		fi

		if [ "$(echo "$MODEM_NETWORK_STATE_UNCHANGED_NET_IFACE" | grep -o "$NET_IFACE" | wc -l)" -ge $MAX_NETWORK_UNCHANGED_TIMES ]; then
			MODEM_NETWORK_STATE_UNCHANGED_NET_IFACE=$(echo "$MODEM_NETWORK_STATE_UNCHANGED_NET_IFACE" | sed "s/ $NET_IFACE//")
			NETWORK_STATE_UNCHANGED=$((NETWORK_STATE_UNCHANGED*1))
		else
			NETWORK_STATE_UNCHANGED=$((NETWORK_STATE_UNCHANGED*0))
		fi

		INDEX=$((INDEX+1))
	done

	MODEM_NETWORK_STATE_UNCHANGED=$NETWORK_STATE_UNCHANGED

	return 0
}

nc_modem_sleep()
{
	sleep $WAIT_TIME

	WAIT_TIME=$((WAIT_TIME*2))
	if [ $WAIT_TIME -gt $MAX_WAIT_TIME ]; then
		WAIT_TIME=$MAX_WAIT_TIME
	fi
}

nc_modem_prepare()
{
	MODEM_PING_IP_1='119.29.29.29 223.5.5.5'
	MODEM_PING_IP_2='119.29.29.29 223.5.5.5'
	MAX_WAIT_TIME=30
	MAX_NETWORK_FAILED_TIMES=3

	WAIT_TIME=2

	DIALED_NET_IFACE=""
	FAILED_NET_IFACE=""

	TEST_OK_NET_IFACE=""
	MODEM_NETWORK_STATE_UNCHANGED_NET_IFACE=""
	MODEM_NETWORK_STATE_UNCHANGED=0
	MAX_NETWORK_UNCHANGED_TIMES=4

	MODEM_NOT_FOUND_TIMES=0
	MAX_MODEM_NOT_FOUND_TIMES=6
}

nc_modem_conf()
{
	if [ ! -e "/etc/norco/modem.conf" ]; then
		return 0
	fi

	. /etc/norco/modem.conf

	OPTION_APN_1=""
	if [ -n "$apn1" ]; then
		OPTION_APN_1="-s $apn1"
		if [ -n "$username1" ] && [ -n "$password1" ]; then
			OPTION_APN_1="$OPTION_APN_1 $username1 $password1"
			if [ -n "$auth1" ]; then
				OPTION_APN_1="$OPTION_APN_1 $auth1"
			fi
		fi

		if [ -n "$pingip1" ]; then
			MODEM_PING_IP_1=$pingip1
		fi
	fi

	OPTION_APN_2=""
	if [ -n "$apn2" ]; then
		OPTION_APN_2="-s $apn2"
		if [ -n "$username2" ] && [ -n "$password2" ]; then
			OPTION_APN_2="$OPTION_APN_2 $username2 $password2"
			if [ -n "$auth2" ]; then
				OPTION_APN_2="$OPTION_APN_2 $auth2"
			fi
		fi

		if [ -n "$pingip2" ]; then
			MODEM_PING_IP_2=$pingip2
		fi
	fi
}

nc_modem_main()
{
	nc_modem_prepare
	nc_modem_conf

	while :; do
		nc_modem_get_device
		nc_modem_get_net_iface
		nc_modem_dial
		nc_modem_sleep
	done
}

echo "start $NC_THIS_FILE"
echo "NC_MODEM_VERSION: $NC_MODEM_VERSION"

nc_modem_main $@
