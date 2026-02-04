#!/bin/sh

NC_THIS_FILE=${BASH_SOURCE:-$0}

echo "start $NC_THIS_FILE"

if [ -f /etc/norco/firstBootFlag ]; then
	echo "firstBoot: first boot"
	sync
	sleep 2
	rm -rf /etc/norco/firstBootFlag
	sync
fi

echo "end $NC_THIS_FILE"
