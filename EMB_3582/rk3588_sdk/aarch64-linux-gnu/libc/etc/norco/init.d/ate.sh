#!/bin/sh

NC_THIS_FILE=${BASH_SOURCE:-$0}

sleep 20

echo "start $NC_THIS_FILE"

[ ! -d /etc/norco/ate ] && { mkdir -p /etc/norco/ate; }
cd /etc/norco/ate
cp -f /usr/bin/client_common .
./client_common 1>/dev/null 2>&1 &

echo "end $NC_THIS_FILE"
