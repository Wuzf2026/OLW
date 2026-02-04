#!/bin/sh

for i in /etc/norco/init.d/*.sh; do
	if [ -x $i ]; then
		$i &
	fi
done

