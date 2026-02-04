#!/bin/bash
### BEGIN INIT INFO
# Provides:          rockchip
# Required-Start:
# Required-Stop:
# Default-Start:
# Default-Stop:
# Short-Description:
# Description:       Setup rockchip platform environment
### END INIT INFO

COMPATIBLE=$(cat /proc/device-tree/compatible)
if [[ $COMPATIBLE =~ "rk3288" ]]; then
    CHIPNAME="rk3288"
elif [[ $COMPATIBLE =~ "rk3328" ]]; then
    CHIPNAME="rk3328"
elif [[ $COMPATIBLE =~ "rk3399" && $COMPATIBLE =~ "rk3399pro" ]]; then
    CHIPNAME="rk3399pro"
elif [[ $COMPATIBLE =~ "rk3399" ]]; then
    CHIPNAME="rk3399"
elif [[ $COMPATIBLE =~ "rk3326" ]]; then
    CHIPNAME="rk3326"
elif [[ $COMPATIBLE =~ "px30" ]]; then
    CHIPNAME="px30"
elif [[ $COMPATIBLE =~ "rk3128" ]]; then
    CHIPNAME="rk3128"
elif [[ $COMPATIBLE =~ "rk3566" ]]; then
    CHIPNAME="rk3566"
elif [[ $COMPATIBLE =~ "rk3568" ]]; then
    CHIPNAME="rk3568"
else
    CHIPNAME="rk3036"
fi
COMPATIBLE=${COMPATIBLE#rockchip,}
BOARDNAME=${COMPATIBLE%%rockchip,*}


if [ -e /usr/bin/gst-launch-1.0 ]
then
    setcap CAP_SYS_ADMIN+ep /usr/bin/gst-launch-1.0
fi

# Cannot open pixbuf loader module file
if [ -e "/usr/lib/arm-linux-gnueabihf/gdk-pixbuf-2.0/gdk-pixbuf-query-loaders" ] ;
then
	/usr/lib/arm-linux-gnueabihf/gdk-pixbuf-2.0/gdk-pixbuf-query-loaders > /usr/lib/arm-linux-gnueabihf/gdk-pixbuf-2.0/2.10.0/loaders.cache
	update-mime-database /usr/share/mime/
elif [ -e "/usr/lib/aarch64-linux-gnu/gdk-pixbuf-2.0/gdk-pixbuf-query-loaders" ];
then
	/usr/lib/aarch64-linux-gnu/gdk-pixbuf-2.0/gdk-pixbuf-query-loaders > /usr/lib/aarch64-linux-gnu/gdk-pixbuf-2.0/2.10.0/loaders.cache
fi

if [[ "$CHIPNAME" == "rk3288" ]]; then
    GPU_VERSION=$(cat /sys/devices/platform/*gpu/gpuinfo)
    if echo $GPU_VERSION|grep -q r1p0; then
        ln -sf libmali-midgard-t76x-r18p0-r1p0-gbm.so /usr/lib/arm-linux-gnueabihf/libmali-gbm.so.1.9.0
        ln -sf libmali-midgard-t76x-r18p0-r1p0-x11.so /usr/lib/arm-linux-gnueabihf/libmali-x11.so.1.9.0
        ln -sf libmali-midgard-t76x-r18p0-r1p0-x11.so /usr/lib/arm-linux-gnueabihf/libmali.so.1.9.0
        ln -sf libmali-midgard-t76x-r18p0-r1p0-wayland.so /usr/lib/arm-linux-gnueabihf/libmali-wayland.so.1.9.0
    else
        ln -sf libmali-midgard-t76x-r18p0-r0p0-gbm.so /usr/lib/arm-linux-gnueabihf/libmali-gbm.so.1.9.0
        ln -sf libmali-midgard-t76x-r18p0-r0p0-x11.so /usr/lib/arm-linux-gnueabihf/libmali-x11.so.1.9.0
        ln -sf libmali-midgard-t76x-r18p0-r0p0-x11.so /usr/lib/arm-linux-gnueabihf/libmali.so.1.9.0
        ln -sf libmali-midgard-t76x-r18p0-r0p0-wayland.so /usr/lib/arm-linux-gnueabihf/libmali-wayland.so.1.9.0
    fi
fi
