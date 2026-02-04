## NC-MODEM

### 拨号流程

开机自启动脚本：

```
/etc/norco/init.d/modem.sh
```

脚本工作流程：

```
nc_modem_main()
{
    while :; do
        nc_modem_get_device
        nc_modem_get_net_iface
        nc_modem_dial
        nc_modem_sleep
    done
}
```

**nc_modem_get_device**

获取 modem 模块的 USB ID，识别出对应的 4G/5G 模块型号，用于后续调用对应模块的拨号方法。

**nc_modem_get_net_iface**

获取 modem 模块生成的网络接口，如 wwan0，用于后续对指定网络接口进行拨号。

**nc_modem_dial**

根据实际模块型号，调用对应模块的拨号方法。拨号前会调用 `ip_test` 和 `ping_test` 测试是否已拨号分配了 IP，是否能正常 ping 通设置的 IP。

**nc_modem_sleep**

脚本睡眠等待。

**while**

while 循环的作用是，在拨号成功以后，周期执行 `ping_test` 检测网络是否正常，若网络持续异常，会尝试重启 4G/5G 模块并重新拨号。

### 日志分析

modem.sh 由 nc-init 服务启动，查看 nc-init 服务产生的日志：

```
journalctl -b 0 -u nc-init -e -f
```

正常拨号时的打印：

```
7月 05 17:55:05 linaro-alip norco.sh[625]: NC_MODEM_VERSION: V1.1
7月 05 17:55:06 linaro-alip norco.sh[625]: [NC-MODEM]: MODEM_USB_ID:
7月 05 17:55:06 linaro-alip norco.sh[625]: [NC-MODEM]: MODEM_DEVICE_NAME:
7月 05 17:55:06 linaro-alip norco.sh[625]: [NC-MODEM]: MODEM_NET_IFACE:
7月 05 17:55:23 linaro-alip norco.sh[625]: [NC-MODEM]: MODEM_USB_ID:  2c7c:0125
7月 05 17:55:23 linaro-alip norco.sh[625]: [NC-MODEM]: MODEM_DEVICE_NAME:  EC20
7月 05 17:55:23 linaro-alip norco.sh[625]: [NC-MODEM]: MODEM_NET_IFACE:  wwan0
7月 05 17:55:23 linaro-alip norco.sh[625]: [NC-MODEM]: wwan0: none ip
7月 05 17:55:23 linaro-alip norco.sh[625]: [NC-MODEM]: dial: wwan0
7月 05 17:55:29 linaro-alip norco.sh[625]: [NC-MODEM]: AT_DEVICE: /dev/ttyUSB3
7月 05 17:55:40 linaro-alip norco.sh[625]: [NC-MODEM]: at+cpin?
7月 05 17:55:40 linaro-alip norco.sh[625]: [NC-MODEM]: +CPIN: READY
7月 05 17:55:40 linaro-alip norco.sh[625]: [NC-MODEM]:
7月 05 17:55:40 linaro-alip norco.sh[625]: [NC-MODEM]: OK
7月 05 17:55:40 linaro-alip norco.sh[625]: [NC-MODEM]: at+csq
7月 05 17:55:40 linaro-alip norco.sh[625]: [NC-MODEM]: +CSQ: 23,99
7月 05 17:55:40 linaro-alip norco.sh[625]: [NC-MODEM]:
7月 05 17:55:40 linaro-alip norco.sh[625]: [NC-MODEM]: OK
7月 05 17:55:40 linaro-alip norco.sh[625]: [NC-MODEM]: at+cops?
7月 05 17:55:40 linaro-alip norco.sh[625]: [NC-MODEM]: +COPS: 0,0,"CHINA MOBILE",7
7月 05 17:55:40 linaro-alip norco.sh[625]: [NC-MODEM]:
7月 05 17:55:40 linaro-alip norco.sh[625]: [NC-MODEM]: OK
7月 05 17:55:58 linaro-alip norco.sh[625]: [NC-MODEM]: MODEM_USB_ID:  2c7c:0125
7月 05 17:55:58 linaro-alip norco.sh[625]: [NC-MODEM]: MODEM_DEVICE_NAME:  EC20
7月 05 17:55:58 linaro-alip norco.sh[625]: [NC-MODEM]: MODEM_NET_IFACE:  wwan0
7月 05 17:55:58 linaro-alip norco.sh[625]: [NC-MODEM]: wwan0: ip exist: 10.128.85.126
7月 05 17:55:59 linaro-alip norco.sh[625]: [NC-MODEM]: wwan0: ping 119.29.29.29 ok
```

**异常情况**

未识别到 modem 模块：

```
7月 05 17:55:06 linaro-alip norco.sh[625]: [NC-MODEM]: MODEM_USB_ID:
7月 05 17:55:06 linaro-alip norco.sh[625]: [NC-MODEM]: MODEM_DEVICE_NAME:
7月 05 17:55:06 linaro-alip norco.sh[625]: [NC-MODEM]: MODEM_NET_IFACE:
```

无 SIM 卡：

```
7月 05 18:09:58 linaro-alip norco.sh[548]: [NC-MODEM]: at+cpin?
7月 05 18:09:58 linaro-alip norco.sh[548]: [NC-MODEM]: +CME ERROR: 10
```

未接天线，无信号：

```
7月 05 18:00:35 linaro-alip norco.sh[714]: [NC-MODEM]: at+csq
7月 05 18:00:35 linaro-alip norco.sh[714]: [NC-MODEM]: +CSQ: 99,99
7月 05 18:00:35 linaro-alip norco.sh[714]: [NC-MODEM]:
7月 05 18:00:35 linaro-alip norco.sh[714]: [NC-MODEM]: OK
```

SIM 卡欠费或 SIM 卡其他原因导致注册不上：

```
7月 05 18:03:02 linaro-alip norco.sh[574]: [NC-MODEM]: at+cops?
7月 05 18:03:02 linaro-alip norco.sh[574]: [NC-MODEM]: +COPS: 0
7月 05 18:03:02 linaro-alip norco.sh[574]: [NC-MODEM]:
7月 05 18:03:02 linaro-alip norco.sh[574]: [NC-MODEM]: OK
```

获取不到 IP：

```
7月 05 18:04:21 linaro-alip norco.sh[574]: [NC-MODEM]: wwan0: none ip
```

网络持续异常，尝试复位 modem 模块：

```
7月 05 18:04:21 linaro-alip norco.sh[574]: [NC-MODEM]: reset modem
```
