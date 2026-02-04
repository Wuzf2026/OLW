## nc_boot

nc_boot 支持使用 U 盘进行升级或导出功能。

升级：支持升级 update.img 完整固件，或升级各分区镜像（uboot.img、boot.img、modules.tar.gz、recovery.img、rootfs.img、userdata.img）。

导出：支持导出 rootfs.img、userdata.img 和系统日志 log.tar.gz。

### 目录结构

```
nc_boot/
├── nc_boot.conf
├── update
│       ├── uboot.img
│       ├── boot.img
│       ├── modules.tar.gz
│       ├── recovery.img
│       ├── modify.sh
│       └── update.sh
├── export
└── nc-boot-20240422.log
```

nc_boot/nc_boot.conf 是配置文件。

将要升级的固件或分区镜像需要存放在 nc_boot/update 目录下。

导出的镜像或系统日志默认存放在 nc_boot/export 目录。

nc_boot/nc-boot-20240422.log 是升级导出产生的日志文件。

### nc_boot/nc_boot.conf 配置文件

NC_BOOT_CMD：设置此次操作是升级还是导出（update 或者 export）。

当 NC_BOOT_CMD 设置为 update 升级时，会在 nc_boot/update 目录中找完整固件 update.img 进行完整升级，若不存在完整固件 update.img 则找各分区镜像进行升级。支持同时升级多个分区镜像。

当 NC_BOOT_CMD 设置为 export 导出时，必须设置 NC_EXPORT_IMG_NAME。

**配置文件示例**

升级：

```
NC_BOOT_CMD="update"
```

导出 rootfs：

```
NC_BOOT_CMD="export"
NC_EXPORT_IMG_NAME="rootfs"
```

导出 rootfs 和 userdata：

```
NC_BOOT_CMD="export"
NC_EXPORT_IMG_NAME="rootfs userdata"
```

导出系统日志：

```
NC_BOOT_CMD="export"
NC_EXPORT_IMG_NAME="systemlog"
```

### 操作步骤

1. 将 U 盘的第一个分区格式化成 ext4 格式
2. 将下载、解压后的 nc_boot 目录放到 U 盘的第一个分区根目录下
3. 修改 nc_boot/nc_boot.conf 配置文件
4. 将 U 盘插到主板上，重新给主板上电（必须重新上电，reboot 重启无效）
5. 等待设备自动升级导出完成并自动重启进入正常系统界面

**提示**

* 一次升级或导出操作完成后，记得拔掉 U 盘或删除 U 盘中的 nc_boot/nc_boot.conf 配置文件，避免下次断电重启后再次进入升级或导出流程。

* 使用大容量 USB 3.0 U 盘，升级或导出速度更快。实测 128G USB 3.0 U 盘在 6 分钟左右能升级完成一个完整固件。

* 若需求将 U 盘的第一个分区格式化成 FAT32 格式。必须保证单个文件大小不超过 4GB，在升级 update.img 或导出 rootfs.img 时要注意检查文件大小。

* 若需求将 U 盘的第一个分区格式化成 exFAT 格式。必须在系统下手动执行 `reboot recovery` 命令，来代替步骤 4 中的”重新给主板上电“。

### modify.sh

`modify.sh` 脚本是在使用 U 盘升级分区镜像时自动执行，用于修改 rootfs。当前 `modify.sh` 仅为修改示例，具体实现根据需要自定义修改。

## update.sh

`update.sh` 脚本是在主板系统下手动执，用于升级部分分区镜像。`update.sh` 不需要使用 U 盘操作，其功能有限，只能升级当前目录下的 uboot.img、boot.img、modules.tar.gz、recovery.img。

### 使用方法

将下载、解压后的 nc_boot 目录放到主板系统下，手动执行：

```
cd nc_boot/update
sudo ./update.sh
```

升级完成后，手动重启生效。
