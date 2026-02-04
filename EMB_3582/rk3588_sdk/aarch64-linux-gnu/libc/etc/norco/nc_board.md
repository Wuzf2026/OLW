## nc_board

nc_board 是系统支持多板型 NC-BOARD-ID 功能的一部分，用于在系统下查看当前板型、查看支持的板型、设置板型。

查看 nc_board 命令使用说明：

```
nc_board -h
...
Usage:
  nc_board [OPTION]...
OPTION:
  show                          view current board
  list                          view supported board
  set                           set board
Example:
  nc_board show
  nc_board list
  nc_board set
  nc_board set 1
```

查看当前板型：

```
nc_board show
...
currently set    board: EMB-3582-V1.2
currently loaded board: EMB-3582-V1.2
```

`currently set board` 为当前设置的板型。`currently loaded board` 为当前实际加载的板型。二者出现不一致的情况：

1. 还未设置板型，`currently set board` 显示为空。
2. 设置了板型，但未重启生效。
3. 设置了板型，但系统不支持多板型功能或未找到当前设置的板型，从而使用默认板型。

查看支持的板型：

```
nc_board list
...
default  supported board: EMB-3582-V1.2
optional supported board:
1) EMB-3582-V1.2
2) EMB-2582-V1.0
3) EMB-3582-V1.0
4) EMB-3582-V1.1-AFC92101-V1.0-PCIex4
5) EMB-3582-V1.1-AFC92101-V1.0
6) EMB-3582-V1.1-AFC92101-V2.0-PCIex4
7) EMB-3582-V1.1-AFC92101-V2.0
8) EMB-3582-V1.1
```

`default supported board` 为系统默认支持的板型，当系统不支持多板型功能或未找到当前设置的板型时将使用该板型。`optional supported board` 为系统支持的板型，可供设置。

设置板型：

```
nc_board set
...
optional supported board:
1) EMB-3582-V1.2
2) EMB-2582-V1.0
3) EMB-3582-V1.0
4) EMB-3582-V1.1-AFC92101-V1.0-PCIex4
5) EMB-3582-V1.1-AFC92101-V1.0
6) EMB-3582-V1.1-AFC92101-V2.0-PCIex4
7) EMB-3582-V1.1-AFC92101-V2.0
8) EMB-3582-V1.1
choose board:
```

输入对应的板型序号或板型并按下回车键：

```
1
```

设置成功后会提示：

```
set board: EMB-3582-V1.2 ok. please reboot.
```

重启系统后生效。

也可以根据 `nc_board list` 输出直接带板型序号或板型参数进行设置：

```
nc_board set 1
```

```
nc_board set EMB-3582-V1.2
```