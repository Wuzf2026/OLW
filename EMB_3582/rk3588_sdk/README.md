## NC-SDK

NC-SDK 是与系统配套的 SDK，用于交叉编译开发上层应用程序。

## 目录结构

```
NC-SDK
├── aarch64-linux-gnu
│   └── libc					// sysroot目录
├── bin
│   ├── aarch64-linux-gnu-gcc	// 交叉编译工具
├── environment-setup			// 环境变量配置脚本
└── toolchainfile.cmake			// cmake交叉编译配置文件
```

## 使用方法

解压命令：

```
tar -xpf NC-SDK-xxx.tar.gz
```

环境变量在当前终端生效：

```
source sdk/environment-setup
```

测试打印 sysroot 目录：

```
aarch64-linux-gnu-gcc -print-sysroot
```

对于使用 `./configure` 进行配置的项目，environment-setup 中定义了 `configure` 命令来代替。

对于 `cmake` 项目，environment-setup 中重新定义了 `cmake` 命令来代替。

对于手写 Makefile 项目，Makefile 中可以引用 environment-setup 中定义的变量或命令：`CC`、`CPP`、`pkg-config` 等。

## 使用示例

环境变量在当前终端生效：

```
source sdk/environment-setup
```

### 移植 iperf3

下载：

```
git clone https://github.com/esnet/iperf.git
```

配置：

```
configure
```

编译：

```
make
```

安装：

```
make DESTDIR=$SYSROOT install
```

测试运行：

```
sudo chroot $SYSROOT iperf3 -v
```

### 移植 curl

下载源码：

```
git clone https://github.com/curl/curl.git
```

配置：

```
cd curl
mkdir build && cd build
cmake ..
```

编译：

```
make
```

安装到 sysroot 中：

```
make DESTDIR=${SYSROOT} install
```

测试运行：

```
sudo chroot $SYSROOT curl -V
```

### 移植 libcjson

下载源码：

```
git clone https://github.com/DaveGamble/cJSON.git
```

配置：

```
cd cJSON
mkdir build && cd build
cmake ..
```

编译：

```
make
```

安装到 sysroot 中：

```
make DESTDIR=${SYSROOT} install
```

### 手写 Makefile 项目交叉编译

创建项目目录：

```
mkdir test_cjson
```

参考 cJSON 项目中的 `tests/readme_examples.c` 添加源码：

```
vim test_cjson/hello.c
...
#include <cjson/cJSON.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *json = "{\n\
                           \t\"code\":\"1\",\n\
                           \t\"data\":{\n\
                           \t\t\"begin\":0,\n\
                           \t\t\"end\":9840,\n\
                           \t\t\"keypointResultList\":[],\n\
                           \t\t\"txt\":\"hello\"\n\
                           \t},\n\
                           \t\"message\":\"\"\n\
                           }";

int CJSON_CDECL main(void)
{
    cJSON *monitor_json = cJSON_Parse(json);
    if (monitor_json == NULL) {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL) {
            fprintf(stderr, "Error before: %s\n", error_ptr);
        }
        goto end;
    }

    cJSON *code = cJSON_GetObjectItemCaseSensitive(monitor_json, "code");
    if (cJSON_IsString(code) && (code->valuestring != NULL)) {
        printf("code: %s\n", code->valuestring);
    }

    cJSON *data = cJSON_GetObjectItemCaseSensitive(monitor_json, "data");
    cJSON *begin = cJSON_GetObjectItemCaseSensitive(data, "begin");
    cJSON *end = cJSON_GetObjectItemCaseSensitive(data, "end");
    cJSON *txt = cJSON_GetObjectItemCaseSensitive(data, "txt");

    printf("begin: %d\n", begin->valueint);
    printf("end: %d\n", end->valueint);
    printf("txt: %s\n", txt->valuestring);

end:
    cJSON_Delete(monitor_json);

    return 0;
}
```

编写 Makefile：

```
vim test_cjson/Makefile
...
CFLAGS += `pkg-config --cflags libcjson`
LDFLAGS += `pkg-config --libs libcjson`

C_SOURCES := $(wildcard *.c)
C_EXECUTABLE := $(C_SOURCES:.c=)

all: $(C_EXECUTABLE)

$(C_EXECUTABLE): $(C_SOURCES)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS) $(LDLIBS)

clean:
	rm -rf $(C_EXECUTABLE)
```

编译：

```
cd test_cjson
make
```

将输出的可执行文件手动拷贝到 sysroot 中：

```
cp hello $SYSROOT/usr/bin
```

测试运行：

```
sudo chroot $SYSROOT hello
...
code: 1
begin: 0
end: 9840
txt: hello
```

## sysroot 说明

交叉编译项目通常需要依赖相关的头文件和库，可以在 sysroot 中安装相关软件开发包。比如，curl 依赖 OpenSSL，如果 sysroot 中没有预装 libssl-dev 开发包，则需要安装。

### 安装步骤

挂载：

```
sudo mount -o bind /dev $SYSROOT/dev
sudo chroot $SYSROOT
```

安装 `libssl-dev`：

```
apt update
apt install libssl-dev
```

修改绝对路径符号为相对路径：

```
find / -type l -lname '/*' | grep -v "/dev" | while IFS= read -r LINE; do rm -f $LINE; ln -sfr $(readlink $LINE) $LINE; done
```

为了避免 sysroot 中的符号链接指向开发环境系统中的文件，引起的交叉编译报错。

卸载：

```
exit
sudo umount $SYSROOT/dev
```
