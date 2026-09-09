# 构建、烧录与恢复

## 环境

已验证环境是 Windows PowerShell、Git 和 ESP-IDF 6.1 / EIM，
目标为 **esp32s31 preview**。现有包装脚本的默认安装路径是：

- SDK：`C:\esp\v6.1\esp-idf`
- EIM：`C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1`
- Python：`C:\Espressif\tools\python\v6.1\venv\Scripts\python.exe`

脚本不升级 SDK，不修改其源码，并限制构建为最多两个并行任务。
其他安装路径需调整包装脚本路径，或在正确的 IDF 环境中使用下列直接构建命令。
本次拆分没有宣称 Linux / macOS 构建经过实测。

```powershell
git submodule update --init --recursive
./scripts/mosaico.ps1 configure
./scripts/mosaico.ps1 build
```

已加载 IDF 环境时的等效命令（从仓库根目录执行）：

```sh
idf.py --preview -C firmware/esp_mosaico -B artifacts/mosaico/build set-target esp32s31
idf.py --preview -C firmware/esp_mosaico -B artifacts/mosaico/build build
```

直接构建时也应设置 `IDF_PY_BUILD_JOBS=2` 和
`CMAKE_BUILD_PARALLEL_LEVEL=1`。保留依赖锁、sdkconfig.defaults 和子模块版本。
常规用户不要启用 `OFT_CABAC32` 等实验编译环境选项。

## 烧录边界：不是通用全量安装器

当前分区表来自实测 Mosaico V1.0，表位于 **0x9000**，
应用位于 **0x20000**，容量 **0x680000**。包装脚本只写应用：
保留 bootloader、分区表、NVS、PHY、其他 NOR 分区及 SPI NAND。

新设备即使同名也必须先核对芯片、分区布局和安全配置。
不要直接运行默认 `idf.py flash`、全片擦除或 NVS / NAND 擦除。
没有兼容工厂布局时，应单独制定安装方案，而不是强写相同地址。

手动进入下载模式后，先运行：

```powershell
./scripts/mosaico.ps1 chip
./scripts/mosaico.ps1 partition-scan
```

`partition-scan` 只读取闪存前缀并保存在忽略的 artifacts 中；
已有同名证据时拒绝覆盖。检查输出与
`firmware/esp_mosaico/partitions.csv` 一致后，才执行：

```powershell
./scripts/mosaico.ps1 flash
```

每次重新枚举串口，不要把 COM11 / COM12 当成固定配置。
多个 Espressif 设备连接时必须明确指定 `-Port` 并核对实体设备。
esptool 的芯片核验、烧录摘要与哈希结果应保留。

## 日志与恢复

```powershell
& C:\Espressif\tools\python\v6.1\venv\Scripts\python.exe scripts/mosaico-serial.py --seconds 30
```

普通无动作参数只记录日志；部分其他参数会启动相机控制、修改设置或电量计，
不是只读选项。不要将开发诊断参数当作初次启动流程。

已运行本固件时可显式请求 1200-baud USB CDC 下载恢复：

```powershell
& C:\Espressif\tools\python\v6.1\venv\Scripts\python.exe scripts/mosaico-serial.py --enter-download --seconds 15
```

该操作会中断应用。先停止相机控制并断开会话；若 USB 无响应，使用开发板
BOOT / 电源下载流程。重新核验芯片和端口后再刷写，不执行 eFuse、
加密或安全启动变更。

构建产物、ELF / BIN / map、串口日志与诊断数据位于 `artifacts/mosaico/`，
默认不提交。此仓库未附带可直接发布的固件二进制包。
