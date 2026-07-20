# MSPM0G3507 CMake 工程模板

这是一个用于 TI MSPM0G3507 的 C 工程模板，使用以下工具完成开发：

- CMake + Ninja：配置和编译工程
- Arm GNU Toolchain：编译、链接以及生成 ELF/HEX/BIN 文件
- TI SysConfig：配置时钟、GPIO 和外设，并生成初始化代码
- probe-rs + DAP-Link/CMSIS-DAP：烧录和调试
- Cortex-Debug + TI OpenOCD：备用调试方式

> 当前工程中的 `main.c` 是空循环，SysConfig 也没有配置 LED GPIO。因此成功烧录后 LED 不闪烁是正常现象，不代表烧录失败。

## 1. 获取工程

第一次使用：

```powershell
git clone https://github.com/CNJerryZheng/Preparation-for-NUEDC.git
cd Preparation-for-NUEDC\control\MSPM0_template
code .
```

已经克隆过工程：

```powershell
git pull origin main
```

请务必在 VS Code 中打开 `MSPM0_template` 文件夹，而不是只打开某一个源文件。

## 2. 安装开发工具

需要安装：

1. Git
2. Visual Studio Code
3. CMake（最低版本 3.20）
4. Ninja
5. Arm GNU Toolchain，确保包含：
   - `arm-none-eabi-gcc`
   - `arm-none-eabi-gdb`
   - `arm-none-eabi-objcopy`
   - `arm-none-eabi-size`
6. probe-rs
7. TI SysConfig
8. TI MSPM0 SDK，推荐使用工程当前版本 `2.02.00.05`

推荐安装的 VS Code 扩展：

- CMake Tools
- Debugger for probe-rs
- Cortex-Debug（仅在使用 OpenOCD 调试时需要）
- TI Embedded Debug（仅在使用 TI OpenOCD 调试时需要）

安装完成后重新启动 VS Code，并在新终端中检查：

```powershell
cmake --version
ninja --version
arm-none-eabi-gcc --version
probe-rs --version
```

如果命令提示“无法识别”，说明对应程序尚未加入 `PATH`。可以将其 `bin` 目录加入 Windows 环境变量，然后完全关闭并重新打开 VS Code。

## 3. 创建本机任务配置

仓库只保存不含个人路径的模板。每位开发者第一次拉取后执行：

```powershell
Copy-Item .vscode\tasks.json.template .vscode\tasks.json
```

打开新生成的 `.vscode/tasks.json`，替换其中全部占位符：

| 占位符 | 含义 | 示例 |
| --- | --- | --- |
| `__SYSCONFIG_ROOT__` | TI SysConfig 安装目录 | `D:/ti/SYSCONFIG` |
| `__MSPM0_SDK_ROOT__` | MSPM0 SDK 版本目录 | `D:/ti/MSPM0_SDK/mspm0_sdk_2_02_00_05` |

路径推荐使用 `/`。例如：

```json
"-SysConfigRoot",
"D:/ti/SYSCONFIG",
"-Mspm0SdkRoot",
"D:/ti/MSPM0_SDK/mspm0_sdk_2_02_00_05"
```

`.vscode/tasks.json` 已加入 `.gitignore`，其中的本机绝对路径不会上传到 GitHub。不要修改模板文件来保存个人路径。

## 4. 第一次配置和编译

在 VS Code 中按 `Ctrl+Shift+P`，依次执行：

1. `CMake: Select Configure Preset`
2. 选择 `MSPM0 ARM GCC (Debug)`，对应预设名为 `arm-gcc-debug`
3. 执行 `CMake: Configure`

然后选择：

```text
终端 → 运行任务 → MSPM0: Build
```

也可以在项目终端中执行：

```powershell
cmake --preset arm-gcc-debug
cmake --build --preset arm-gcc-debug
```

编译成功后，主要输出位于 `build` 目录：

- `build/MSPB.elf`：烧录和调试使用
- `build/MSPB.hex`
- `build/MSPB.bin`
- `build/MSPB.map`

## 5. 使用 DAP-Link 烧录并运行

连接关系至少需要：

- SWDIO
- SWCLK
- GND
- 目标板供电/VTref

连接 DAP-Link 后，在 VS Code 中选择：

```text
终端 → 运行任务 → MSPM0: Build, Flash & Run
```

该任务会按顺序执行：

1. 编译工程
2. 使用 probe-rs 下载 `build/MSPB.elf`
3. 复位 MSPM0G3507 并运行程序

也可以分别执行：

```powershell
probe-rs download --chip MSPM0G3507 --verify build/MSPB.elf
probe-rs reset --chip MSPM0G3507
```

烧录后程序会保存在芯片 Flash 中。以后重新上电时，不需要按 F5，程序会自行启动。

## 6. 调试程序

### probe-rs（推荐）

1. 先成功编译工程
2. 打开 VS Code 的“运行和调试”面板
3. 选择 `probe-rs Debug (MSPM0G3507)`
4. 按 F5

### Cortex-Debug + TI OpenOCD（备用）

1. 安装 Cortex-Debug 和 TI Embedded Debug 扩展
2. 确认 TI Embedded Debug 已配置 OpenOCD 路径
3. 选择 `Cortex-Debug (DAP-Link + MSPM0G3507)`
4. 按 F5

工程中的 `openocd.cfg` 已选择 `cmsis-dap` 接口和 TI MSPM0 目标配置。

## 7. 修改并生成 SysConfig 文件

SysConfig 的源配置文件是：

```text
Core/MSPB.syscfg
```

正确流程：

1. 运行任务 `MSPM0: Open SysConfig GUI`
2. 在图形界面中修改外设配置
3. 保存 `Core/MSPB.syscfg`
4. 运行任务 `MSPM0: Generate SysConfig`
5. 重新编译工程

生成任务会更新：

```text
Core/Src/ti_msp_dl_config.c
Core/Inc/ti_msp_dl_config.h
```

不要只在 GUI 中保存后就直接编译。保存 `.syscfg` 只更新配置源文件，必须再运行生成任务，C/H 文件才会同步。

生成脚本不会覆盖工程自定义的 `mspm0g3507.lds` 链接脚本。

## 8. Git 协作规范

建议提交以下内容：

- `Core/MSPB.syscfg`
- `Core/Src/ti_msp_dl_config.c`
- `Core/Inc/ti_msp_dl_config.h`
- 自己修改的源码和头文件

不要提交：

- `.vscode/tasks.json`：包含个人安装路径
- `build/`、`build-release/`：本机构建产物

修改 SysConfig 后，建议三份文件一起提交：

```powershell
git add Core/MSPB.syscfg Core/Src/ti_msp_dl_config.c Core/Inc/ti_msp_dl_config.h
git commit -m "配置：更新 SysConfig 外设配置"
git push
```

开始修改代码前先拉取同伴更改：

```powershell
git pull --rebase origin main
```

如果 Git 提示冲突，不要随意删除同伴代码，先确认冲突文件再处理。

## 9. 常见问题

### `ninja: no work to do`

这不是报错，表示源码从上次构建后没有变化。

### `probe-rs.exe was not found` 或无法识别 `probe-rs`

probe-rs 没有加入 `PATH`，或者 VS Code 是在修改环境变量之前启动的。加入 PATH 后彻底退出并重启 VS Code。

### 找不到 `ti_msp_dl_config.c/.h` 的新配置

保存 SysConfig 后，还需要执行 `MSPM0: Generate SysConfig`。

### 找不到 SysConfig 或 MSPM0 SDK

检查 `.vscode/tasks.json` 中的两个路径是否指向真实存在的目录，并确认没有保留 `__...__` 占位符。

### 编译时使用了 `cl.exe`

选择了 Visual Studio 生成器。重新选择 CMake 预设 `arm-gcc-debug`，该工程必须使用 Ninja 和 Arm GCC。

### 烧录成功但 LED 不亮

烧录成功只代表程序写入并运行，不代表 LED 引脚配置正确。需要同时确认：

- 原理图上的实际 LED 引脚
- LED 是高电平点亮还是低电平点亮
- SysConfig 中已将该引脚配置为 GPIO 数字输出
- `main.c` 中使用的是 SysConfig 生成的正确宏名

当前模板没有配置 LED，因此默认不会闪灯。
