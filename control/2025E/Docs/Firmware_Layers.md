# MSPA / MSPB 五层架构

两个 MSPM0 工程均保留原有 `Core` 和 TI 原厂 `Driver`。`Driver` 目录只存放
TI DriverLib、CMSIS 等第三方代码，不再放本工程的功能模块。

```text
App       仅负责初始化顺序和主循环调度
  └─ Service  周期任务、状态处理、通信等业务逻辑
       └─ Control  PID、循迹/云台控制等纯控制算法
            └─ Device  WT901、循迹、编码器、电机等物理器件抽象
                 └─ BSP  GPIO、UART、Timer 等最底层硬件调用
                      └─ Driver  TI DriverLib（不写项目代码）
```

## MSPA 底盘主控

- `BSP/bsp_uart`：使能 UART0（ESP，PA28/PA31）和 UART2（MSPB，PA21/PA22）中断。
- `BSP/bsp_timer`：使能 TIMG6，并产生 10ms 底盘控制节拍。
- `Device/linetrack`：八路红外循迹设备。位 0 至位 7 对应 PA27、PA26、PA25、PA24、PA23、PB24、PB20、PA7。
- `Control/chassis_control`：底盘控制算法接口，不直接访问寄存器或 GPIO。
- `Service/chassis_task`：在每个 10ms 节拍读取循迹设备并调用控制层。
- `App/app_mspa`：初始化各层并调度服务。

## MSPB 云台主控

- `BSP/bsp_uart`：UART1 接收中断将 PA9 的 WT901 数据送入设备层；其余通信串口在协议接入前安全取走接收字节。
- `BSP/bsp_timer`：使能 TIMG6，并产生 1ms 云台控制节拍。
- `Device/wt901`：保留 F407 工程的 11 字节帧缓存、校验和、加速度/角速度/欧拉角换算。
- `Control/gimbal_axis`：云台轴控制算法接口，不直接访问步进器寄存器。
- `Service/gimbal_task`：每 1ms 解析已接收的 WT901 帧并调用控制层。
- `App/app_mspb`：初始化各层并调度服务。

WT901 仍由 SysConfig 配置为 UART1、PA8/PA9、115200。移植代码不会在上电时
执行原 F407 工程中“校准、修改输出率、改成 230400 波特率”等命令，避免改变传感器
现有设置。

## 注释规范

新增文件统一采用 `@file`、`@author`、`@brief`、`@date` 文件头；公开函数和
内部关键函数均在定义前保留中文 Doxygen 注释。SysConfig 自动生成的
`ti_msp_dl_config.c/.h` 不手动编辑。
