# Firmware layer arrangement

The two MSPM0 projects keep the original `Core`, `BSP`, and TI `Driver`
directories.  The new application code is split by responsibility instead of
placing board access and vehicle logic in `main.c`.

```
Core/main.c
    -> App                 application startup and main-loop sequencing
        -> Service         non-blocking vehicle/gimbal tasks
            -> Control     control decisions without register access
            -> Driver      device-specific decoding and data handling
                -> BSP     MSPM0 peripheral and NVIC boundary
                    -> SysConfig generated configuration / TI DriverLib

Protocol
    -> byte buffering and future A/B, ESP, and Raspberry Pi frame parsing
```

## MSPA chassis controller

| Layer | Current module | Responsibility |
| --- | --- | --- |
| App | `app_mspa` | Starts the chassis services and runs their non-blocking loop. |
| BSP | `bsp_uart` | Enables the existing MSPA UART2/UART3 NVIC lines; peripheral setup stays in SysConfig. |
| Driver | `linetrack` | Reads `LINE_1` through `LINE_8` generated GPIO symbols and retains the F407 weighted line-position algorithm. |
| Control | `chassis_control` | Boundary for line result to chassis/motor command conversion. |
| Protocol | `ring_buffer` | Reusable ISR/task-safe byte-buffer primitive. |
| Service | `chassis_task`, `communication_service` | Calls line processing and provides the communication scheduling point. |

`LINE_1` through `LINE_8` retain their SysConfig order: PA27, PA26, PA25,
PA24, PA23, PB24, PB20, and PA7.  Bit 0 through bit 7 of `LINE_ReadRaw()`
follow exactly that order, so the original F407 weights remain applicable.

## MSPB gimbal controller

| Layer | Current module | Responsibility |
| --- | --- | --- |
| App | `app_mspb` | Starts gimbal services and runs the non-blocking loop. |
| BSP | `bsp_uart` | Enables existing UART NVIC lines and forwards UART1 RX bytes to WT901. |
| Driver | `wt901` | Retains the F407 11-byte frame buffering and acceleration/gyro/angle conversions. |
| Control | `gimbal_axis` | Boundary for future yaw/pitch control laws. |
| Protocol | `ring_buffer` | Reusable ISR/task-safe byte-buffer primitive. |
| Service | `gimbal_task`, `communication_service` | Drains WT901 frames and schedules gimbal control. |

WT901 remains on the SysConfig UART1 configuration: PA8 TX, PA9 RX and
115200 baud.  The port intentionally does **not** execute the F407
`WT901_Init()` sequence that calibrated the sensor and changed its baud rate
to 230400.  The MSPM0 `WT901_Init()` only resets parser state; configuration
commands can be added later as an explicit, operator-triggered service.

## Source-code conventions

All newly added C and header files use the existing `@file`, `@author`,
`@brief`, and `@date` library header style.  Public functions have a brief
comment immediately before the declaration.  Existing user files and
SysConfig-generated `ti_msp_dl_config.c/.h` are left untouched.
