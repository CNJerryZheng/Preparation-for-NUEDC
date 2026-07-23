/**
 * @file        linetrack_config.h
 * @author      JerryZheng
 * @brief       红外传感器配置文件
 * @date        2026-07-12
 */

#pragma once

/* <----------------接受方式----------------> */
/** @brief 使用串口模式读取循迹模块。 */
//#define LINETRACK_USE_UART
/** @brief 使用 GPIO 模式读取八路循迹模块。 */
#define LINETRACK_USE_IO

/* <---------------IO内设置----------------> */
#ifdef LINETRACK_USE_IO
/** @brief 模块检测到黑线时的 GPIO 电平。 */
#define LINETRACK_BLACK 0U
/** @brief 模块检测到白底时的 GPIO 电平。 */
#define LINETRACK_WHITE 1U
/** @brief 同时检测到该数量及以上黑线时判定为全黑区域。 */
#define LINETRACK_MAX_BLACK 6U
#endif

/* <----------------串口设置----------------> */
#ifdef LINETRACK_USE_UART

#endif

/* <-----------------ERROR-----------------> */
#if defined(LINETRACK_USE_IO) && defined(LINETRACK_USE_UART)
#error "至多开启一个模式"
#endif
#if !defined(LINETRACK_USE_IO) && !defined(LINETRACK_USE_UART)
#error "至少开启一个模式"
#endif
