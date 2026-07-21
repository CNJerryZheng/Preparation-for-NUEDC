/**
 * @file        linetrack_config.h
 * @author      JerryZheng
 * @brief       红外传感器配置文件
 * @date        2026-07-12
 */

#pragma once

/* <----------------接受方式----------------> */
//#define LINETRACK_USE_UART//使用串口
#define LINETRACK_USE_IO //使用IO

/* <---------------IO内设置----------------> */
#ifdef LINETRACK_USE_IO
#define LINETRACK_BLACK 0U //黑色电压
#define LINETRACK_WHITE 1U //白色电压
#define LINETRACK_MAX_BLACK 6U //多少传感器黑视为全黑
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
