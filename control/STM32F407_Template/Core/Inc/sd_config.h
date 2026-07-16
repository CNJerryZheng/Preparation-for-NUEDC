/**
 * @file        sd_config.h
 * @author      JerryZheng
 * @brief       SD 配置文件
 * @date        2026-07-6
 */

#pragma once

/* <---------------------打印路径---------------------> */
#define APP_SYSTEM_LOG_FILE_PATH "0:/SYSTEM.LOG"
#define APP_LINETRACK_LOG_FILE_PATH "0:/LINE.LOG"
#define APP_WT901_LOG_FILE_PATH "0:/WT901.LOG"

/* <---------------------打印内容---------------------> */
#define APP_SYSTEM_LOG_STARTUP_MESSAGE "System log start"
#define APP_LINETRACK_LOG_STARTUP_MESSAGE "Line tracking log start"
#define APP_WT901_LOG_STARTUP_MESSAGE "WT901 log start"

/* <---------------------打印配置---------------------> */
#define APP_LOG_LINE_SIZE 256U //临时缓冲区大小，溢出返回-1
#define APP_LOG_SAMPLE_INTERVAL_MS 20U //日志采样间隔：50Hz
#define APP_TF_STARTUP_DELAY_MS 100U //启动等待时长
#define APP_TF_DMA_TIMEOUT_MS 5000U //DMA传输超时
#define APP_LOG_FLUSH_INTERVAL_MS 1000U //日志刷新间隔

/* <--------------------打印LED配置--------------------> */
#define APP_LED_LOG_OK_DELAY_MS 1000U //日志写入成功时LED闪烁间隔
#define APP_LED_LOG_ERROR_DELAY_MS 100U //日志写入错误时LED闪烁间隔

/* <---------------------寄存器位置---------------------> */
#define APP_RTC_BACKUP_MAGIC 0x4071U //写入RTC备份寄存器的识别标记

/* <---------------------编译检查---------------------> */
#if APP_LOG_LINE_SIZE < 64U
#error "APP_LOG_LINE_SIZE must be at least 64 bytes"
#endif

#if APP_TF_DMA_TIMEOUT_MS == 0U
#error "APP_TF_DMA_TIMEOUT_MS must be greater than zero"
#endif
