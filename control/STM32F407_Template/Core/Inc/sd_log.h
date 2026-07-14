/**
 * @file        sd_config.h
 * @author      JerryZheng
 *              codex
 * @brief       SD 配置文件
 * @warning     一定要先打开主日志再打开其他日志
 * @warning     一定要先关闭其他日志再关闭主日志
 * @date        2026-07-06
 */

#pragma once

#include <stdbool.h>
#include "ff.h"
#include "linetrack.h"

#ifdef __cplusplus
extern "C" {
#endif

/* <--------------------数据结构相关--------------------> */
typedef struct
{
    FIL file; //文件
    bool is_open; //是否打开
} SD_LogFile_t;

/* <--------------------错误代码相关--------------------> */
/*  
0  FR_OK             成功
1  FR_DISK_ERR       底层读写失败
3  FR_NOT_READY      卡或日志未初始化
11 FR_INVALID_DRIVE  FatFs驱动未注册
13 FR_NO_FILESYSTEM  文件系统不支持
19 FR_INVALID_PARAMETER 参数或日志过长
*/

/* <--------------------单文件函数相关--------------------> */
/**
 * @brief 直接初始化（创建文件或追加）
 * 
 * @param path 创建的文件路径
 * @return FRESULT 完成状态
 */
FRESULT SD_Log_Init(const char* path);

/**
 * @brief 日志系统启动（启动延时+日志写入+初始化文件）
 * 
 * @param path 日志写入路径
 * @param startup_message 启动消息（打印进日志）
 * @return FRESULT 完成状态
 */
FRESULT SD_Log_Start(const char* path, const char* startup_message);

/**
 * @brief 直接往主日志文件末尾追加原始数据
 * 
 * @param data 写入的信息内容
 * @param length 写入的信息长度
 * @param written 实际写入的长度
 * @return FRESULT 完成状态
 */
FRESULT SD_Log_Write(const void* data, UINT length, UINT* written);

/**
 * @brief 日志打印printf（RTC+信息+换行）
 * 
 * @example SD_Log_Printf("speed=%.2f", speed);（类似printf）
 * @retval int 实际追加字节数，失败返回-1 
 */
int SD_Log_Printf(const char* format, ...);

/**
 * @brief 读取主日志数据
 * 
 * @param offset 偏移量（从多少字节开始读）
 * @param buffer 读取数据缓冲区
 * @param length 读取长度
 * @param read 实际读取的长度
 * @return FRESULT 完成状态
 */
FRESULT SD_Log_Read(FSIZE_t offset, void* buffer, UINT length, UINT* read); //读取指定偏移量的数据，再移动到文件末

/**
 * @brief 把文件缓存和文件系统元数据提交到卡中
 * 
 * @note 适合在定期同步，错误，停止系统，电源下降使用，快速同步剩余信息
 * @return FRESULT 完成状态
 */
FRESULT SD_Log_Flush(void);

/**
 * @brief 关闭文件系统
 * 
 * @note 同步缓存 -> 关闭文件 -> 关闭文件系统（卸载fatfs）
 * @return FRESULT 完成状态
 */
FRESULT SD_Log_Close(void);

/**
 * @brief 判断文件打开状态
 * 
 * @retval bool 是否打开文件
 */
bool SD_Log_IsOpen(void);

/**
 * @brief 获取文件大小
 * 
 * @retval FSIZE_t 日志文件大小，单位：字节
 */
FSIZE_t SD_Log_Size(void);

/**
 * @brief 返回通用日志层最近一次记录的错误状态
 * 
 * @return FRESULT 最后一次错误状态
 */
FRESULT SD_Log_LastError(void);

/* <--------------------多文件函数相关--------------------> */
/**
 * @brief 多文件日志打开
 * 
 * @note 主日志必须处于打开状态，即s_log_open == true
 * @param log_file 日志文件对象地址
 * @param path 额外文件的路径
 * @return FRESULT 完成状态
 */
FRESULT SD_Log_FileOpen(SD_LogFile_t* log_file, const char* path);

/**
 * @brief 向指定额外日志文件写入
 * 
 * @note 主日志必须处于打开状态，即s_log_open == true
 * @param log_file 额外文件对象地址
 * @example SD_Log_FilePrintf(&motor_log,"left=%d,right=%d",left_pwm,right_pwm);
 * @return FRESULT 完成状态
 */
int SD_Log_FilePrintf(SD_LogFile_t* log_file, const char* format, ...);

/**
 * @brief 快速同步缓存到指定额外日志文件
 * 
 * @note 主日志必须处于打开状态，即s_log_open == true
 * @param log_file 额外文件对象地址
 * @return FRESULT 完成状态
 */
FRESULT SD_Log_FileFlush(SD_LogFile_t* log_file);

/**
 * @brief 关闭额外日志文件
 * 
 * @note 主日志必须处于打开状态，即s_log_open == true
 * @note 先快速同步缓存，再关闭文件，不卸载fatfs系统
 * @param log_file 额外文件对象地址
 * @return FRESULT 完成状态
 */
FRESULT SD_Log_FileClose(SD_LogFile_t* log_file);

/* <-------------------APP进程函数相关-------------------> */
/**
 * @brief 启动日志程序
 * 
 * @note 打开SYSTEM.LOG，打开LINE.LOG，打开WT901.LOG
 * @return FRESULT 完成状态
 */
FRESULT SD_Log_AppStart(void);

/**
 * @brief 关闭日志程序
 * 
 * @note 关闭LINE.LOG，关闭WT901.LOG，关闭SYSTEM.LOG
 * @return FRESULT 完成状态
 */
FRESULT SD_Log_AppStop(void);

/**
 * @brief 向主日志添加带RTC的信息
 * 
 * @note 主程序运行中，即s_app_ready == true
 * @example SD_Log_AppEvent("left=%d,right=%d",left_pwm,right_pwm);
 * @return FRESULT 完成状态
 */
int SD_Log_AppEvent(const char* format, ...);

/**
 * @brief APP进程
 * 
 * @note 写入正常LED快闪，写入错误慢闪
 * @note 周期同步写入WT901.LOG，LINE.LOG
 * @param line 最新循迹处理的结果的地址
 */
void SD_Log_AppProcess(const LINE_Result_t* line);

/**
 * @brief 返回整个应用日志系统当前是否可以继续自动记录
 * 
 * @retval bool true可以继续，false出现错误
 */
bool SD_Log_AppIsReady(void);

/**
 * @brief 返回最后一次APP错误
 * 
 * @return FRESULT APP最后一次错误
 */
FRESULT SD_Log_AppLastError(void);

#ifdef __cplusplus
}
#endif
