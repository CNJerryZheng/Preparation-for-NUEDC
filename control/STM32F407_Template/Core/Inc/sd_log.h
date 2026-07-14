#pragma once

#include <stdbool.h>
#include "ff.h"
#include "linetrack.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    FIL file;
    bool is_open;
} SD_LogFile_t;

/**
 * @brief Mount the TF card and open a log file for read/append access.
 * @param path FatFs path, for example "0:/LOG.TXT".
 */
FRESULT SD_Log_Init(const char* path); //初始化创建文件

FRESULT SD_Log_Write(const void* data, UINT length, UINT* written); //本行继续追加数据

/**
 * Append a timestamped printf-style line.
 * Example: SD_Log_Printf("speed=%.2f", speed);
 * @return Number of bytes appended, or -1 on error.
 */
int SD_Log_Printf(const char* format, ...); //RTC+日志合并输出并换行

FRESULT SD_Log_FileOpen(SD_LogFile_t* log_file, const char* path);
int SD_Log_FilePrintf(SD_LogFile_t* log_file, const char* format, ...);
FRESULT SD_Log_FileFlush(SD_LogFile_t* log_file);
FRESULT SD_Log_FileClose(SD_LogFile_t* log_file);

FRESULT SD_Log_Read(FSIZE_t offset, void* buffer, UINT length, UINT* read); //读取指定偏移量的数据，再移动到文件末

FRESULT SD_Log_Flush(void); //把文件缓存和文件系统元数据提交到卡中

FRESULT SD_Log_Close(void); //关闭文件并卸载TF卡

bool SD_Log_IsOpen(void); //判断日志文件是否可用
FSIZE_t SD_Log_Size(void); //获取日志文件大小，单位：字节
FRESULT SD_Log_Start(const char* path, const char* startup_message); //
FRESULT SD_Log_LastError(void);

/**
 * @brief Start the application log set: SYSTEM.TXT, LINE.TXT and WT901.TXT.
 * @return FatFs status of the complete startup sequence.
 */
FRESULT SD_Log_AppStart(void);
FRESULT SD_Log_AppStop(void);
int SD_Log_AppEvent(const char* format, ...);

/**
 * @brief Run the non-blocking LED indication and periodic application logging.
 * @param line Latest line-tracking result.
 */
void SD_Log_AppProcess(const LINE_Result_t* line);

bool SD_Log_AppIsReady(void);
FRESULT SD_Log_AppLastError(void);
//返回最后一次错误的结果代码
/*  
0  FR_OK             成功
1  FR_DISK_ERR       底层读写失败
3  FR_NOT_READY      卡或日志未初始化
11 FR_INVALID_DRIVE  FatFs驱动未注册
13 FR_NO_FILESYSTEM  文件系统不支持
19 FR_INVALID_PARAMETER 参数或日志过长
*/

#ifdef __cplusplus
}
#endif
