#include "sd_log.h"

#include "fatfs.h"
#include "gpio.h"
#include "rtc.h"
#include "sd_config.h"
#include "wt901.h"

#include <stdarg.h>
#include <stdio.h>

static FIL s_log_file;
static bool s_log_open;
static FRESULT s_last_error = FR_NOT_READY;
static SD_LogFile_t s_line_log;
static SD_LogFile_t s_wt901_log;
static FRESULT s_app_error = FR_NOT_READY;
static bool s_app_ready;
static uint32_t s_log_sample_tick;
static uint32_t s_log_flush_tick;
static uint32_t s_led_tick;

static FRESULT SD_Log_SetError(FRESULT result)
{
    s_last_error = result;
    return result;
}

static void SD_Log_AppSetError(FRESULT result)
{
    s_app_error = SD_Log_SetError(result);
    s_app_ready = false;
}

static const char* SD_Log_LineStateToString(LINE_State_t state)
{
    switch (state)
    {
    case State_OK:
        return "OK";
    case State_Lose_Left:
        return "LOSE_LEFT";
    case State_Lose_Right:
        return "LOSE_RIGHT";
    case State_Lose_Unknown:
        return "LOSE_UNKNOWN";
    case State_All_Black:
        return "ALL_BLACK";
    case State_Discontinuous:
        return "DISCONTINUOUS";
    case State_Unknown:
    default:
        return "UNKNOWN";
    }
}

static FRESULT SD_Log_WriteFile(FIL* file, const void* data, UINT length, UINT* written)
{
    UINT local_written = 0U;
    FRESULT result;

    if (written != NULL)
    {
        *written = 0U;
    }

    if ((file == NULL) || (data == NULL) || (length == 0U))
    {
        return SD_Log_SetError(FR_INVALID_PARAMETER);
    }

    result = f_lseek(file, f_size(file));
    if (result == FR_OK)
    {
        result = f_write(file, data, length, &local_written);
    }

    if ((result == FR_OK) && (local_written != length))
    {
        result = FR_DISK_ERR;
    }

    if (written != NULL)
    {
        *written = local_written;
    }

    return SD_Log_SetError(result);
}

static int SD_Log_VPrintf(FIL* file, const char* format, va_list arguments)
{
    RTC_TimeTypeDef time = { 0 };
    RTC_DateTypeDef date = { 0 };
    char line[APP_LOG_LINE_SIZE];
    int prefix_length;
    int body_length;
    UINT total_length;
    UINT written;

    if ((file == NULL) || (format == NULL))
    {
        (void)SD_Log_SetError(FR_INVALID_PARAMETER);
        return -1;
    }

    if ((HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK) ||
        (HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK))
    {
        (void)SD_Log_SetError(FR_INT_ERR);
        return -1;
    }

    prefix_length = snprintf(line, sizeof(line),
                             "[20%02u-%02u-%02u %02u:%02u:%02u] ",
                             (unsigned int)date.Year, (unsigned int)date.Month,
                             (unsigned int)date.Date, (unsigned int)time.Hours,
                             (unsigned int)time.Minutes, (unsigned int)time.Seconds);
    if ((prefix_length < 0) || ((UINT)prefix_length >= sizeof(line)))
    {
        (void)SD_Log_SetError(FR_INVALID_PARAMETER);
        return -1;
    }

    body_length = vsnprintf(&line[prefix_length],
                            sizeof(line) - (UINT)prefix_length - 2U,
                            format, arguments);
    if ((body_length < 0) ||
        ((UINT)body_length >= (sizeof(line) - (UINT)prefix_length - 2U)))
    {
        (void)SD_Log_SetError(FR_INVALID_PARAMETER);
        return -1;
    }

    total_length = (UINT)prefix_length + (UINT)body_length;
    line[total_length++] = '\r';
    line[total_length++] = '\n';

    if (SD_Log_WriteFile(file, line, total_length, &written) != FR_OK)
    {
        return -1;
    }

    return (int)written;
}

FRESULT SD_Log_Init(const char* path)
{
    FRESULT result;

    if (path == NULL)
    {
        return SD_Log_SetError(FR_INVALID_PARAMETER);
    }

    if (s_log_open)
    {
        return SD_Log_SetError(FR_OK);
    }

    if (retSD != 0U)
    {
        return SD_Log_SetError(FR_INVALID_DRIVE);
    }

    result = f_mount(&SDFatFS, SDPath, 1U);
    if (result != FR_OK)
    {
        return SD_Log_SetError(result);
    }

    result = f_open(&s_log_file, path, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
    if (result != FR_OK)
    {
        (void)f_mount(NULL, SDPath, 1U);
        return SD_Log_SetError(result);
    }

    result = f_lseek(&s_log_file, f_size(&s_log_file));
    if (result != FR_OK)
    {
        (void)f_close(&s_log_file);
        (void)f_mount(NULL, SDPath, 1U);
        return SD_Log_SetError(result);
    }

    s_log_open = true;
    return SD_Log_SetError(FR_OK);
}

FRESULT SD_Log_Start(const char* path, const char* startup_message)
{
    FRESULT result;

    /* Allow the card supply and internal controller to become stable. */
    HAL_Delay(APP_TF_STARTUP_DELAY_MS);

    result = SD_Log_Init(path);
    if ((result == FR_OK) && (startup_message != NULL) && (startup_message[0] != '\0'))
    {
        if (SD_Log_Printf("%s", startup_message) < 0)
        {
            result = SD_Log_LastError();
        }
    }

    if (result == FR_OK)
    {
        result = SD_Log_Flush();
    }

    return SD_Log_SetError(result);
}

FRESULT SD_Log_Write(const void* data, UINT length, UINT* written)
{
    if (!s_log_open)
    {
        return SD_Log_SetError(FR_NOT_READY);
    }

    return SD_Log_WriteFile(&s_log_file, data, length, written);
}

int SD_Log_Printf(const char* format, ...)
{
    va_list arguments;
    int result;

    if ((!s_log_open) || (format == NULL))
    {
        (void)SD_Log_SetError(FR_NOT_READY);
        return -1;
    }

    va_start(arguments, format);
    result = SD_Log_VPrintf(&s_log_file, format, arguments);
    va_end(arguments);

    return result;
}

FRESULT SD_Log_FileOpen(SD_LogFile_t* log_file, const char* path)
{
    FRESULT result;

    if ((log_file == NULL) || (path == NULL))
    {
        return SD_Log_SetError(FR_INVALID_PARAMETER);
    }

    if (!s_log_open)
    {
        return SD_Log_SetError(FR_NOT_READY);
    }

    if (log_file->is_open)
    {
        return SD_Log_SetError(FR_OK);
    }

    result = f_open(&log_file->file, path, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
    if (result == FR_OK)
    {
        result = f_lseek(&log_file->file, f_size(&log_file->file));
        if (result != FR_OK)
        {
            (void)f_close(&log_file->file);
        }
    }

    if (result == FR_OK)
    {
        log_file->is_open = true;
    }

    return SD_Log_SetError(result);
}

int SD_Log_FilePrintf(SD_LogFile_t* log_file, const char* format, ...)
{
    va_list arguments;
    int result;

    if ((log_file == NULL) || !log_file->is_open || (format == NULL))
    {
        (void)SD_Log_SetError(FR_NOT_READY);
        return -1;
    }

    va_start(arguments, format);
    result = SD_Log_VPrintf(&log_file->file, format, arguments);
    va_end(arguments);

    return result;
}

FRESULT SD_Log_FileFlush(SD_LogFile_t* log_file)
{
    if ((log_file == NULL) || !log_file->is_open)
    {
        return SD_Log_SetError(FR_NOT_READY);
    }

    return SD_Log_SetError(f_sync(&log_file->file));
}

FRESULT SD_Log_FileClose(SD_LogFile_t* log_file)
{
    FRESULT result;

    if ((log_file == NULL) || !log_file->is_open)
    {
        return SD_Log_SetError(FR_NOT_READY);
    }

    result = f_sync(&log_file->file);
    if (f_close(&log_file->file) != FR_OK)
    {
        result = FR_DISK_ERR;
    }
    log_file->is_open = false;

    return SD_Log_SetError(result);
}

FRESULT SD_Log_Read(FSIZE_t offset, void* buffer, UINT length, UINT* read)
{
    FRESULT result;
    FRESULT restore_result;
    FSIZE_t file_size;
    UINT local_read = 0U;

    if (read != NULL)
    {
        *read = 0U;
    }

    if (!s_log_open)
    {
        return SD_Log_SetError(FR_NOT_READY);
    }

    if ((buffer == NULL) || (length == 0U))
    {
        return SD_Log_SetError(FR_INVALID_PARAMETER);
    }

    result = f_sync(&s_log_file);
    file_size = f_size(&s_log_file);
    if ((result == FR_OK) && (offset > file_size))
    {
        result = FR_INVALID_PARAMETER;
    }
    if (result == FR_OK)
    {
        result = f_lseek(&s_log_file, offset);
    }
    if (result == FR_OK)
    {
        result = f_read(&s_log_file, buffer, length, &local_read);
    }

    restore_result = f_lseek(&s_log_file, file_size);
    if (result == FR_OK)
    {
        result = restore_result;
    }

    if (read != NULL)
    {
        *read = local_read;
    }

    return SD_Log_SetError(result);
}

FRESULT SD_Log_Flush(void)
{
    if (!s_log_open)
    {
        return SD_Log_SetError(FR_NOT_READY);
    }

    return SD_Log_SetError(f_sync(&s_log_file));
}

FRESULT SD_Log_Close(void)
{
    FRESULT result = FR_OK;
    FRESULT close_result;
    FRESULT unmount_result;

    if (s_log_open)
    {
        result = f_sync(&s_log_file);
        close_result = f_close(&s_log_file);
        if (result == FR_OK)
        {
            result = close_result;
        }
        s_log_open = false;
    }

    unmount_result = f_mount(NULL, SDPath, 1U);
    if (result == FR_OK)
    {
        result = unmount_result;
    }

    return SD_Log_SetError(result);
}

bool SD_Log_IsOpen(void)
{
    return s_log_open;
}

FSIZE_t SD_Log_Size(void)
{
    return s_log_open ? f_size(&s_log_file) : 0U;
}

FRESULT SD_Log_LastError(void)
{
    return s_last_error;
}

FRESULT SD_Log_AppStart(void)
{
    FRESULT result;
    uint32_t now;

    s_app_ready = false;
    result = SD_Log_Start(APP_SYSTEM_LOG_FILE_PATH, APP_SYSTEM_LOG_STARTUP_MESSAGE);
    if (result == FR_OK)
    {
        result = SD_Log_FileOpen(&s_line_log, APP_LINETRACK_LOG_FILE_PATH);
    }
    if (result == FR_OK)
    {
        result = SD_Log_FileOpen(&s_wt901_log, APP_WT901_LOG_FILE_PATH);
    }
    if ((result == FR_OK) &&
        (SD_Log_FilePrintf(&s_line_log, "%s", APP_LINETRACK_LOG_STARTUP_MESSAGE) < 0))
    {
        result = SD_Log_LastError();
    }
    if ((result == FR_OK) &&
        (SD_Log_FilePrintf(&s_wt901_log, "%s", APP_WT901_LOG_STARTUP_MESSAGE) < 0))
    {
        result = SD_Log_LastError();
    }
    if ((result == FR_OK) &&
        (SD_Log_Printf("log_sample=%lu ms, log_flush=%lu ms",
                       (unsigned long)APP_LOG_SAMPLE_INTERVAL_MS,
                       (unsigned long)APP_LOG_FLUSH_INTERVAL_MS)
         < 0))
    {
        result = SD_Log_LastError();
    }
    if (result == FR_OK)
    {
        result = SD_Log_Flush();
    }
    if (result == FR_OK)
    {
        result = SD_Log_FileFlush(&s_line_log);
    }
    if (result == FR_OK)
    {
        result = SD_Log_FileFlush(&s_wt901_log);
    }

    s_app_error = result;
    s_app_ready = (result == FR_OK);
    now = HAL_GetTick();
    s_log_sample_tick = now;
    s_log_flush_tick = now;
    s_led_tick = now;

    return SD_Log_SetError(result);
}

FRESULT SD_Log_AppStop(void)
{
    FRESULT result;

    if (!s_app_ready)
    {
        return SD_Log_SetError(FR_NOT_READY);
    }

    if (SD_Log_Printf("Button stop: WT901=off; beep=slow-2") < 0)
    {
        result = SD_Log_LastError();
    }
    else
    {
        result = SD_Log_Flush();
    }
    if (result == FR_OK)
    {
        result = SD_Log_FileFlush(&s_line_log);
    }
    if (result == FR_OK)
    {
        result = SD_Log_FileFlush(&s_wt901_log);
    }

    if (result == FR_OK)
    {
        s_app_ready = false;
    }
    else
    {
        SD_Log_AppSetError(result);
    }

    return SD_Log_SetError(result);
}

int SD_Log_AppEvent(const char* format, ...)
{
    va_list arguments;
    int result;

    if ((!s_app_ready) || (format == NULL))
    {
        (void)SD_Log_SetError(FR_NOT_READY);
        return -1;
    }

    va_start(arguments, format);
    result = SD_Log_VPrintf(&s_log_file, format, arguments);
    va_end(arguments);

    return result;
}

void SD_Log_AppProcess(const LINE_Result_t* line)
{
    uint32_t now = HAL_GetTick();
    uint32_t led_interval = s_app_ready ? APP_LED_LOG_OK_DELAY_MS : APP_LED_LOG_ERROR_DELAY_MS;
    FRESULT result;

    if ((now - s_led_tick) >= led_interval)
    {
        s_led_tick = now;
        HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    }

    if ((!s_app_ready) || (line == NULL))
    {
        return;
    }

    if ((now - s_log_sample_tick) >= APP_LOG_SAMPLE_INTERVAL_MS)
    {
        s_log_sample_tick = now;

        if (SD_Log_FilePrintf(&s_line_log,
                              "raw=0x%02X,count=%u,pos=%.2f,state=%s",
                              (unsigned int)line->raw,
                              (unsigned int)line->black_count,
                              (double)line->position,
                              SD_Log_LineStateToString(line->state))
            < 0)
        {
            SD_Log_AppSetError(SD_Log_LastError());
            return;
        }

        if (SD_Log_FilePrintf(&s_wt901_log,
                              "acc=%.3f,%.3f,%.3f;gyro=%.2f,%.2f,%.2f;angle=%.2f,%.2f,%.2f",
                              (double)g_wt901_accel.x,
                              (double)g_wt901_accel.y,
                              (double)g_wt901_accel.z,
                              (double)g_wt901_gyro.x,
                              (double)g_wt901_gyro.y,
                              (double)g_wt901_gyro.z,
                              (double)g_wt901_angle.roll,
                              (double)g_wt901_angle.pitch,
                              (double)g_wt901_angle.yaw)
            < 0)
        {
            SD_Log_AppSetError(SD_Log_LastError());
            return;
        }
    }

    if ((now - s_log_flush_tick) >= APP_LOG_FLUSH_INTERVAL_MS)
    {
        s_log_flush_tick = now;
        result = SD_Log_Flush();
        if (result == FR_OK)
        {
            result = SD_Log_FileFlush(&s_line_log);
        }
        if (result == FR_OK)
        {
            result = SD_Log_FileFlush(&s_wt901_log);
        }
        if (result != FR_OK)
        {
            SD_Log_AppSetError(result);
        }
    }
}

bool SD_Log_AppIsReady(void)
{
    return s_app_ready;
}

FRESULT SD_Log_AppLastError(void)
{
    return s_app_error;
}
