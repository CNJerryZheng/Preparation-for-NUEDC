#include "sd_log.h"

#include "fatfs.h"
#include "rtc.h"
#include "config.h"

#include <stdarg.h>
#include <stdio.h>

static FIL s_log_file;
static bool s_log_open;
static FRESULT s_last_error = FR_NOT_READY;

static FRESULT SD_Log_SetError(FRESULT result)
{
  s_last_error = result;
  return result;
}

FRESULT SD_Log_Init(const char *path)
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

FRESULT SD_Log_Start(const char *path, const char *startup_message)
{
  FRESULT result;

  /* Allow the card supply and internal controller to become stable. */
  HAL_Delay(APP_TF_STARTUP_DELAY_MS);

  result = SD_Log_Init(path);
  if ((result == FR_OK) && (startup_message != NULL) &&
      (startup_message[0] != '\0'))
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

FRESULT SD_Log_Write(const void *data, UINT length, UINT *written)
{
  UINT local_written = 0U;
  FRESULT result;

  if (written != NULL)
  {
    *written = 0U;
  }

  if (!s_log_open)
  {
    return SD_Log_SetError(FR_NOT_READY);
  }

  if ((data == NULL) || (length == 0U))
  {
    return SD_Log_SetError(FR_INVALID_PARAMETER);
  }

  result = f_lseek(&s_log_file, f_size(&s_log_file));
  if (result == FR_OK)
  {
    result = f_write(&s_log_file, data, length, &local_written);
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

int SD_Log_Printf(const char *format, ...)
{
  RTC_TimeTypeDef time = {0};
  RTC_DateTypeDef date = {0};
  char line[APP_LOG_LINE_SIZE];
  va_list arguments;
  int prefix_length;
  int body_length;
  UINT total_length;
  UINT written;

  if ((!s_log_open) || (format == NULL))
  {
    (void)SD_Log_SetError(FR_NOT_READY);
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
                           (unsigned int)date.Year,
                           (unsigned int)date.Month,
                           (unsigned int)date.Date,
                           (unsigned int)time.Hours,
                           (unsigned int)time.Minutes,
                           (unsigned int)time.Seconds);
  if ((prefix_length < 0) || ((UINT)prefix_length >= sizeof(line)))
  {
    (void)SD_Log_SetError(FR_INVALID_PARAMETER);
    return -1;
  }

  va_start(arguments, format);
  body_length = vsnprintf(&line[prefix_length],
                          sizeof(line) - (UINT)prefix_length - 2U,
                          format, arguments);
  va_end(arguments);

  if ((body_length < 0) ||
      ((UINT)body_length >= (sizeof(line) - (UINT)prefix_length - 2U)))
  {
    (void)SD_Log_SetError(FR_INVALID_PARAMETER);
    return -1;
  }

  total_length = (UINT)prefix_length + (UINT)body_length;
  line[total_length++] = '\r';
  line[total_length++] = '\n';

  if (SD_Log_Write(line, total_length, &written) != FR_OK)
  {
    return -1;
  }

  return (int)written;
}

FRESULT SD_Log_Read(FSIZE_t offset, void *buffer, UINT length, UINT *read)
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
