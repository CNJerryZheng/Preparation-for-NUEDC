/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sd_diskio.c
  * @brief   SD Disk I/O driver
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Note: DMA-enabled SDIO disk I/O path. */

/* USER CODE BEGIN firstSection */
/* can be used to modify / undefine following code or add new definitions */
/* USER CODE END firstSection*/

/* Includes ------------------------------------------------------------------*/
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include "config.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* use the default SD timout as defined in the platform BSP driver*/
#if defined(SDMMC_DATATIMEOUT)
#define SD_TIMEOUT SDMMC_DATATIMEOUT
#elif defined(SD_DATATIMEOUT)
#define SD_TIMEOUT SD_DATATIMEOUT
#else
#define SD_TIMEOUT 30 * 1000
#endif

#define SD_DEFAULT_BLOCK_SIZE 512
#define SD_DMA_TIMEOUT_MS     APP_TF_DMA_TIMEOUT_MS

/*
 * Depending on the use case, the SD card initialization could be done at the
 * application level: if it is the case define the flag below to disable
 * the BSP_SD_Init() call in the SD_Initialize() and add a call to
 * BSP_SD_Init() elsewhere in the application.
 */
/* USER CODE BEGIN disableSDInit */
/* #define DISABLE_SD_INIT */
/* USER CODE END disableSDInit */

/* Private variables ---------------------------------------------------------*/
/* DMA works on words. FatFs may occasionally provide an unaligned buffer. */
__ALIGN_BEGIN static uint8_t ScratchBuffer[SD_DEFAULT_BLOCK_SIZE] __ALIGN_END;

/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;
static volatile UINT ReadStatus = 0U;
static volatile UINT WriteStatus = 0U;

/* Private function prototypes -----------------------------------------------*/
static DSTATUS SD_CheckStatus(BYTE lun);
static int SD_WaitCardReady(uint32_t timeout);
static int SD_WaitDma(volatile UINT *status, uint32_t timeout);
DSTATUS SD_initialize (BYTE);
DSTATUS SD_status (BYTE);
DRESULT SD_read (BYTE, BYTE*, DWORD, UINT);
#if _USE_WRITE == 1
DRESULT SD_write (BYTE, const BYTE*, DWORD, UINT);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
DRESULT SD_ioctl (BYTE, BYTE, void*);
#endif  /* _USE_IOCTL == 1 */

const Diskio_drvTypeDef  SD_Driver =
{
  SD_initialize,
  SD_status,
  SD_read,
#if  _USE_WRITE == 1
  SD_write,
#endif /* _USE_WRITE == 1 */

#if  _USE_IOCTL == 1
  SD_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* USER CODE BEGIN beforeFunctionSection */
/* can be used to modify / undefine following code or add new code */
/* USER CODE END beforeFunctionSection */

/* Private functions ---------------------------------------------------------*/

static int SD_WaitCardReady(uint32_t timeout)
{
  uint32_t start = HAL_GetTick();

  while ((HAL_GetTick() - start) < timeout)
  {
    if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
    {
      return 0;
    }
  }

  return -1;
}

static int SD_WaitDma(volatile UINT *status, uint32_t timeout)
{
  uint32_t start = HAL_GetTick();

  while ((*status == 0U) && ((HAL_GetTick() - start) < timeout))
  {
  }

  return (*status == 1U) ? 0 : -1;
}

static DSTATUS SD_CheckStatus(BYTE lun)
{
  Stat = STA_NOINIT;

  if(BSP_SD_GetCardState() == MSD_OK)
  {
    Stat &= ~STA_NOINIT;
  }

  return Stat;
}

/**
  * @brief  Initializes a Drive
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_initialize(BYTE lun)
{
Stat = STA_NOINIT;

#if !defined(DISABLE_SD_INIT)

  if(BSP_SD_Init() == MSD_OK)
  {
    Stat = SD_CheckStatus(lun);
  }

#else
  Stat = SD_CheckStatus(lun);
#endif

  return Stat;
}

/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS SD_status(BYTE lun)
{
  return SD_CheckStatus(lun);
}

/* USER CODE BEGIN beforeReadSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeReadSection */
/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */

DRESULT SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
  UINT index;

  if ((buff == NULL) || (count == 0U) ||
      (SD_WaitCardReady(SD_DMA_TIMEOUT_MS) != 0))
  {
    return RES_ERROR;
  }

  if (((uint32_t)buff & 0x3U) == 0U)
  {
    ReadStatus = 0U;
    if ((BSP_SD_ReadBlocks_DMA((uint32_t *)buff, (uint32_t)sector, count) != MSD_OK) ||
        (SD_WaitDma(&ReadStatus, SD_DMA_TIMEOUT_MS) != 0) ||
        (SD_WaitCardReady(SD_DMA_TIMEOUT_MS) != 0))
    {
      return RES_ERROR;
    }
    return RES_OK;
  }

  for (index = 0U; index < count; ++index)
  {
    ReadStatus = 0U;
    if ((BSP_SD_ReadBlocks_DMA((uint32_t *)ScratchBuffer,
                               (uint32_t)sector + index, 1U) != MSD_OK) ||
        (SD_WaitDma(&ReadStatus, SD_DMA_TIMEOUT_MS) != 0) ||
        (SD_WaitCardReady(SD_DMA_TIMEOUT_MS) != 0))
    {
      return RES_ERROR;
    }
    memcpy(buff + (index * SD_DEFAULT_BLOCK_SIZE), ScratchBuffer,
           SD_DEFAULT_BLOCK_SIZE);
  }

  return RES_OK;
}

/* USER CODE BEGIN beforeWriteSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeWriteSection */
/**
  * @brief  Writes Sector(s)
  * @param  lun : not used
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1

DRESULT SD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
  UINT index;

  if ((buff == NULL) || (count == 0U) ||
      (SD_WaitCardReady(SD_DMA_TIMEOUT_MS) != 0))
  {
    return RES_ERROR;
  }

  if (((uint32_t)buff & 0x3U) == 0U)
  {
    WriteStatus = 0U;
    if ((BSP_SD_WriteBlocks_DMA((uint32_t *)buff, (uint32_t)sector, count) != MSD_OK) ||
        (SD_WaitDma(&WriteStatus, SD_DMA_TIMEOUT_MS) != 0) ||
        (SD_WaitCardReady(SD_DMA_TIMEOUT_MS) != 0))
    {
      return RES_ERROR;
    }
    return RES_OK;
  }

  for (index = 0U; index < count; ++index)
  {
    memcpy(ScratchBuffer, buff + (index * SD_DEFAULT_BLOCK_SIZE),
           SD_DEFAULT_BLOCK_SIZE);
    WriteStatus = 0U;
    if ((BSP_SD_WriteBlocks_DMA((uint32_t *)ScratchBuffer,
                                (uint32_t)sector + index, 1U) != MSD_OK) ||
        (SD_WaitDma(&WriteStatus, SD_DMA_TIMEOUT_MS) != 0) ||
        (SD_WaitCardReady(SD_DMA_TIMEOUT_MS) != 0))
    {
      return RES_ERROR;
    }
  }

  return RES_OK;
}
#endif /* _USE_WRITE == 1 */

/* USER CODE BEGIN beforeIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeIoctlSection */
/**
  * @brief  I/O control operation
  * @param  lun : not used
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff)
{
  DRESULT res = RES_ERROR;
  BSP_SD_CardInfo CardInfo;

  if (Stat & STA_NOINIT) return RES_NOTRDY;

  switch (cmd)
  {
  /* Make sure that no pending write process */
  case CTRL_SYNC :
    res = (SD_WaitCardReady(SD_DMA_TIMEOUT_MS) == 0) ? RES_OK : RES_ERROR;
    break;

  /* Get number of sectors on the disk (DWORD) */
  case GET_SECTOR_COUNT :
    BSP_SD_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockNbr;
    res = RES_OK;
    break;

  /* Get R/W sector size (WORD) */
  case GET_SECTOR_SIZE :
    BSP_SD_GetCardInfo(&CardInfo);
    *(WORD*)buff = CardInfo.LogBlockSize;
    res = RES_OK;
    break;

  /* Get erase block size in unit of sector (DWORD) */
  case GET_BLOCK_SIZE :
    BSP_SD_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
    res = RES_OK;
    break;

  default:
    res = RES_PARERR;
  }

  return res;
}
#endif /* _USE_IOCTL == 1 */

/* USER CODE BEGIN afterIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END afterIoctlSection */

/* USER CODE BEGIN lastSection */

void BSP_SD_WriteCpltCallback(void)
{
  WriteStatus = 1U;
}

void BSP_SD_ReadCpltCallback(void)
{
  ReadStatus = 1U;
}

void BSP_SD_AbortCallback(void)
{
  ReadStatus = 2U;
  WriteStatus = 2U;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *sdHandle)
{
  (void)sdHandle;
  ReadStatus = 2U;
  WriteStatus = 2U;
}

/* USER CODE END lastSection */
