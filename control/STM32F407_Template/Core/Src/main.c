/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fatfs.h"
#include "gpio.h"
#include "rtc.h"
#include "sdio.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "linetrack.h"
#include "sd_config.h"
#include "sd_log.h"
#include "wt901.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Inspect these variables in the debugger after startup. */
volatile FRESULT g_sd_result = FR_NOT_READY;
volatile uint8_t g_sd_test_ok = 0U;
static SD_LogFile_t s_wt901_log;
static uint32_t s_log_sample_tick = 0U;
static uint32_t s_log_flush_tick = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
LINE_Result_t line;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_DMA_Init();
    MX_GPIO_Init();
    MX_SDIO_SD_Init();
    MX_RTC_Init();
    MX_USART1_UART_Init();
    MX_FATFS_Init();
    /* USER CODE BEGIN 2 */

    g_sd_result = SD_Log_Start(APP_LOG_FILE_PATH, APP_LOG_STARTUP_MESSAGE);
    if (g_sd_result == FR_OK)
    {
        g_sd_result = SD_Log_FileOpen(&s_wt901_log, APP_WT901_LOG_FILE_PATH);
    }
    if ((g_sd_result == FR_OK) && (SD_Log_FilePrintf(&s_wt901_log, "%s", APP_WT901_LOG_STARTUP_MESSAGE) < 0))
    {
        g_sd_result = SD_Log_LastError();
    }
    if (g_sd_result == FR_OK)
    {
        g_sd_result = SD_Log_FileFlush(&s_wt901_log);
    }
    g_sd_test_ok = (g_sd_result == FR_OK) ? 1U : 0U;

    WT901_Init();
    WT901_StartReceive();

    s_log_sample_tick = HAL_GetTick();
    s_log_flush_tick = s_log_sample_tick;

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        // HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
        // HAL_Delay(g_sd_test_ok != 0U ? APP_LED_LOG_OK_DELAY_MS : APP_LED_LOG_ERROR_DELAY_MS);
        line = LINE_Process();
        uint32_t now = HAL_GetTick();

        (void)WT901_AnalyzeData();

        if ((g_sd_test_ok != 0U) && ((now - s_log_sample_tick) >= APP_LOG_SAMPLE_INTERVAL_MS))
        {
            s_log_sample_tick = now;

            if (SD_Log_Printf("raw=0x%02X,count=%u,pos=%.2f,state=%u",
                              (unsigned int)line.raw,
                              (unsigned int)line.black_count,
                              (double)line.position,
                              (unsigned int)line.state)
                < 0)
            {
                g_sd_result = SD_Log_LastError();
                g_sd_test_ok = 0U;
            }

            if ((g_sd_test_ok != 0U) && (SD_Log_FilePrintf(&s_wt901_log, "acc=%.3f,%.3f,%.3f;gyro=%.2f,%.2f,%.2f;angle=%.2f,%.2f,%.2f", (double)g_wt901_accel.x, (double)g_wt901_accel.y, (double)g_wt901_accel.z, (double)g_wt901_gyro.x, (double)g_wt901_gyro.y, (double)g_wt901_gyro.z, (double)g_wt901_angle.roll, (double)g_wt901_angle.pitch, (double)g_wt901_angle.yaw) < 0))
            {
                g_sd_result = SD_Log_LastError();
                g_sd_test_ok = 0U;
            }
        }

        if ((g_sd_test_ok != 0U) && ((now - s_log_flush_tick) >= APP_LOG_FLUSH_INTERVAL_MS))
        {
            s_log_flush_tick = now;
            g_sd_result = SD_Log_Flush();
            if (g_sd_result == FR_OK)
            {
                g_sd_result = SD_Log_FileFlush(&s_wt901_log);
            }
            g_sd_test_ok = (g_sd_result == FR_OK) ? 1U : 0U;
        }

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Configure the main internal regulator output voltage
  */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
  */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
