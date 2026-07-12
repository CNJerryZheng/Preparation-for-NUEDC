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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_RX_BUFFER_SIZE 32
#define UART_TIMEOUT 200
#define DEAD_ZONE 10
#define MIN_STEP_FREQ 150
#define MAX_STEP_FREQ 1500
#define STEP_KP 6

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t uart2_rx_byte = 0;

char uart2_build_buffer[UART_RX_BUFFER_SIZE];
char uart2_line_buffer[UART_RX_BUFFER_SIZE];

volatile uint16_t uart2_rx_index = 0;
volatile uint8_t uart2_line_ready = 0;

volatile int32_t vision_dx = 0;
volatile int32_t vision_dy = 0;
volatile uint8_t target_found = 0;

uint32_t last_uart_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Stepper_Set(TIM_HandleTypeDef *htim,uint32_t channel,GPIO_TypeDef *dir_port,uint16_t dir_pin,int32_t error);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(EN1_GPIO_Port,EN1_Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(EN2_GPIO_Port,EN2_Pin,GPIO_PIN_SET);

  HAL_GPIO_WritePin(PAN_DIR_GPIO_Port,PAN_DIR_Pin,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TILT_DIR_GPIO_Port,TILT_DIR_Pin,GPIO_PIN_RESET);

  HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
  HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);

  HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);

  if(HAL_UART_Receive_DMA(&huart2,&uart2_rx_byte,1) != HAL_OK)Error_Handler();

  last_uart_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
    if(uart2_line_ready == 1){
      int received_dx = 0;
      int received_dy = 0;

      last_uart_tick = HAL_GetTick();

      if(strcmp(uart2_line_buffer,"L") == 0){
        target_found = 0;
        vision_dx = 0;
        vision_dy = 0;
      }
      else if(sscanf(uart2_line_buffer,"%d,%d",&received_dx,&received_dy) == 2){
        vision_dx = received_dx;
        vision_dy = received_dy;
        target_found = 1;
      }
      else{
        target_found = 0;
        vision_dx = 0;
        vision_dy = 0;
      }

      if(target_found == 1){
        Stepper_Set(&htim4,TIM_CHANNEL_3,PAN_DIR_GPIO_Port,PAN_DIR_Pin,vision_dx);
        Stepper_Set(&htim1,TIM_CHANNEL_4,TILT_DIR_GPIO_Port,TILT_DIR_Pin,vision_dy);
        HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
      }
      else{
        HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
        HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);
        HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
      }

      uart2_line_ready = 0;
    }

    if(target_found == 1 && HAL_GetTick() - last_uart_tick > UART_TIMEOUT){
      target_found = 0;
      vision_dx = 0;
      vision_dy = 0;

      HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
      HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);

      HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK){
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_LATENCY_2) != HAL_OK){
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if(huart->Instance == USART2){
    uint8_t received_char = uart2_rx_byte;

    if(received_char == '\n'){
      uart2_build_buffer[uart2_rx_index] = '\0';

      if(uart2_line_ready == 0){
        memcpy(uart2_line_buffer,uart2_build_buffer,uart2_rx_index + 1);
        uart2_line_ready = 1;
      }

      uart2_rx_index = 0;
    }
    else if(received_char != '\r'){
      if(uart2_rx_index < UART_RX_BUFFER_SIZE - 1){
        uart2_build_buffer[uart2_rx_index] = (char)received_char;
        uart2_rx_index++;
      }
      else{
        uart2_rx_index = 0;
      }
    }

    HAL_UART_Receive_DMA(&huart2,&uart2_rx_byte,1);
  }
}

void Stepper_Set(TIM_HandleTypeDef *htim,uint32_t channel,GPIO_TypeDef *dir_port,uint16_t dir_pin,int32_t error){
  uint32_t abs_error;
  uint32_t frequency;
  uint32_t arr;

  if(error >= -DEAD_ZONE && error <= DEAD_ZONE){
    HAL_TIM_PWM_Stop(htim,channel);
    return;
  }

  if(error > 0){
    HAL_GPIO_WritePin(dir_port,dir_pin,GPIO_PIN_SET);
    abs_error = error;
  }
  else{
    HAL_GPIO_WritePin(dir_port,dir_pin,GPIO_PIN_RESET);
    abs_error = -error;
  }

  frequency = abs_error * STEP_KP;

  if(frequency < MIN_STEP_FREQ)frequency = MIN_STEP_FREQ;
  if(frequency > MAX_STEP_FREQ)frequency = MAX_STEP_FREQ;

  arr = 1000000U / frequency - 1U;

  __HAL_TIM_SET_AUTORELOAD(htim,arr);
  __HAL_TIM_SET_COMPARE(htim,channel,(arr + 1U) / 2U);
  __HAL_TIM_SET_COUNTER(htim,0U);
  HAL_TIM_GenerateEvent(htim,TIM_EVENTSOURCE_UPDATE);

  HAL_TIM_PWM_Start(htim,channel);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1){}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file name and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file,uint32_t line)
{
  /* USER CODE BEGIN 6 */

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */