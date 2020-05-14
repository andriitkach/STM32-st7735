/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "semphr.h"
#include <stdio.h>
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
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */
SemaphoreHandle_t sDisplaySPI;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

const uint32_t delay_ms = 63;
static int8_t out_x = 0;
static int8_t out_y = 0;
static int8_t out_z = 0;
char str_x[4];
char str_y[4];
char str_z[4];

void vTaskDisplayTest(void * displayTestParam) {
	const uint32_t delay_ticks = delay_ms / portTICK_PERIOD_MS;
    ST7735_Init();
    LIS302DL_Init();

	ST7735_AddFill(ST7735_BLACK);

	ST7735_AddHorLine(38, 0, ST7735_WIDTH-1, 1, ST7735_YELLOW, SOLID);
	ST7735_AddHorLine(77, 0, ST7735_WIDTH-1, 1, ST7735_YELLOW, SOLID);
	ST7735_AddHorLine(116, 0, ST7735_WIDTH-1, 1, ST7735_YELLOW, SOLID);

	ST7735_AddVerLine(31, 0, 127, 1, ST7735_YELLOW, SOLID);

	ST7735_AddString(6, 2, "X", Font_16x26, ST7735_WHITE, ST7735_BLACK);
	ST7735_AddString(1, 27, str_x, Font_7x10, ST7735_WHITE, ST7735_BLACK);
	ST7735_AddString(6, 41, "Y", Font_16x26, ST7735_WHITE, ST7735_BLACK);
	ST7735_AddString(1, 66, str_y, Font_7x10, ST7735_WHITE, ST7735_BLACK);
	ST7735_AddString(6, 80, "Z", Font_16x26, ST7735_WHITE, ST7735_BLACK);
	ST7735_AddString(1, 105, str_z, Font_7x10, ST7735_WHITE, ST7735_BLACK);
	ST7735_AddString(1, 118, "Time", Font_7x10, ST7735_WHITE, ST7735_BLACK);

	ChannelData_t data_x;
	ChannelData_t data_y;
	ChannelData_t data_z;
	data_x.head = 0;
	data_x.cnt = 0;
	data_y.head = 0;
	data_y.cnt = 0;
	data_z.head = 0;
	data_z.cnt = 0;

	while(1) {

		out_x = LIS302DL_GetX();
		out_y = LIS302DL_GetY();
		out_z = LIS302DL_GetZ();
		pushChannelData(out_x, &data_x);
		pushChannelData(out_y, &data_y);
		pushChannelData(out_z, &data_z);
		sprintf(str_x, "%d", out_x);
		sprintf(str_y, "%d", out_y);
		sprintf(str_z, "%d", out_z);
		printChannelData(32, 19, &data_x, ST7735_WHITE);
		printChannelData(32, 58, &data_y, ST7735_WHITE);
		printChannelData(32, 96, &data_z, ST7735_WHITE);

		ST7735_AddHorLine(19, 31, ST7735_WIDTH-1, 1, ST7735_GRAY, DOTTED);
		ST7735_AddHorLine(58, 31, ST7735_WIDTH-1, 1, ST7735_GRAY, DOTTED);
		ST7735_AddHorLine(96, 31, ST7735_WIDTH-1, 1, ST7735_GRAY, DOTTED);

		ST7735_AddVerLine(ST7735_WIDTH - 32, 0, 116, 1, ST7735_GRAY, DOTTED);
		ST7735_AddVerLine(ST7735_WIDTH - 64, 0, 116, 1, ST7735_GRAY, DOTTED);
		ST7735_AddVerLine(ST7735_WIDTH - 96, 0, 116, 1, ST7735_GRAY, DOTTED);

		ST7735_AddString(1, 27, str_x, Font_7x10, ST7735_WHITE, ST7735_BLACK);
		ST7735_AddString(1, 66, str_y, Font_7x10, ST7735_WHITE, ST7735_BLACK);
		ST7735_AddString(1, 105, str_z, Font_7x10, ST7735_WHITE, ST7735_BLACK);
		ST7735_Refresh();
		vTaskDelay(delay_ticks);
		ST7735_AddRectangle(1, 27, 30, 37, ST7735_BLACK);
		ST7735_AddRectangle(1, 66, 30, 76, ST7735_BLACK);
		ST7735_AddRectangle(1, 105, 30, 115, ST7735_BLACK);

		ST7735_AddRectangle(32, 1, 159, 37, ST7735_BLACK);
		ST7735_AddRectangle(32, 40, 159, 75, ST7735_BLACK);
		ST7735_AddRectangle(32, 78, 159, 115, ST7735_BLACK);


	}
}

//void vTaskScreenRefresher(void * pRefresher) {
//	uint16_t refreshPeriod_ms = 123;
//	uint8_t refreshPeriod_ticks = refreshPeriod_ms / portTICK_PERIOD_MS;
//	while(1) {
//		vTaskDelay(refreshPeriod_ticks);
//		ST7735_Refresh();
//
//	}
//}

//void vTaskMEMS(void * pMEMS) {
//	const uint32_t delay_ticks = 123 / portTICK_PERIOD_MS;
//	uint8_t deviceId = 0;
//	LIS302DL_Init();
//	while(1) {
//		out_x = LIS302DL_GetX();
//		out_y = LIS302DL_GetY();
//		out_z = LIS302DL_GetZ();
//		sprintf(str_x, "%d", out_x);
//		sprintf(str_y, "%d", out_y);
//		sprintf(str_z, "%d", out_z);
//		vTaskDelay(delay_ticks);
//		deviceId = 0;
//	}
//}


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
  sDisplaySPI = xSemaphoreCreateBinary();
  xSemaphoreGive(sDisplaySPI);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_SPI1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
//  BaseType_t xTaskCreate(	TaskFunction_t pxTaskCode,
//  							const char * const pcName,	/*lint !e971 Unqualified char types are allowed for strings and single characters only. */
//  							const configSTACK_DEPTH_TYPE usStackDepth,
//  							void * const pvParameters,
//  							UBaseType_t uxPriority,
//  							TaskHandle_t * const pxCreatedTask ) PRIVILEGED_FUNCTION;

  xTaskCreate(
		  vTaskDisplayTest,
		  "DisplayTest",
		  (uint16_t)2048,
		  NULL,
		  2,
		  NULL
  );
//  xTaskCreate(
//		  vTaskScreenRefresher,
//		  "ScreenRefresher",
//		  configMINIMAL_STACK_SIZE,
//		  NULL,
//		  20,
//		  NULL
//  );

//  xTaskCreate(
//  		  vTaskMEMS,
//  		  "MEMS_measurement",
//		  configMINIMAL_STACK_SIZE,
//  		  NULL,
//  		  2,
//  		  NULL
//    );

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  uint32_t cnt = 0;
//  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  	// Set CS (no transmission)
//  HAL_GPIO_WritePin(GPIOD,  GPIO_PIN_1, GPIO_PIN_SET); 		// No reset
//  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  while (1)
  {
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* SPI3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SPI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(SPI3_IRQn);
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	ST7735_Unselect();
	xSemaphoreGiveFromISR(sDisplaySPI, NULL);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
