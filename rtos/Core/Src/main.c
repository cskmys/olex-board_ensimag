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
#include<stdio.h>
#include<stdarg.h>
#include<string.h>
#include "task.h"
#include "sensor.h"
#include "blespi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
		.name = "defaultTask",
		.stack_mem = &defaultTaskBuffer[0],
		.stack_size = sizeof(defaultTaskBuffer),
		.cb_mem = &defaultTaskControlBlock,
		.cb_size = sizeof(defaultTaskControlBlock),
		.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for blueLedTask */
osThreadId_t blueLedTaskHandle;
uint32_t blueLedTaskBuffer[ 128 ];
osStaticThreadDef_t blueLedTaskControlBlock;
const osThreadAttr_t blueLedTask_attributes = {
		.name = "blueLedTask",
		.stack_mem = &blueLedTaskBuffer[0],
		.stack_size = sizeof(blueLedTaskBuffer),
		.cb_mem = &blueLedTaskControlBlock,
		.cb_size = sizeof(blueLedTaskControlBlock),
		.priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for pushButtonTask */
osThreadId_t pushButtonTaskHandle;
uint32_t pushButtonTaskBuffer[ 128 ];
osStaticThreadDef_t pushButtonTaskControlBlock;
const osThreadAttr_t pushButtonTask_attributes = {
		.name = "pushButtonTask",
		.stack_mem = &pushButtonTaskBuffer[0],
		.stack_size = sizeof(pushButtonTaskBuffer),
		.cb_mem = &pushButtonTaskControlBlock,
		.cb_size = sizeof(pushButtonTaskControlBlock),
		.priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
uint32_t sensorTaskBuffer[ 256 ];
osStaticThreadDef_t sensorTaskControlBlock;
const osThreadAttr_t sensorTask_attributes = {
		.name = "sensorTask",
		.stack_mem = &sensorTaskBuffer[0],
		.stack_size = sizeof(sensorTaskBuffer),
		.cb_mem = &sensorTaskControlBlock,
		.cb_size = sizeof(sensorTaskControlBlock),
		.priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for bleTask */
osThreadId_t bleTaskHandle;
uint32_t bleTaskBuffer[ 256 ];
osStaticThreadDef_t bleTaskControlBlock;
const osThreadAttr_t bleTask_attributes = {
		.name = "bleTask",
		.stack_mem = &bleTaskBuffer[0],
		.stack_size = sizeof(bleTaskBuffer),
		.cb_mem = &bleTaskControlBlock,
		.cb_size = sizeof(bleTaskControlBlock),
		.priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void StartBlueLedTask(void *argument);
void StartPushButtonTask(void *argument);
void StartSensorTask(void *argument);
void StartBleTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
void dbgPrintf(const char* fmt, ...) {
	vTaskSuspendAll();
	va_list args;
	va_start(args, fmt);
	char printStr[32] = {0};
	vsnprintf(printStr, sizeof(printStr), fmt, args);
	va_end(args);
	if( HAL_UART_Transmit(&huart1, (uint8_t*)printStr, strlen(printStr), 100) != HAL_OK ){
		xTaskResumeAll();
		Error_Handler();
	}
	xTaskResumeAll();
}

void dbgPrintBuff(uint8_t *buff, uint8_t nbEle){
	vTaskSuspendAll();
	for(int i = 0; i < nbEle; ++i){
		dbgPrintf("0x%02x ");
	}
	dbgPrintf("\n");
	xTaskResumeAll();
}

void printSensorCfg( void ){
	int accCfg = getAccCfg();
	int gyroCfg = getGyroCfg();
	dbgPrintf("CfgAG: 0x%02x 0x%02x\n", accCfg, gyroCfg);
}

void printAccErr( void ){
	Reading accErr;
	getAccErr(&accErr);
	dbgPrintf("AEXY: %d %d\n", (int)accErr.x, (int)accErr.y);
}

void printGyroErr( void ){
	Reading gyroErr;
	getGyroErr(&gyroErr);
	dbgPrintf("GEXYZ: %d %d %d\n", (int)gyroErr.x, (int)gyroErr.y, (int)gyroErr.z);
}

void printSensorErr( void ) {
	// PLACE SENSOR FLAT INORDER TO GET PROPER VALUES
	dbgPrintf("ERR CALC:KEEP SENS FLAT\n");
	// The error values used in the equations are calculated here
	printAccErr();
	printGyroErr();
}
Reading rpy;
void printRollPitchYaw( void ) {
	getRollPitchYaw(&rpy);
	dbgPrintf("RPY: %d %d %d\n", (int)rpy.x, (int)rpy.y, (int)rpy.z);
}
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
	MX_USART1_UART_Init();
	MX_SPI1_Init();
	MX_I2C1_Init();
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

	/* creation of blueLedTask */
	blueLedTaskHandle = osThreadNew(StartBlueLedTask, NULL, &blueLedTask_attributes);

	/* creation of pushButtonTask */
	pushButtonTaskHandle = osThreadNew(StartPushButtonTask, NULL, &pushButtonTask_attributes);

	/* creation of sensorTask */
	sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);

	/* creation of bleTask */
	bleTaskHandle = osThreadNew(StartBleTask, NULL, &bleTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		Error_Handler();
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

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
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, blueLed_Pin|greenLed_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : blueButton_Pin */
	GPIO_InitStruct.Pin = blueButton_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(blueButton_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : blueLed_Pin greenLed_Pin */
	GPIO_InitStruct.Pin = blueLed_Pin|greenLed_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

/* USER CODE BEGIN Header_StartBlueLedTask */
/**
 * @brief Function implementing the blueLedTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBlueLedTask */
void StartBlueLedTask(void *argument)
{
	/* USER CODE BEGIN StartBlueLedTask */
	/* Infinite loop */
	for(;;)
	{
		HAL_GPIO_TogglePin(blueLed_GPIO_Port, blueLed_Pin);
		osDelay(1000);
	}
	/* USER CODE END StartBlueLedTask */
}

/* USER CODE BEGIN Header_StartPushButtonTask */
/**
 * @brief Function implementing the pushButtonTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartPushButtonTask */
void StartPushButtonTask(void *argument)
{
	/* USER CODE BEGIN StartPushButtonTask */
	/* Infinite loop */
	for(;;)
	{
		HAL_GPIO_WritePin(greenLed_GPIO_Port, greenLed_Pin, HAL_GPIO_ReadPin(blueButton_GPIO_Port, blueButton_Pin));
		osDelay(100);
	}
	/* USER CODE END StartPushButtonTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
 * @brief Function implementing the sensorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
	/* USER CODE BEGIN StartSensorTask */
	initSensor();
	//	printSensorCfg();
	//	printSensorErr();
	/* Infinite loop */
	for(;;)
	{
		printRollPitchYaw();
		osDelay(50);
	}
	/* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartBleTask */
/**
 * @brief Function implementing the bleTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartBleTask */
void StartBleTask(void *argument)
{
	/* USER CODE BEGIN StartBleTask */
	bleRst();
	osDelay(1000);
	/* Infinite loop */
	for(;;)
	{
//		uint8_t pTxData[3][20] = {
//				{ 0x10, 0x01, 0x0a, 0x10, 0x54, 0x68, 0x65, 0x20, 0x71, 0x75, 0x69, 0x63, 0x6b, 0x20, 0x62, 0x72, 0x6f, 0x77, 0x6e, 0x20},
//				{ 0x10, 0x01, 0x0a, 0x10, 0x66, 0x6f, 0x78, 0x20, 0x6a, 0x75, 0x6d, 0x70, 0x73, 0x20, 0x6f, 0x76, 0x65, 0x72, 0x20, 0x74},
//				{ 0x10, 0x01, 0x0a, 0xc, 0x68, 0x65, 0x20, 0x6c, 0x61, 0x7a, 0x79, 0x20, 0x64, 0x6f, 0x67, 0x0a, 0xff, 0xff, 0xff, 0xff}
//		};
//		uint8_t eRxResp[20] = { 0x20, 0x01, 0x0a, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
//		for(int i = 0; i < 3; ++i){
//			sendBlePkt(&pTxData[i][0], eRxResp);
//			osDelay(50);
//		}
		blePrintf("RPY: %d %d %d\n", (int)rpy.x, (int)rpy.y, (int)rpy.z);
		osDelay(500);
	}
	/* USER CODE END StartBleTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM2 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM2) {
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
	HAL_GPIO_WritePin(greenLed_GPIO_Port, greenLed_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(blueLed_GPIO_Port, blueLed_Pin, GPIO_PIN_SET);
	dbgPrintf("fuck\n");
	while(1);
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
