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
#include<string.h>
#include<math.h>
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
#define I2C_SLAVE_ADD 0xd0
#define I2C_REG_ADD_SIZ 0x01
#define WHO_AM_I_REG_ADD 0x75
#define WHO_AM_I_REG_VAL 0x68
#define WAKEUP_REG_ADD 0x6b
#define WAKEUP_REG_VAL 0x00
#define ACC_REG_ADD 0x3b
#define GYRO_REG_ADD 0x43
#define ACC_SEN_CFG_REG 0x1c
#define ACC_SEN_FULL_SCALE_CFG_VAL 0x10
#define GYRO_SEN_CFG_REG 0x1b
#define GYRO_SEN_FULL_SCALE_VAL_REG 0x10

// According to the datasheet
#define ACC_2G_DIV_CONST 16384.0 // For +-2g range, divide raw values by 16384
#define ACC_4G_DIV_CONST 8192.0
#define ACC_8G_DIV_CONST 4096.0
#define ACC_16G_DIV_CONST 2048.0

#define GYRO_250DPS_DIV_CONST 131.0 // For 250deg/s range, divide raw value by 131.0
#define GYRO_500DPS_DIV_CONST 65.5
#define GYRO_1000DPS_DIV_CONST 32.8
#define GYRO_2000DPS_DIV_CONST 16.4

#define PI 3.14
#define NB_EXPERIMENTS 200

// yet to be determined properly via experimentation
#define ACC_X_ERR (+1.472944)
#define ACC_Y_ERR (-3.946817)

#define GYRO_X_ERR (-1.105988)
#define GYRO_Y_ERR (-0.717590)
#define GYRO_Z_ERR (-0.481207)

#define DRIFT 0.04 // 4%
#define ACC_DRIFT DRIFT
#define GYRO_DRIFT (1.00 - ACC_DRIFT)
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
/* USER CODE BEGIN 0 */
void cfgSensorFullScale() {
#ifdef FULL_SCALE_CFG_CODE
	// IN CASE YOU UNCOMMENT THIS, MAKE SURE TO CHANGE ALL THE DIVISIBILITY VALUES AS PER THE DATASHEET FOR FULL SCALE
	// Configure Accelerometer Sensitivity - +/- 8g i,e, Full Scale Range. (default +/- 2g)
	// Reg 0x1C -> 0x10:
	uint_8 accFullScaleCfgVal = ACC_SEN_FULL_SCALE_CFG_VAL;
	HAL_I2C_Mem_Write(&hi2c1, I2C_SLAVE_ADD, ACC_SEN_CFG_REG, I2C_REG_ADD_SIZ, &accFullScaleCfgVal, sizeof(accFullScaleCfgVal), 100);

	// Configure Gyro Sensitivity - 1000deg/s i,e, Full Scale Range (default +/- 250deg/s)
	// Reg 0x1B -> 0x10;
	uint_8 gyroFullScaleCfgVal = GYRO_SEN_FULL_SCALE_VAL_REG;
	HAL_I2C_Mem_Write(&hi2c1, I2C_SLAVE_ADD, GYRO_SEN_CFG_REG, I2C_REG_ADD_SIZ, &gyroFullScaleCfgVal, sizeof(gyroFullScaleCfgVal), 100);
	delay(20);
#endif
}

int decode2sCompliment(uint16_t val){
	return (int)( 0x8000 & val ? (int)( 0x7FFF & val ) - 0x8000 : val );
}

void printSensorErr( void ) {
	// The error values used in the equations are calculated here
	// PLACE SENSOR FLAT INORDER TO GET PROPER VALUES
	float accErrorX = 0.0;
	float accErrorY = 0.0;
	for (int i = 0; i < NB_EXPERIMENTS; ++i) {
		uint8_t accVal[6];
		memset(accVal, 0xff, sizeof(accVal));
		HAL_I2C_Mem_Read(&hi2c1, I2C_SLAVE_ADD, ACC_REG_ADD, I2C_REG_ADD_SIZ, accVal, sizeof(accVal), 100);

		float accX = (float)decode2sCompliment(accVal[0] << 8 | accVal[1]) / ACC_2G_DIV_CONST;
		float accY = (float)decode2sCompliment(accVal[2] << 8 | accVal[3]) / ACC_2G_DIV_CONST;
		float accZ = (float)decode2sCompliment(accVal[4] << 8 | accVal[5]) / ACC_2G_DIV_CONST;
		// Sum all readings
		accErrorX = accErrorX + ((atan((accY) / sqrt(pow((accX), 2) + pow((accZ), 2))) * 180 / PI));
		accErrorY = accErrorY + ((atan(-1 * (accX) / sqrt(pow((accY), 2) + pow((accZ), 2))) * 180 / PI));
	}
	//Divide the sum by NB_EXPERIMENTS to get the error value
	accErrorX = accErrorX / NB_EXPERIMENTS;
	accErrorY = accErrorY / NB_EXPERIMENTS;

	float gyroErrorX = 0.0;
	float gyroErrorY = 0.0;
	float gyroErrorZ = 0.0;

	for (int i = 0; i < NB_EXPERIMENTS; ++i) {
		uint8_t gyroVal[6];
		memset(gyroVal, 0xff, sizeof(gyroVal));
		HAL_I2C_Mem_Read(&hi2c1, I2C_SLAVE_ADD, GYRO_REG_ADD, I2C_REG_ADD_SIZ, gyroVal, sizeof(gyroVal), 100);

		float gyroX = (float)decode2sCompliment(gyroVal[0] << 8 | gyroVal[1]);
		float gyroY = (float)decode2sCompliment(gyroVal[2] << 8 | gyroVal[3]);
		float gyroZ = (float)decode2sCompliment(gyroVal[4] << 8 | gyroVal[5]);
		// Sum all readings
		gyroErrorX = gyroErrorX + (gyroX / GYRO_250DPS_DIV_CONST);
		gyroErrorY = gyroErrorY + (gyroY / GYRO_250DPS_DIV_CONST);
		gyroErrorZ = gyroErrorZ + (gyroZ / GYRO_250DPS_DIV_CONST);
	}
	//Divide the sum by NB_EXPERIMENTS to get the error value
	gyroErrorX = gyroErrorX / NB_EXPERIMENTS;
	gyroErrorY = gyroErrorY / NB_EXPERIMENTS;
	gyroErrorZ = gyroErrorZ / NB_EXPERIMENTS;

	// Print AccErrXY
	char printStr[32] = {0};
	snprintf(printStr, sizeof(printStr), "AEXY: %d %d\n", (int)accErrorX, (int)accErrorY);
	HAL_UART_Transmit(&huart1, (uint8_t*)printStr, strlen(printStr), 100);
	// Print GyroErrXYZ
	snprintf(printStr, sizeof(printStr), "GEXYZ: %d %d %d\n", (int)gyroErrorX, (int)gyroErrorY, (int)gyroErrorZ);
	HAL_UART_Transmit(&huart1, (uint8_t*)printStr, strlen(printStr), 100);
}

void printSensorVal( void ) {
	uint8_t accVal[6];
	memset(accVal, 0xff, sizeof(accVal));
	HAL_I2C_Mem_Read(&hi2c1, I2C_SLAVE_ADD, ACC_REG_ADD, I2C_REG_ADD_SIZ, accVal, sizeof(accVal), 100);

	float accX = (float)decode2sCompliment(accVal[0] << 8 | accVal[1]) / ACC_2G_DIV_CONST;
	float accY = (float)decode2sCompliment(accVal[2] << 8 | accVal[3]) / ACC_2G_DIV_CONST;
	float accZ = (float)decode2sCompliment(accVal[4] << 8 | accVal[5]) / ACC_2G_DIV_CONST;
	// Calculating Roll and Pitch from the accelerometer data
	float accAngleX = (atan(accY / sqrt(pow(accX, 2) + pow(accZ, 2))) * 180 / PI) - ACC_X_ERR; // See the calSensorErr() for more details
	float accAngleY = (atan(-1 * accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / PI) - ACC_Y_ERR;

	static float prevTim = 0; // tick is in ms
	float curTim = HAL_GetTick();
	float elapsedTime = (curTim - prevTim) / 1000; // /1000 to get seconds
	prevTim = curTim;

	uint8_t gyroVal[6];
	memset(gyroVal, 0xff, sizeof(gyroVal));
	HAL_I2C_Mem_Read(&hi2c1, I2C_SLAVE_ADD, GYRO_REG_ADD, I2C_REG_ADD_SIZ, gyroVal, sizeof(gyroVal), 100);
	float gyroX = (float)decode2sCompliment(gyroVal[0] << 8 | gyroVal[1]) / GYRO_250DPS_DIV_CONST;
	float gyroY = (float)decode2sCompliment(gyroVal[2] << 8 | gyroVal[3]) / GYRO_250DPS_DIV_CONST;
	float gyroZ = (float)decode2sCompliment(gyroVal[4] << 8 | gyroVal[5]) / GYRO_250DPS_DIV_CONST;

	gyroX = gyroX - GYRO_X_ERR;
	gyroY = gyroY - GYRO_Y_ERR;
	gyroZ = gyroZ - GYRO_Z_ERR;

	// raw values are in deg/s, so // deg/s * s = deg
	static float gyroAngleX = 0.0;
	static float gyroAngleY = 0.0;
	static float gyroAngleZ = 0.0;
	gyroAngleX = gyroAngleX + gyroX * elapsedTime; // integrating angular velocity over time to get angle
	gyroAngleY = gyroAngleY + gyroY * elapsedTime;
	gyroAngleZ = gyroAngleZ + gyroZ * elapsedTime;
	int yaw = gyroAngleZ;
	// Complementary filter - combine acceleromter and gyro angle values
	int roll = GYRO_DRIFT * gyroAngleX + ACC_DRIFT * accAngleX;
	int pitch = GYRO_DRIFT * gyroAngleY + ACC_DRIFT * accAngleY;

	char printStr[32] = {0};
	snprintf(printStr, sizeof(printStr), "RPY: %d %d %d\n", roll, pitch, yaw);
	HAL_UART_Transmit(&huart1, (uint8_t*)printStr, strlen(printStr), 100);
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
	uint8_t whoAmI = 0;
	HAL_I2C_Mem_Read(&hi2c1, I2C_SLAVE_ADD, WHO_AM_I_REG_ADD, I2C_REG_ADD_SIZ, &whoAmI, sizeof(whoAmI), 100);
	if( whoAmI != WHO_AM_I_REG_VAL ){
		char whoAmIStr[6] = {0, 0, 0, 0, 0, 0};
		snprintf(whoAmIStr, sizeof(whoAmIStr), "0x%x\n", whoAmI);
		HAL_UART_Transmit(&huart1, (uint8_t*)whoAmIStr, strlen(whoAmIStr), 100);
	}

	uint8_t wakeUp = WAKEUP_REG_VAL;
	if( HAL_OK != HAL_I2C_Mem_Write(&hi2c1, I2C_SLAVE_ADD, WAKEUP_REG_ADD, I2C_REG_ADD_SIZ, &wakeUp, sizeof(wakeUp), 100) ){
		Error_Handler();
	}

	uint8_t accCfg = 0xff;
	HAL_I2C_Mem_Read(&hi2c1, I2C_SLAVE_ADD, ACC_SEN_CFG_REG, I2C_REG_ADD_SIZ, &accCfg, sizeof(accCfg), 100);
	uint8_t gyroCfg = 0xff;
	HAL_I2C_Mem_Read(&hi2c1, I2C_SLAVE_ADD, GYRO_SEN_CFG_REG, I2C_REG_ADD_SIZ, &gyroCfg, sizeof(gyroCfg), 100);
	char cfgStr[12] = {0};
	snprintf(cfgStr, sizeof(cfgStr), "0x%x 0x%x\n", accCfg, gyroCfg);
	HAL_UART_Transmit(&huart1, (uint8_t*)cfgStr, strlen(cfgStr), 100);

	printSensorErr();
	/* Infinite loop */
	for(;;)
	{
		printSensorVal(); // HAL_GPIO_TogglePin(greenLed_GPIO_Port, greenLed_Pin);
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
	uint8_t pTxData[20] = { 0x10, 0x00, 0x0a, 0x4, 0x41, 0x54, 0x5a, 0x0a, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	uint8_t eRxResp[20] = { 0x20, 0x00, 0x0a, 0x4, 0x4f, 0x4b, 0x5a, 0x0d, 0x0a, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	uint8_t pRxData[20] = {0};
	HAL_SPI_Init(&hspi1);
	HAL_SPI_TransmitReceive(&hspi1, pTxData, pRxData, sizeof(pTxData), 100);
	HAL_SPI_DeInit(&hspi1);
	if( memcmp(eRxResp, pRxData, sizeof(eRxResp)) != 0 ){
		HAL_UART_Transmit(&huart1, pRxData, sizeof(pRxData), 100);
	}
	osDelay(1000);
	/* Infinite loop */
	for(;;)
	{
		uint8_t pTxData[3][20] = {
				{ 0x10, 0x01, 0x0a, 0x10, 0x54, 0x68, 0x65, 0x20, 0x71, 0x75, 0x69, 0x63, 0x6b, 0x20, 0x62, 0x72, 0x6f, 0x77, 0x6e, 0x20},
				{ 0x10, 0x01, 0x0a, 0x10, 0x66, 0x6f, 0x78, 0x20, 0x6a, 0x75, 0x6d, 0x70, 0x73, 0x20, 0x6f, 0x76, 0x65, 0x72, 0x20, 0x74},
				{ 0x10, 0x01, 0x0a, 0xc, 0x68, 0x65, 0x20, 0x6c, 0x61, 0x7a, 0x79, 0x20, 0x64, 0x6f, 0x67, 0x0a, 0xff, 0xff, 0xff, 0xff}
		};
		uint8_t eRxResp[20] = { 0x20, 0x01, 0x0a, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
		for(int i = 0; i < 3; ++i){
			uint8_t pRxData[20] = { 0 };
			HAL_SPI_Init(&hspi1);
			HAL_SPI_TransmitReceive(&hspi1, &pTxData[i][0], pRxData, 20, 100);
			HAL_SPI_DeInit(&hspi1);
			if( memcmp(eRxResp, pRxData, sizeof(eRxResp)) != 0 ){
				HAL_UART_Transmit(&huart1, pRxData, sizeof(pRxData), 100);
			}
			osDelay(50);
		}
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
	HAL_UART_Transmit(&huart1, (uint8_t*)"fuck", strlen("fuck"), 100);
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
