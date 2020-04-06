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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<string.h>
#include<math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
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

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
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

int normalizeDegrees(float angle){
	int retDeg = ((int)angle) % 360;
	if(retDeg < 0){
		retDeg += 360;
	}
	return retDeg;
}

int decode2sCompliment(uint16_t val){
	return (int)( 0x8000 & val ? (int)( 0x7FFF & val ) - 0x8000 : val );
}

void calSensorErr( void ) {
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
	char printStr[64] = {0};
	snprintf(printStr, sizeof(printStr), "AccErrXY: %f %f\n", accErrorX, accErrorY);
	HAL_UART_Transmit(&huart1, (uint8_t*)printStr, strlen(printStr), 100);
	// Print GyroErrXYZ
	snprintf(printStr, sizeof(printStr), "GyroErrXYZ: %f %f %f\n", gyroErrorX, gyroErrorY, gyroErrorZ);
	HAL_UART_Transmit(&huart1, (uint8_t*)printStr, strlen(printStr), 100);
}

void printSensorErr( void ){
	// Make sure sensor is perfectly flat
	calSensorErr();
	HAL_Delay(20);
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

	char printStr[64] = {0};
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
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */
	uint8_t whoAmI = 0;
	HAL_I2C_Mem_Read(&hi2c1, I2C_SLAVE_ADD, WHO_AM_I_REG_ADD, I2C_REG_ADD_SIZ, &whoAmI, sizeof(whoAmI), 100);
	if( whoAmI != WHO_AM_I_REG_VAL ){
		char whoAmIStr[6] = {0, 0, 0, 0, 0, 0};
		snprintf(whoAmIStr, sizeof(whoAmIStr), "0x%x\n", whoAmI);
		HAL_UART_Transmit(&huart1, (uint8_t*)whoAmIStr, strlen(whoAmIStr), 100);
	}

	uint8_t wakeUp = WAKEUP_REG_VAL;
	if( HAL_OK != HAL_I2C_Mem_Write(&hi2c1, I2C_SLAVE_ADD, WAKEUP_REG_ADD, I2C_REG_ADD_SIZ, &wakeUp, sizeof(wakeUp), 100) ){
		char errMsg[32];
		snprintf(errMsg, sizeof(errMsg), "screwed\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)errMsg, strlen(errMsg), 100);
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
		while(1);
	}

	uint8_t accCfg = 0xff;
	HAL_I2C_Mem_Read(&hi2c1, I2C_SLAVE_ADD, ACC_SEN_CFG_REG, I2C_REG_ADD_SIZ, &accCfg, sizeof(accCfg), 100);
	uint8_t gyroCfg = 0xff;
	HAL_I2C_Mem_Read(&hi2c1, I2C_SLAVE_ADD, GYRO_SEN_CFG_REG, I2C_REG_ADD_SIZ, &gyroCfg, sizeof(gyroCfg), 100);
	char cfgStr[12] = {0};
	snprintf(cfgStr, sizeof(cfgStr), "0x%x 0x%x\n", accCfg, gyroCfg);
	HAL_UART_Transmit(&huart1, (uint8_t*)cfgStr, strlen(cfgStr), 100);

	printSensorErr();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		printSensorVal();
		HAL_Delay(50);
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
	HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD4_Pin LD3_Pin */
	GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
