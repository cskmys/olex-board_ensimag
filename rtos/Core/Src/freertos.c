/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "cmsis_os.h"
#include "sensordata.h"
#include "utils.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void printSensorCfg(void) {
	int aCfg;
	int gCfg;
	getSensorCfg(&aCfg, &gCfg);
	dbgPrintf("CfgAG: 0x%02x 0x%02x\n", aCfg, gCfg);
}

void printSensorErr(void) {
	// PLACE SENSOR FLAT INORDER TO GET PROPER VALUES
	dbgPrintf("ERR CALC:KEEP SENS FLAT\n");
	// The error values used in the equations are calculated here
	Reading aErr, gErr;
	getSensorErr(&aErr, &gErr);
	dbgPrintf("AEXY: %d %d\n", (int) aErr.x, (int) aErr.y);
	dbgPrintf("GEXYZ: %d %d %d\n", (int) gErr.x, (int) gErr.y, (int) gErr.z);
}

void printRollPitchYaw(void) {
	Reading r;
	getSensorRollPitchYaw(&r);
	dbgPrintf("R:%d P:%d Y:%d\n", (int) r.x, (int) r.y, (int) r.z);
}

void bleTxRollPitchYaw(void) {
	Reading r;
	getSensorRollPitchYaw(&r);
	blePrintf("RPY: %d %d %d\n", (int) r.x, (int) r.y, (int) r.z);
}

void bleTxSensorCfg(void) {
	int aCfg;
	int gCfg;
	getSensorCfg(&aCfg, &gCfg);
	blePrintf("CfgAG: 0x%02x 0x%02x\n", aCfg, gCfg);
}

void bleTxSensorErr(void) {
	// The error values used in the equations are calculated here
	Reading aErr, gErr;
	getSensorErr(&aErr, &gErr);
	// PLACE SENSOR FLAT INORDER TO GET PROPER VALUES
	blePrintf("ERR CALC:KEEP SENS FLAT\n");
	blePrintf("AEXY: %d %d\n", (int) aErr.x, (int) aErr.y);
	blePrintf("GEXYZ: %d %d %d\n", (int) gErr.x, (int) gErr.y, (int) gErr.z);
}

void bleTxDevInfo(){
	char verStr[32];
	get32CharDevInfo(verStr);
	blePrintf("%s\n", verStr);
}

void nullTask(void) {
	for (;;) {
		osDelay(1);
	}
}
// PRIORITY OF SENSOR TASK SHOULD BE HIGHER THAN TASKS WHICH CONSUME SENSOR DATA!!!
// THIS OS DOESN"T SEEM TO HAVE PRIORITY INVERSION.
// HENCE, THERE COULD BE DEADLOCK ON SENSOR DATA IF SENSOR TASK IS AT LOWER PRIORITY THAN TASKS THAT CONSUME SENSOR DATA
void sensorTask(void) {
	initSensor();

	//	// DONT USE ANY OF THE APIS BELOW WITHOUT IMPLEMENTING SIGNAL SYNC MECHANISM
	//	// IF YOU USE WITH SYNC MECHANISM, YOU CAN FIND YOURSELF IN DEADLOCK
	//	rdSensorCfg();
	//	// sensor config read signal
	//	printSensorCfg();
	//
	//	calcSensorErr();
	//	// sensor error read signal
	//	printSensorErr();

	//  // sendor init complete signal
	for (;;) {
		calcSensorRollPitchYaw();
		printRollPitchYaw();

		osDelay(50);
	}
}

void bleTask(void) {
	bleRst();
	osDelay(1000);

	bleTxDevInfo();
	//	// DONT USE ANY OF THE APIS BELOW WITHOUT IMPLEMENTING SIGNAL SYNC MECHANISM
	//	// IF YOU USE WITH SYNC MECHANISM, YOU CAN FIND YOURSELF IN DEADLOCK
	//	// wait for sensor task signals
	//	// if sensor cfg read signal
	//		bleTxSensorCfg();
	//	// if sensor error read signal
	//		bleTxSensorErr();
	//	// if sensor init complete signal
	for (;;) {
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
		bleTxRollPitchYaw();
		osDelay(500);
	}
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
