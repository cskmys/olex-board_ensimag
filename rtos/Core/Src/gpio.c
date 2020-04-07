/*
 * leds.c
 *
 *  Created on: Apr 7, 2020
 *      Author: csk
 */

#include "main.h"

void toggleBleStatusLed(void) {
	HAL_GPIO_TogglePin(blueLed_GPIO_Port, blueLed_Pin);
}

void toggleSensorStatusLed(void) {
	HAL_GPIO_TogglePin(greenLed_GPIO_Port, greenLed_Pin);
}

void clrBleStatusLed(void) {
	HAL_GPIO_WritePin(blueLed_GPIO_Port, blueLed_Pin, GPIO_PIN_RESET);
}

void clrSensorStatusLed(void) {
	HAL_GPIO_WritePin(greenLed_GPIO_Port, greenLed_Pin, GPIO_PIN_RESET);
}
