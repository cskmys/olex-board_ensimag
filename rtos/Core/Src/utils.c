
#include "utils.h"
#include "main.h"
#include "cmsis_os.h"
#include<stdio.h>
#include<stdarg.h>
#include<string.h>

extern UART_HandleTypeDef huart1;
extern osMutexId_t printUartLckHandle;

void lckPrintUart(){
	osMutexAcquire(printUartLckHandle,0);
}

void ulckPrintUart(){
	osMutexRelease(printUartLckHandle);
}

void uartPrint(uint8_t *buff, uint8_t len){
#ifdef DEBUG
	lckPrintUart();
	if( HAL_UART_Transmit(&huart1, buff, len, 100) != HAL_OK ){
		Error_Handler();
	}
	ulckPrintUart();
#endif
}

void dbgPutc(char c){
	HAL_UART_Transmit(&huart1, (uint8_t*)&c, 1, 100);
}

void dbgPrintBuff(uint8_t *buff, uint8_t nbEle){
	for(int i = 0; i < nbEle; ++i){
		dbgPrintf("0x%02x ");
	}
	dbgPrintf("\n");
}


void dbgPrintf(const char* fmt, ...) {
	va_list args;
	va_start(args, fmt);
	char printStr[32] = {0};
	vsnprintf(printStr, sizeof(printStr), fmt, args);
	va_end(args);
	uartPrint((uint8_t*)printStr, strnlen(printStr, sizeof(printStr)));
}

int decode2sCompliment(uint16_t val){
	return (int)( 0x8000 & val ? (int)( 0x7FFF & val ) - 0x8000 : val );
}

uint16_t concatBytes(uint8_t msbByte, uint8_t lsbByte){
	return (uint16_t)(msbByte << 8 | lsbByte);
}

