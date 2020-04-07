#include "utils.h"
#include "main.h"
#include "cmsis_os.h"
#include<stdio.h>
#include<stdarg.h>
#include<string.h>

static __inline uint32_t ITM_SendCharOnPort(uint8_t port, uint32_t ch) {
	if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) && /* ITM enabled */
	((ITM->TER & (1 << port)) != 0UL)) /* ITM Port # enabled */
	{
		while (ITM->PORT[port].u32 == 0UL) {
			__NOP();
		}
		ITM->PORT[port].u8 = (uint8_t) ch;
	}
	return (ch);
}

void itmPrint(uint8_t *buff, uint8_t len, uint8_t port) {
#ifdef DEBUG
	for (int i = 0; i < len; ++i) {
		ITM_SendCharOnPort(port, buff[i]);
	}
#endif
}

void dbgPrintBuff(uint8_t *buff, uint8_t nbEle) {
	for (int i = 0; i < nbEle; ++i) {
		char printStr[8] = { 0 };
		snprintf(printStr, sizeof(printStr), "0x%02x ", buff[i]);
		itmPrint((uint8_t*) printStr, strnlen(printStr, sizeof(printStr)), 1);
	}
	char printStr[8] = { 0 };
	snprintf(printStr, sizeof(printStr), "\n");
	itmPrint((uint8_t*) printStr, strnlen(printStr, sizeof(printStr)), 1);
}

void dbgPrintf(const char *fmt, ...) {
	va_list args;
	va_start(args, fmt);
	char printStr[32] = { 0 };
	vsnprintf(printStr, sizeof(printStr), fmt, args);
	va_end(args);
	itmPrint((uint8_t*) printStr, strnlen(printStr, sizeof(printStr)), 0);
}

int decode2sCompliment(uint16_t val) {
	return (int) (0x8000 & val ? (int) (0x7FFF & val) - 0x8000 : val);
}

uint16_t concatBytes(uint8_t msbByte, uint8_t lsbByte) {
	return (uint16_t) (msbByte << 8 | lsbByte);
}

