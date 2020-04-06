/*
 * blespi.c
 *
 *  Created on: Apr 6, 2020
 *      Author: csk
 */

#include "main.h"
#include "blespi.h"
#include<stdio.h>
#include<stdarg.h>
#include<string.h>
#include<math.h>

#define BLE_PKT_SIZ 20
#define PKT_DAT_SIZ 16
#define DAT_SIZ_IDX 3
#define DAT_START_IDX 4

extern SPI_HandleTypeDef hspi1;

void txRxBleSpi(uint8_t *pTxData, uint8_t *pRxData){
	HAL_SPI_Init(&hspi1);
	HAL_SPI_TransmitReceive(&hspi1, pTxData, pRxData, BLE_PKT_SIZ, 100);
	HAL_SPI_DeInit(&hspi1);
}

bool sendBlePkt(uint8_t *req, uint8_t *eRxResp){
	uint8_t pRxData[BLE_PKT_SIZ] = {0};
	txRxBleSpi(req, pRxData);
	if(eRxResp != NULL){
		if( memcmp(eRxResp, pRxData, BLE_PKT_SIZ) != 0 ){
			return false;
		}
	}
	return true;
}

void sendBleATZCmd( void ){
	uint8_t pTxData[BLE_PKT_SIZ] = { 0x10, 0x00, 0x0a, 0x4, 0x41, 0x54, 0x5a, 0x0a, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	uint8_t eRxResp[BLE_PKT_SIZ] = { 0x20, 0x00, 0x0a, 0x4, 0x4f, 0x4b, 0x5a, 0x0d, 0x0a, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	sendBlePkt(pTxData, eRxResp);
}

void bleRst( void ){
	sendBleATZCmd();
}

void blePrintStr(char *printStr, uint8_t siz) {
	uint8_t pTxData[BLE_PKT_SIZ] = { 0x10, 0x01, 0x0a, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	pTxData[DAT_SIZ_IDX] = siz;
	for(int i = 0; i < siz; ++i){
		pTxData[DAT_START_IDX+i] = printStr[i];
	}

	uint8_t eRxResp[20] = { 0x20, 0x01, 0x0a, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	sendBlePkt( pTxData, eRxResp);
}

void blePrintf(const char* fmt, ...) {
	va_list args;
	va_start(args, fmt);
	char printStr[PKT_DAT_SIZ * 2] = {0};
	memset(printStr, 0, sizeof(printStr));
	vsnprintf(printStr, sizeof(printStr), fmt, args);
	va_end(args);

	uint8_t siz = strlen(printStr);
	uint8_t nbPkt = ceil((float)siz / PKT_DAT_SIZ);
	for(int i = 0; i < nbPkt; ++i){
		char curStr[PKT_DAT_SIZ];
		memcpy(curStr, printStr + (i * PKT_DAT_SIZ), PKT_DAT_SIZ);
		uint8_t curStrSiz = strlen(curStr);
		curStrSiz = (strlen(curStr) > PKT_DAT_SIZ) ? PKT_DAT_SIZ : curStrSiz;
		blePrintStr(curStr, curStrSiz);
		osDelay(50); // if multiple packets you'll need to put this.
	}

}
