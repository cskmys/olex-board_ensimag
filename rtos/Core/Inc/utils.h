/*
 * utils.h
 *
 *  Created on: Apr 7, 2020
 *      Author: csk
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include <stdint.h>

extern void dbgPrintf(const char* fmt, ...);
extern void dbgPrintBuff(uint8_t *buff, uint8_t nbEle);
extern int decode2sCompliment(uint16_t val);
extern uint16_t concatBytes(uint8_t msbByte, uint8_t lsbByte);

#endif /* INC_UTILS_H_ */
