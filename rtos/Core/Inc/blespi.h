/*
 * blespi.h
 *
 *  Created on: Apr 6, 2020
 *      Author: csk
 */

#ifndef INC_BLESPI_H_
#define INC_BLESPI_H_

#include<stdbool.h>

void bleRst( void );
bool sendBlePkt(uint8_t *req, uint8_t *eRxResp);
void blePrintf(const char* fmt, ...);

#endif /* INC_BLESPI_H_ */
