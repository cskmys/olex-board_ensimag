/*
 * sensordata.h
 *
 *  Created on: Apr 7, 2020
 *      Author: csk
 */

#ifndef INC_SENSORDATA_H_
#define INC_SENSORDATA_H_

#include "blespi.h"
#include "sensor.h"

void rdSensorCfg(void);
void getSensorCfg(int *aCfg, int *gCfg);
void calcSensorErr(void);
void getSensorErr(Reading *aErr, Reading *gErr);
void calcSensorRollPitchYaw(void);
void getSensorRollPitchYaw(Reading *r);

#endif /* INC_SENSORDATA_H_ */
