/*
 * sensor.h
 *
 *  Created on: Apr 6, 2020
 *      Author: csk
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

typedef struct{
	float x;
	float y;
	float z;
} Reading;

void initSensor( void );
int getAccCfg( void );
int getGyroCfg( void );
void getAccErr(Reading *accErr);
void getGyroErr(Reading *gyroErr);
void getRollPitchYaw(Reading *rpy);


#endif /* INC_SENSOR_H_ */
