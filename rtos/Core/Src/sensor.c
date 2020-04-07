/*
 * sensor.c
 *
 *  Created on: Apr 6, 2020
 *      Author: csk
 */

#include "main.h"
#include "sensor.h"
#include "gpio.h"
#include "utils.h"

#include<math.h>

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

extern I2C_HandleTypeDef hi2c1;

void sensorRegRd(uint8_t regAddr, uint8_t *buff, uint8_t nbReg) {
	if (HAL_I2C_Mem_Read(&hi2c1, I2C_SLAVE_ADD, regAddr, I2C_REG_ADD_SIZ, buff,
			nbReg, 100) != HAL_OK) {
		Error_Handler();
	}
	toggleSensorStatusLed();
}
void sensorRegWr(uint8_t regAddr, uint8_t *buff, uint8_t nbReg) {
	if (HAL_I2C_Mem_Write(&hi2c1, I2C_SLAVE_ADD, regAddr, I2C_REG_ADD_SIZ, buff,
			nbReg, 100) != HAL_OK) {
		Error_Handler();
	}
	toggleSensorStatusLed();
}

void chkSensor(void) {
	uint8_t whoAmI = 0;
	sensorRegRd(WHO_AM_I_REG_ADD, &whoAmI, sizeof(whoAmI));
	if (whoAmI != WHO_AM_I_REG_VAL) {
		Error_Handler();
	}
}

void wakeSensor(void) {
	uint8_t wakeUp = WAKEUP_REG_VAL;
	sensorRegWr(WAKEUP_REG_ADD, &wakeUp, sizeof(wakeUp));
}

int getAccCfg(void) {
	uint8_t accCfg = 0xff;
	sensorRegRd(ACC_SEN_CFG_REG, &accCfg, sizeof(accCfg));
	return accCfg;
}

int getGyroCfg(void) {
	uint8_t gyroCfg = 0xff;
	sensorRegRd(GYRO_SEN_CFG_REG, &gyroCfg, sizeof(gyroCfg));
	return gyroCfg;
}

void initSensor(void) {
	chkSensor();
	wakeSensor();
}

/* USER CODE BEGIN 0 */
void cfgSensorFullScale() {
#ifdef FULL_SCALE_CFG_CODE
	// IN CASE YOU UNCOMMENT THIS, MAKE SURE TO CHANGE ALL THE DIVISIBILITY VALUES AS PER THE DATASHEET FOR FULL SCALE
	// Configure Accelerometer Sensitivity - +/- 8g i,e, Full Scale Range. (default +/- 2g)
	// Reg 0x1C -> 0x10:
	uint_8 accFullScaleCfgVal = ACC_SEN_FULL_SCALE_CFG_VAL;
	sensorRegWr(ACC_SEN_CFG_REG, &accFullScaleCfgVal, sizeof(accFullScaleCfgVal));

	// Configure Gyro Sensitivity - 1000deg/s i,e, Full Scale Range (default +/- 250deg/s)
	// Reg 0x1B -> 0x10;
	uint_8 gyroFullScaleCfgVal = GYRO_SEN_FULL_SCALE_VAL_REG;
	sensorRegWr(GYRO_SEN_CFG_REG, &gyroFullScaleCfgVal, sizeof(gyroFullScaleCfgVal));
#endif
}

int mkSensorReading(uint8_t msbByte, uint8_t lsbByte) {
	return decode2sCompliment(concatBytes(msbByte, lsbByte));
}

float mkAccReading(uint8_t msbByte, uint8_t lsbByte) {
	return (float) mkSensorReading(msbByte, lsbByte) / ACC_2G_DIV_CONST;
}

float mkGyroReading(uint8_t msbByte, uint8_t lsbByte) {
	return (float) mkSensorReading(msbByte, lsbByte) / GYRO_250DPS_DIV_CONST;
}

void getAccReading(Reading *acc) {
	uint8_t accVal[6];
	sensorRegRd(ACC_REG_ADD, accVal, sizeof(accVal));

	acc->x = mkAccReading(accVal[0], accVal[1]);
	acc->y = mkAccReading(accVal[2], accVal[3]);
	acc->z = mkAccReading(accVal[4], accVal[5]);
	return;
}

void getGyroReading(Reading *gyro) {
	uint8_t gyroVal[6];
	sensorRegRd(GYRO_REG_ADD, gyroVal, sizeof(gyroVal));

	gyro->x = mkGyroReading(gyroVal[0], gyroVal[1]);
	gyro->y = mkGyroReading(gyroVal[2], gyroVal[3]);
	gyro->z = mkGyroReading(gyroVal[4], gyroVal[5]);
	return;
}

void getAccErr(Reading *accErr) {
	// PLACE SENSOR FLAT INORDER TO GET PROPER VALUES
	float accErrorX = 0.0;
	float accErrorY = 0.0;
	for (int i = 0; i < NB_EXPERIMENTS; ++i) {
		Reading acc;
		getAccReading(&acc);
		// Sum all readings
		accErrorX = accErrorX
				+ ((atan((acc.y) / sqrt(pow((acc.x), 2) + pow((acc.z), 2)))
						* 180 / PI));
		accErrorY = accErrorY
				+ ((atan(-1 * (acc.x) / sqrt(pow((acc.y), 2) + pow((acc.z), 2)))
						* 180 / PI));
	}
	//Divide the sum by NB_EXPERIMENTS to get the error value
	accErr->x = accErrorX / NB_EXPERIMENTS;
	accErr->y = accErrorY / NB_EXPERIMENTS;
	accErr->z = 0; // can't calculate z component with accelerometer
}

void getGyroErr(Reading *gyroErr) {
	float gyroErrorX = 0.0;
	float gyroErrorY = 0.0;
	float gyroErrorZ = 0.0;
	for (int i = 0; i < NB_EXPERIMENTS; ++i) {
		Reading gyro;
		getGyroReading(&gyro);
		// Sum all readings
		gyroErrorX = gyroErrorX + gyro.x;
		gyroErrorY = gyroErrorY + gyro.y;
		gyroErrorZ = gyroErrorZ + gyro.z;
	}
	//Divide the sum by NB_EXPERIMENTS to get the error value
	gyroErr->x = gyroErrorX / NB_EXPERIMENTS;
	gyroErr->y = gyroErrorY / NB_EXPERIMENTS;
	gyroErr->z = gyroErrorZ / NB_EXPERIMENTS;
}

void getAccRollPitchYaw(Reading *rpy) {
	Reading acc;
	getAccReading(&acc);

	// Calculating Roll and Pitch from the accelerometer data
	rpy->x = (atan(acc.y / sqrt(pow(acc.x, 2) + pow(acc.z, 2))) * 180 / PI)
			- ACC_X_ERR; // See the calSensorErr() for more details
	rpy->y = (atan(-1 * acc.x / sqrt(pow(acc.y, 2) + pow(acc.z, 2))) * 180 / PI)
			- ACC_Y_ERR;
	rpy->z = 0.0; // cant calculate with accelerometer
}

void getGyroRollPitchYaw(Reading *rpy) {
	static float prevTim = 0; // tick is in ms
	float curTim = HAL_GetTick();
	float elapsedTime = (curTim - prevTim) / 1000; // /1000 to get seconds
	prevTim = curTim;

	Reading gyro;
	getGyroReading(&gyro);

	gyro.x = gyro.x - GYRO_X_ERR;
	gyro.y = gyro.y - GYRO_Y_ERR;
	gyro.z = gyro.z - GYRO_Z_ERR;

	// raw values are in deg/s, so // deg/s * s = deg
	static float gyroAngleX = 0.0;
	static float gyroAngleY = 0.0;
	static float gyroAngleZ = 0.0;
	rpy->x = gyroAngleX = gyroAngleX + gyro.x * elapsedTime; // integrating angular velocity over time to get angle
	rpy->y = gyroAngleY = gyroAngleY + gyro.y * elapsedTime;
	rpy->z = gyroAngleZ = gyroAngleZ + gyro.z * elapsedTime;
}

void getDriftCorrectedRollPitchYaw(Reading accRpy, Reading gyroRpy,
		Reading *rpy) {
	// Complementary filter - combine acceleromter and gyro angle values
	rpy->x = GYRO_DRIFT * gyroRpy.x + ACC_DRIFT * accRpy.x;
	rpy->y = GYRO_DRIFT * gyroRpy.y + ACC_DRIFT * accRpy.y;
	rpy->z = gyroRpy.z; // since we cant calculate yaw in accelerometer
}

void getRollPitchYaw(Reading *rpy) {
	Reading accRpy;
	getAccRollPitchYaw(&accRpy);

	Reading gyroRpy;
	getGyroRollPitchYaw(&gyroRpy);

	getDriftCorrectedRollPitchYaw(accRpy, gyroRpy, rpy);
}
