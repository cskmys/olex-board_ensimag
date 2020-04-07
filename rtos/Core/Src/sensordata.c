#include "cmsis_os.h"
#include "sensordata.h"

extern osMutexId_t sensorDatLckHandle;

void lckSensorData( void ){
	osMutexAcquire(sensorDatLckHandle,0);
}

void ulckSensorData( void ){
	osMutexRelease(sensorDatLckHandle);
}

static int accCfg = 0xff, gyroCfg = 0xff;
void rdSensorCfg( void ){
	lckSensorData();
	accCfg = getAccCfg();
	gyroCfg = getGyroCfg();
	ulckSensorData();
}

void getSensorCfg(int *aCfg, int *gCfg){
	lckSensorData();
	*aCfg = accCfg;
	*gCfg = gyroCfg;
	ulckSensorData();
}

static Reading accErr, gyroErr;
void calcSensorErr( void ){
	lckSensorData();
	getAccErr(&accErr);
	getGyroErr(&gyroErr);
	ulckSensorData();
}

void getSensorErr(Reading *aErr, Reading *gErr){
	lckSensorData();
	*aErr = accErr;
	*gErr = gyroErr;
	ulckSensorData();
}

static Reading rpy;
void calcSensorRollPitchYaw( void ) {
	lckSensorData();
	getRollPitchYaw(&rpy);
	ulckSensorData();
}

void getSensorRollPitchYaw(Reading *r){
	lckSensorData();
	*r = rpy;
	ulckSensorData();
}
