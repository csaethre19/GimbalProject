#ifndef HMC5883_H
#define HMC5883_H

#include "I2C.h"
#include <math.h>
#include "stm32f0xx_hal.h"


#define HMC5883_ADDR        0x1E  // HMC5883 chip address
#define HMC5883_CONFIG_REGA 0x00  // Configuration Register A address
#define HMC5883_MODE_REG    0x02  // Mode Register address
#define HMC5883_DATA_REG    0x03  // Data Output Register address


// datastruct used to hold magnetometer data that are read from HMC5883 device

typedef struct {
	
	uint16_t deviceAddr;
	double *rawX;
	double *rawY;
	double *rawZ;
	
	double yaw;
} HMC5883_t;

void HMC5883_Init(volatile HMC5883_t *dataStruct);	

void HMC5883_ReadRawData(volatile HMC5883_t *dataStruct);
	
void HMC5883_GetYaw(volatile HMC5883_t *dataStruct);

#endif /* HMC5883_H */