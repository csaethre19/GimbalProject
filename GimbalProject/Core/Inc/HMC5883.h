#ifndef HMC5883_H
#define HMC5883_H

#include "I2C.h"
#include <math.h>
#include "stm32f0xx_hal.h"
#include "USART.h"

#define QMC_ADDR 0x0D

#define MAG_LSB_SENS 420

// datastruct used to hold magnetometer data that are read from HMC5883 device

typedef struct {
	
	uint16_t deviceAddr;
	
	float x_raw;
	float y_raw;
	
	double AngleYaw;
} HMC5883_t;

void HMC5883_Init(volatile HMC5883_t *dataStruct);	

void HMC5883_ReadRawData(volatile HMC5883_t *dataStruct);
	


#endif /* HMC5883_H */