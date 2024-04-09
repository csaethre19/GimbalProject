#include "HMC5883.h"


//READ  ADDRESS = 0x3D
//WRITE ADDRESS = 0x3C


void HMC5883_Init(volatile HMC5883_t *dataStruct) {
		I2C_WriteRegister(QMC_ADDR, 0x0B, 0x01);
		I2C_WriteRegister(QMC_ADDR, 0x09, 0x1D);
}

void HMC5883_ReadRawData(volatile HMC5883_t *dataStruct) {
  int8_t dataBuffer[4]; // reading X and Y
	
	I2C_ReadBurst(QMC_ADDR, 0x00, dataBuffer, 4); 
	
	uint8_t xhigh = dataBuffer[0];
	uint8_t xlow = dataBuffer[1];
	float x_raw = (int16_t)(xhigh << 8 | xlow);
	float mag_x = x_raw/MAG_LSB_SENS; 
	dataStruct->x_raw = x_raw;
	uint8_t yhigh = dataBuffer[2];
	uint8_t ylow = dataBuffer[3];
	float y_raw = (int16_t)(yhigh << 8 | ylow);
	float mag_y = y_raw/MAG_LSB_SENS; 
	dataStruct->y_raw = y_raw;
	// Calculating Yaw Angle
	dataStruct->AngleYaw = atan2( y_raw, x_raw ) * 180.0 / 3.14159265;

}





