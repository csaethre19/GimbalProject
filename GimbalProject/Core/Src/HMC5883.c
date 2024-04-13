#include "HMC5883.h"


//READ  ADDRESS = 0x3D
//WRITE ADDRESS = 0x3C


void HMC5883_Init(volatile HMC5883_t *dataStruct) {
	I2C_WriteRegister(0x0D, 0x0B, 0x01);
	I2C_WriteRegister(0x0D, 0x09, 0x1D);
	USART_Transmit_String("HMC Init Complete");	
}

void HMC5883_ReadRawData(volatile HMC5883_t *dataStruct) {
  int8_t dataBuffer[4]; // reading X and Y
	
	I2C_ReadBurst(0x0D, 0x00, dataBuffer, 4); 
	
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
	USART_Transmit_String("MPU6050 Address: ");
	USART_Transmit_Number(dataStruct->AngleYaw);
	USART_Transmit_String("ReadRawdata Complete"); 
}





