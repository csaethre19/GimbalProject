
#include "HMC5883.h"



void HMC5883_Init(volatile HMC5883_t *dataStruct) {
		dataStruct->deviceAddr = 0x0D;
    uint8_t configRegA_data = 0x70; // Normal measurement mode, 8 samples averaged, 15 Hz output rate
    uint8_t modeReg_data = 0x00;    // Continuous measurement mode

    // Configure Configuration Register A
		//I2C_WriteRegister(dataStruct->deviceAddr, 0x00, 0x70);
    // Configure Mode Register
		I2C_WriteRegister(0x0D, 0x02, 0x00);

	
		GPIOC->ODR |= GPIO_ODR_8;
}

void HMC5883_ReadRawData(volatile HMC5883_t *dataStruct) {
    int8_t dataBuffer[6]; // reading X and Y
	
	I2C_ReadBurst(dataStruct->deviceAddr, 0x03, dataBuffer, 6); 
	
	int8_t xhigh = dataBuffer[0];
	int8_t xlow = dataBuffer[1];
	float x_raw = (int16_t)(xhigh << 8 | xlow);
	float mag_x = x_raw/MAG_LSB_SENS; 
	
	int8_t yhigh = dataBuffer[4];
	int8_t ylow = dataBuffer[5];
	float y_raw = (int16_t)(yhigh << 8 | ylow);
	float mag_y = y_raw/MAG_LSB_SENS; 
	
	// Calculating Yaw Angle
	dataStruct->yaw = atan2(-mag_y, mag_x);
}





