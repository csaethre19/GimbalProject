#include "I2C.h"
#include "HMC5883.h"

#include "stm32f0xx_hal.h"


I2C_HandleTypeDef hi2c;

void HMC5883_Init(volatile HMC5883_t *dataStruct) {
		dataStruct->deviceAddr = HMC5883_ADDR;
    uint8_t configRegA_data = 0x70; // Normal measurement mode, 8 samples averaged, 15 Hz output rate
    uint8_t modeReg_data = 0x00;    // Continuous measurement mode

    // Configure Configuration Register A
		I2C_WriteRegister(dataStruct->deviceAddr, 0x02, 0x00);
    // Configure Mode Register
    HAL_I2C_Mem_Write(&hi2c, HMC5883_ADDR, HMC5883_MODE_REG, 1, &modeReg_data, 1, HAL_MAX_DELAY);
}

void HMC5883_ReadRawData(volatile HMC5883_t *dataStruct) {
    uint8_t buffer[6];
		double tempholderx = 0;
		double tempholdery = 0;
		double tempholderz = 0;
    // Read data from Data Output Register

		I2C_ReadBurst(dataStruct->deviceAddr, 0x03, buffer, 6);
    // Combine the 8-bit high and low data for each axis into a 16-bit signed value
		tempholderx = (double)((buffer[0] << 8) | buffer[1]);
		tempholderz = (double)((buffer[2] << 8) | buffer[3]);
		tempholdery = (double)((buffer[4] << 8) | buffer[5]);
		//LSB SENSE Adjustment
		tempholderx = tempholderx / 440;
		tempholdery = tempholdery / 440;
	
	dataStruct->rawX = &tempholderx; // X-axis
	dataStruct->rawZ = &tempholderz; // Z-axis (YAW)
	dataStruct->rawY = &tempholdery; // Y-axis

}

void HMC5883_GetYaw(volatile HMC5883_t *dataStruct) {
    // Calculate yaw angle (orientation) using atan2
    dataStruct->yaw = atan2(*dataStruct->rawY, *dataStruct->rawX);

    // Convert radian to degrees
    dataStruct->yaw = dataStruct->yaw * 180.0f / 3.141592;

    // Ensure yaw is within [0, 360) degrees range
    if (dataStruct->yaw < 0) {
        dataStruct->yaw += 360.0f;
    }

}



