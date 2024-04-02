#include "main.h"

	/*SCL -> PB10
	 SDA -> PB11
	*/

void I2C_Ports_Config();

void I2C_SetUp();

void I2C_WriteRegister(uint16_t deviceAddr, uint8_t regAddr, uint8_t data);

int8_t I2C_ReadRegister(uint16_t deviceAddr);

int I2C_ReadBurst(uint16_t deviceAddr, uint8_t regAddr, int8_t *dataBuffer, uint16_t length);

void I2C_SetRegAddress(uint16_t deviceAddr, uint8_t regAddr);