/*
* 	I2C communication functionality.

*		PIN setup:
*		SCL -> PB10
*		SDA -> PB11
*
*		STMCubeMX has been used to set up I2C. 
*		Read/Write functions are the only things being used. 
*/
#include "main.h"


void I2C_Ports_Config();

void I2C_SetUp();

/*
	Given a device with deviceAddr as its address, writes data to regAddr.
*/
char I2C_WriteRegister(uint16_t deviceAddr, uint8_t regAddr, uint8_t data);

/*
	Given a device with deviceAddr as its address, returns data from register 
	specified from previous write.
	This should be called after a call to I2C_SetRegAddress()!
*/
int8_t I2C_ReadRegister(uint16_t deviceAddr);

/*
	Given a device with deviceAddr as its address, performs a burst read starting at the register 
	specified by regAddr. 
	An array of type int8_t should be passed in for dataBuffer with size length.
	Will read as many bytes that length parameter specifies. 
	Data is retreived after read through dataBuffer.
*/
int I2C_ReadBurst(uint16_t deviceAddr, uint8_t regAddr, int8_t *dataBuffer, uint16_t length);

/*
	Given a device with deviceAddr as its address, sets the register address to read from to regAddr.
*/
void I2C_SetRegAddress(uint16_t deviceAddr, uint8_t regAddr);

void I2C_BurstRead_Cheap(uint16_t deviceAddr, uint8_t regAddr, uint16_t length);

void I2C_DMA_Read(uint16_t deviceAddr, uint8_t regAddr, uint16_t length);

