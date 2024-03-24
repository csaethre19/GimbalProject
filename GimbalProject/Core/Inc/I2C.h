#include "main.h"


void I2C_Ports_Config();

void I2C_SetUp();

void I2C_WriteRegister(uint16_t deviceAddr, uint8_t regAddr, uint8_t data);

int8_t I2C_ReadRegister(uint16_t deviceAddr);

void I2C_BurstRead(uint16_t deviceAddr, uint8_t firstReg, int8_t* dataBuffer, uint8_t length);

void I2C_SetRegAddress(uint16_t deviceAddr, uint8_t regAddr);