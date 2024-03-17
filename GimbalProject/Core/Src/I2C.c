#include <stdlib.h>

#include "main.h"

void Enable_GPIO_Clks();

void I2C_Ports_Config();

void I2C_SetUp();

void I2C_WriteRegister(uint16_t deviceAddr, uint8_t regAddr, uint8_t data);

int8_t I2C_ReadRegister(uint16_t deviceAddr);

void I2C_SetRegAddress(uint16_t deviceAddr, uint8_t regAddr);



void I2C_SetRegAddress(uint16_t deviceAddr, uint8_t regAddr)
{
	I2C2->CR2 = 0; // clear register
	// Use SADD[7:1] bit field in CR2 register to set slave address to addr
	I2C2->CR2 |= (deviceAddr << 1);
	// Use NBYTES[7:0] bit field to set number of data bytes to be transmitted to 1
	I2C2->CR2 |= (0x1 << 16);
	// Set RD_WRN to WRITE operation - 0 indicates WRITE
	I2C2->CR2 &= ~(1 << 10);
	// Set START bit to begin the address frame
	I2C2->CR2 |= I2C_CR2_START;
	
	// While TXIS or NACKF flags not set wait
	while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) {} // getting stuck here on second call to this function!
	// Once TXIS flag set continue
		
	// Check if NACK set
	if (I2C2->ISR & I2C_ISR_NACKF)
	{
		//GPIOC->ODR |= GPIO_ODR_6; // RED - I2C not working!
	}
	
	// Write data into the TXDR 
	I2C2->TXDR = regAddr;
		
	// Wait until TC flag set - transfer complete
	while (!(I2C2->ISR & I2C_ISR_TC)) {}
}

/*

	I2C communication handler for writing to specified device.
	deviceAddr - Address of device communicating on I2C
	data 			 - Either a register address or the data to be written to a register

*/
void I2C_WriteRegister(uint16_t deviceAddr, uint8_t regAddr, uint8_t data) 
{
	I2C2->CR2 = 0; // clear register
	// Use SADD[7:1] bit field in CR2 register to set slave address to addr
	I2C2->CR2 |= (deviceAddr << 1);
	// Use NBYTES[7:0] bit field to set number of data bytes to be transmitted to 1
	I2C2->CR2 |= (0x2 << 16);
	// Set RD_WRN to WRITE operation - 0 indicates WRITE
	I2C2->CR2 &= ~(1 << 10);
	// Set START bit to begin the address frame
	I2C2->CR2 |= I2C_CR2_START;
	
	// While TXIS or NACKF flags not set wait
	while (!(I2C2->ISR & (I2C_ISR_TXIS | I2C_ISR_NACKF))) {} // getting stuck here on second call to this function!
	// Once TXIS flag set continue
		
	// Check if NACK set
	if (I2C2->ISR & I2C_ISR_NACKF)
	{
		//GPIOC->ODR |= GPIO_ODR_6; // RED - I2C not working!
	}
	
	// Set reg address
	I2C2->TXDR = regAddr;

	
	while (!(I2C2->ISR & I2C_ISR_TXIS)) {}
		
	// Write data into the TXDR 	
	I2C2->TXDR = data;
		
	// Wait until TC flag set - transfer complete
	while (!(I2C2->ISR & I2C_ISR_TC)) {}
}

/*

	I2C communication handler for reading from specified device.
	deviceAddr - Address of device communicating on I2C
	returns		 - 1 byte of data read from specified register
*/
int8_t I2C_ReadRegister(uint16_t deviceAddr) 
{
	I2C2->CR2 = 0; // clear register
	int8_t data = 0;

	// Use SADD[7:1] bit field in CR2 register to set slave address to L3GD20
	I2C2->CR2 |= (deviceAddr << 1);
	// Use NBYTES[7:0] bit field to set number of data bytes to be transmitted to 1
	I2C2->CR2 |= (0x1 << 16);
	// Set RD_WRN to READ operation - 1 indicates READ
	I2C2->CR2 |= (1 << 10);
	// Set START bit to begin the address frame
	I2C2->CR2 |= I2C_CR2_START;
		
	// While RXNE or NACKF flags not set wait
	while (!(I2C2->ISR & (I2C_ISR_RXNE | I2C_ISR_NACKF))) {}
	// Once RXNE flag set continue
	
	// Check if NACK set
	if (I2C2->ISR & I2C_ISR_NACKF)
	{
		GPIOC->ODR |= GPIO_ODR_8; // ORANGE - I2C not working!
	}
		
	// Wait for TC flag set
	while (!(I2C2->ISR & I2C_ISR_TC)) {}
		
	// Read contents of RXDR register and return data - remember it is 1 byte at a time
	data = I2C2->RXDR;
	
	return data;
}

void I2C_SetUp()
{
	// PB11 -> SDA Line (data)
	// PB13 -> SCL Line (clock)
	// Enable I2C system clock using RCC register
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	// Configure bus timing using I2Cx_TIMINGR register to 100kHz standard-mode (taken from figure 5.4)
	I2C2->TIMINGR |= (0x1 << 28); // PRESC
	I2C2->TIMINGR |= (0x13 << 0); // SCLL
	I2C2->TIMINGR |= (0xF << 8); 	// SCHL
	I2C2->TIMINGR |= (0x2 << 16); // SDADEL
	I2C2->TIMINGR |= (0x4 << 20); // SCLDEL

	// Enable I2C peripheral using PE bit in CR1 register
	I2C2->CR1 |= I2C_CR1_PE;
}
