#include <stdlib.h>

#include "I2C.h"
#include "USART.h"



void I2C_SetRegAddress(uint16_t deviceAddr, uint8_t regAddr)
{
	//USART_Transmit_String("SET REG deviceAddr: ");
	//USART_Transmit_Number(deviceAddr);
	//USART_Transmit_Newline();
	
	//USART_Transmit_String("SET REG regAddr: ");
	//USART_Transmit_Number(regAddr);
	//USART_Transmit_Newline();
	
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
		GPIOC->ODR |= GPIO_ODR_8; // Orange - I2C not working!
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
	//USART_Transmit_String("READ deviceAddr: ");
	//USART_Transmit_Number(deviceAddr);
	//USART_Transmit_Newline();
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
		
	//USART_Transmit_String("data: ");
	//USART_Transmit_Number(data);
	//USART_Transmit_Newline();
	
	return data;
}

void I2C_SetUp()
{
	I2C_Ports_Config();
	
	USART_SetUp(); // for debugging
	
	// PB11 -> SDA Line (data)
	// PB13 -> SCL Line (clock)
	// Enable I2C system clock using RCC register
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	
	// Configure bus timing using I2Cx_TIMINGR register to 400kHz highspeed-mode (taken from figure 5.4)
	I2C2->TIMINGR |= (0x0 << 28); // PRESC
	I2C2->TIMINGR |= (0x9 << 0);  // SCLL
	I2C2->TIMINGR |= (0x3 << 8); 	// SCHL
	I2C2->TIMINGR |= (0x1 << 16); // SDADEL
	I2C2->TIMINGR |= (0x3 << 20); // SCLDEL

	// Enable I2C peripheral using PE bit in CR1 register
	I2C2->CR1 |= I2C_CR1_PE;
}

void I2C_Ports_Config() 
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// Set PB11 to alt func mode (and PB13)
	GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER13)) 
								| GPIO_MODER_MODER11_1 | GPIO_MODER_MODER13_1; 
	
	// set PB11 to open-drain output type using OTYPER reg
	GPIOB->OTYPER |= (1 << 11);
	// Select I2C2_SDA as alt func (using AFR register bit pattern: AF1=0001)
	GPIOB->AFR[1] |= 0x1 << GPIO_AFRH_AFSEL11_Pos;
	
	// Set PB13 to alt func mode (done above)
	// set PB13 to open-drain output type
	GPIOB->OTYPER |= (1 << 13);
	// Select I2C2_SCL as alt func (using AFR register bit pattern: AF5=0101)
	GPIOB->AFR[1] |= 0x5 << GPIO_AFRH_AFSEL13_Pos;
	
	// Set PB14 to output mode, push-pull output type
	GPIOB->MODER |= (1 << 28);
	GPIOB->OTYPER &= ~(1 << 14);
	// set HIGH
	GPIOB->ODR |= GPIO_ODR_14;
	
	// Set PC0 to output mode, push-pull output type - what were we using PC0 for in lab5?
	GPIOC->MODER |= (1 << 0);
	GPIOC->OTYPER &= ~(1 << 0);
	// set HIGH
	GPIOC->ODR |= GPIO_ODR_0;
}
