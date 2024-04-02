/*
* 	USART communication functionality.

*		PIN setup:
*	  PC4 -> USART_3TX (transmitter)
*	  PC5 -> USART_3RX (receiver)
*
*		STMCubeMX has been used to set up I2C. 
*		Transmit functions are the only things being used. 
*/
#include "main.h"


void Init_LEDs(void);

void USART_SetUp();

void USART_Transmit_Byte(uint8_t b);

void USART_Transmit_Binary(uint8_t byte);

void USART_Transmit_String(const char* str);

void USART_Transmit_Number(int16_t number);

void USART_Transmit_Float(float number, unsigned int decimal_places);

void USART_Transmit_Newline();