#include "main.h"

void Init_LEDs(void);

void Enable_GPIO_Clks();

void USART_SetUp();

void USART_USART_Transmit_Byte(uint8_t b);

void USART_Transmit_String(const char* str);

void USART_Transmit_Number(int16_t number);

void USART_Transmit_Newline();