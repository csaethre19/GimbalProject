#include <stdlib.h>
#include <stdio.h>

#include "USART.h"



void USART_SetUp() {
	// Configure pins PC4 and PC5 to alternate function mode:
	// PC4 -> USART_3TX (transmitter)
	// PC5 -> USART_3RX (receiver)
	// Use bit pattern for AF1 -> 0001
	// Using GPIOC_AFRL register
	// BAUD RATE: 115200
	
	GPIOC->MODER = (GPIOC->MODER & ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5)) 
								| GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1; 
								
	// Select appropriate function number in alternate function registers 
	GPIOC->AFR[0] |= 0x1 << GPIO_AFRL_AFRL4_Pos;
	GPIOC->AFR[0] |= 0x1 << GPIO_AFRL_AFRL5_Pos;
	
	// Enable system clock for USART3 in RCC peripheral
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	
	// Set Baud rate for communication to 115200 bits/second
	uint32_t clk_freq = HAL_RCC_GetHCLKFreq();
	USART3->BRR = clk_freq / 115200;
	
	// Enable transmitter and receiver hardware
	USART3->CR1 |= USART_CR1_TE;
	USART3->CR1 |= USART_CR1_RE;
	
	// Enable USART3
	USART3->CR1 |= USART_CR1_UE;
}

void USART_Transmit_Byte(uint8_t b)
{
	while (!(USART3->ISR & (1 << 7))) 
	{
		// Wait for data to be transferred to shift register - transmit register is empty
	}
	
	// Write the byte into the transmit data register
	USART3->TDR = b;
}

void USART_Transmit_Binary(uint8_t byte) 
	{
    for (int8_t i = 7; i >= 0; i--) {
        // Check each bit in the byte
        if (byte & (1 << i)) {
            USART_Transmit_Byte('1'); // If bit is set, send '1'
        } else {
            USART_Transmit_Byte('0'); // If bit is clear, send '0'
        }
    }
}

void USART_Transmit_String(const char* str)
{
    while (*str)
    {
        USART_Transmit_Byte((uint8_t)(*str));
        str++;
    }
}

void USART_Transmit_Number(int16_t number)
{
		int base = 10;
    char numberString[7]; // "-32768" + null terminator
    char* ptr = numberString, *ptr1 = numberString, tmp_char;
    int tmp_value;

    do {
        tmp_value = number;
        number /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - number * base)];
    } while ( number );

    // Apply negative sign
    if (tmp_value < 0) *ptr++ = '-';
    *ptr-- = '\0';
    while(ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
		
		USART_Transmit_String(numberString);
}

void USART_Transmit_Newline()
{
    USART_Transmit_Byte('\r'); // Carriage Return
    USART_Transmit_Byte('\n'); // Line Feed
}

void USART_Transmit_Float(float number, unsigned int decimal_places)
{
    char numberString[32]; // Buffer large enough for float representation
    // Convert the float to a string
    snprintf(numberString, sizeof(numberString), "%.*f", decimal_places, number);
    // Transmit the converted string
    USART_Transmit_String(numberString);
}

void Init_LEDs(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
	
	// Set pins to general purpose output mode in MODER register
  GPIOC->MODER |= (1<<12); // PC6 RED
	GPIOC->MODER |= (1<<14); // PC7 BLUE
  GPIOC->MODER |= (1<<16); // PC8 ORANGE
	GPIOC->MODER |= (1<<18); // PC9 GREEN
	
	// Set pins to push-pull output type in OTYPER register
	GPIOC->OTYPER &= ~(1<<6); // PC6
	GPIOC->OTYPER &= ~(1<<7); // PC7
	GPIOC->OTYPER &= ~(1<<8); // PC8
	GPIOC->OTYPER &= ~(1<<9); // PC9
	
	// Set pins to low speed in OSPEEDR register
	GPIOC->OSPEEDR &= ~(1<<12); // PC6
	GPIOC->OSPEEDR &= ~(1<<14); // PC7
	GPIOC->OSPEEDR &= ~(1<<16); // PC8
	GPIOC->OSPEEDR &= ~(1<<18); // PC9
	
	// Set to no pull-up/down resistors in PUPDR register
	GPIOC->PUPDR &= ~((1<<12) | (1<<13)); // PC6
	GPIOC->PUPDR &= ~((1<<14) | (1<<15)); // PC7
	GPIOC->PUPDR &= ~((1<<16) | (1<<17)); // PC8
	GPIOC->PUPDR &= ~((1<<18) | (1<<19)); // PC9
}