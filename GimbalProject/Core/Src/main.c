/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************

Includes ------------------------------------------------------------------*/
#include "main.h"

#include "I2C.h"
#include "USART.h"

void SystemClock_Config(void);

#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define WHO_AM_I     0x75

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Configure the system clock */
  SystemClock_Config();
	
	// TODO: Wire up IMU sensor -> VCC to 3V, GND to GND, SDA to STM-SDA, SCL to STM-SCL
	Enable_GPIO_Clks();
	
	Init_LEDs();
	
	I2C_Ports_Config();
	
	I2C_SetUp();
	
	I2C_SetRegAddress(MPU6050_ADDR, WHO_AM_I); // Write WHO_AM_I address 
	
	int8_t data = I2C_ReadRegister(MPU6050_ADDR); // Read from WHO_AM_I register
	
	int8_t expected_whoAmI = 0x68;
	int8_t expected_pwrMgmt = 0x40;

	if (data == expected_whoAmI) 	GPIOC->ODR |= GPIO_ODR_7; // SUCCESS - set blue LED HIGH
	else GPIOC->ODR |= GPIO_ODR_6; // FAILURE - set red LED HIGH
	
	// Taking device out of sleep mode
	I2C_WriteRegister(MPU6050_ADDR, PWR_MGMT_1, 0x00);
	I2C_SetRegAddress(MPU6050_ADDR, PWR_MGMT_1);
	int8_t pwr_mgmt = I2C_ReadRegister(MPU6050_ADDR);
	
	if (pwr_mgmt == 0) 
	{
		USART_Transmit_String("MPU6050 Awake!");
		USART_Transmit_Newline();
	}
	
	// TODO: Set digital low pass filter for noise performance using register 0x1A (CONFIG register)

	// All axes for gyro and acc are enabled by default - once device is not in sleep mode anymore, we can start reading
	
	// TODO: Read and print data for Gyroscope and Accelerometer to PuTTY console via USART

  while (1)
  {

  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
