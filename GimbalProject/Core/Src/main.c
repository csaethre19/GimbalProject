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

#define MPU6050_ADDR   0x68
#define WHO_AM_I       0x75

#define PWR_MGMT_1     0x6B
#define GYRO_CONFIG    0x1B
#define ACC_CONFIG     0x1A
#define SMPLRT_DIV     0x19
#define CONFIG         0x1A

#define GYRO_XOUT_LOW  0x44
#define GYRO_XOUT_HIGH 0x43
#define GYRO_YOUT_LOW  0x46
#define GYRO_YOUT_HIGH 0x44
#define GYRO_ZOUT_LOW  0x47
#define GYRO_ZOUT_HIGH 0x48


int16_t ReadGyroData(int8_t deviceAddr, int8_t gyroLowAddr, int8_t gyroHighAddr);

void MPU_Init(void);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Configure the system clock */
  SystemClock_Config();
	
	Init_LEDs();
		
	I2C_SetUp();
	
	MPU_Init();
	
	int16_t data = ReadGyroData(MPU6050_ADDR, GYRO_XOUT_LOW, GYRO_XOUT_HIGH);

  while (1)
  {
		
			//HAL_Delay(10);

  }
}

void MPU_Init()
{
	// Check WHO_AM_I register
	I2C_SetRegAddress(MPU6050_ADDR, WHO_AM_I);
	int8_t whoAmI = I2C_ReadRegister(MPU6050_ADDR);
	int8_t expected_whoAmI = 0x68;
	if (whoAmI == expected_whoAmI) 	USART_Transmit_String("Successfully read WHO_AM_I"); 
	else USART_Transmit_String("Erorr: did not get expected WHO_AM_I"); 

	USART_Transmit_Newline();
	
	// Wake up device and set clock to 8MHz
	I2C_WriteRegister(MPU6050_ADDR, PWR_MGMT_1, 0x00);
	I2C_SetRegAddress(MPU6050_ADDR, PWR_MGMT_1);
	int8_t pwr_mgmt = I2C_ReadRegister(MPU6050_ADDR);

if (pwr_mgmt == 0) 
	{
		USART_Transmit_String("MPU6050 Awake!");
		USART_Transmit_Newline();
	}
	
	// Setting full-scale range to +-500 degress/sec
	I2C_WriteRegister(MPU6050_ADDR, GYRO_CONFIG, 0x08);
	I2C_SetRegAddress(MPU6050_ADDR, GYRO_CONFIG);
	int8_t gyro_config = I2C_ReadRegister(MPU6050_ADDR);
	if (gyro_config == 0x08) USART_Transmit_String("Successfully configured GYRO to 500 deg/s");
	
	USART_Transmit_Newline();
	
	// Set SMPRT_DIV register to get 1kHz sample rate
	I2C_WriteRegister(MPU6050_ADDR, SMPLRT_DIV, 0x07);
	I2C_SetRegAddress(MPU6050_ADDR, SMPLRT_DIV); 
	int8_t sample_rate = I2C_ReadRegister(MPU6050_ADDR);
	USART_Transmit_String("sample rate: ");
	USART_Transmit_Number(sample_rate);
	
	USART_Transmit_Newline();
	
	// Configure DLPF for balanced noise performance - setting to ~44Hz bandwidth
	I2C_WriteRegister(MPU6050_ADDR, CONFIG, 0x03);
	I2C_SetRegAddress(MPU6050_ADDR, GYRO_CONFIG);
	int8_t config = I2C_ReadRegister(MPU6050_ADDR);
	if (config == 0x03) USART_Transmit_String("Enabled digital low pass filter");
	
	USART_Transmit_Newline();
}

int16_t ReadGyroData(int8_t deviceAddr, int8_t gyroLowAddr, int8_t gyroHighAddr)
{
	I2C_SetRegAddress(MPU6050_ADDR, GYRO_XOUT_LOW); 
	int8_t low = I2C_ReadRegister(MPU6050_ADDR); 
	
	char xl_str[] = "GYRO_XOUT_LOW: ";
	USART_Transmit_String(xl_str);
	USART_Transmit_Number(low);
	USART_Transmit_Newline();
	
	I2C_SetRegAddress(MPU6050_ADDR, GYRO_XOUT_HIGH);
	int8_t high = I2C_ReadRegister(MPU6050_ADDR); 
	
	char xh_str[] = "GYRO_XOUT_HIGH: ";
	USART_Transmit_String(xh_str);
	USART_Transmit_Number(high);
	USART_Transmit_Newline();
	
	int16_t data = ((int16_t)high << 8) | (uint8_t)low;

	char xd_str[] = "GYRO_XOUT: ";
	USART_Transmit_String(xd_str);
	USART_Transmit_Number(data);
	USART_Transmit_Newline();

	return data;
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
