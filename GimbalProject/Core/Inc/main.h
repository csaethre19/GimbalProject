/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void PID_execute();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Yaw_CurrSense_Pin GPIO_PIN_1
#define Yaw_CurrSense_GPIO_Port GPIOC
#define Pitch_CurrSense_Pin GPIO_PIN_2
#define Pitch_CurrSense_GPIO_Port GPIOC
#define Roll_CurrSense_Pin GPIO_PIN_3
#define Roll_CurrSense_GPIO_Port GPIOC
#define BLDC_1A_Pin GPIO_PIN_0
#define BLDC_1A_GPIO_Port GPIOA
#define BLDC_1B_Pin GPIO_PIN_1
#define BLDC_1B_GPIO_Port GPIOA
#define BLDC_1C_Pin GPIO_PIN_2
#define BLDC_1C_GPIO_Port GPIOA
#define Yaw_ADC_Pin GPIO_PIN_3
#define Yaw_ADC_GPIO_Port GPIOA
#define Pitch_ADC_Pin GPIO_PIN_4
#define Pitch_ADC_GPIO_Port GPIOA
#define Roll_ADC_Pin GPIO_PIN_5
#define Roll_ADC_GPIO_Port GPIOA
#define BLDC_2A_Pin GPIO_PIN_6
#define BLDC_2A_GPIO_Port GPIOA
#define BLDC_2B_Pin GPIO_PIN_7
#define BLDC_2B_GPIO_Port GPIOA
#define BLDC_2C_Pin GPIO_PIN_0
#define BLDC_2C_GPIO_Port GPIOB
#define Yaw_PWMIN_Pin GPIO_PIN_14
#define Yaw_PWMIN_GPIO_Port GPIOB
#define Pitch_PWMIN_Pin GPIO_PIN_15
#define Pitch_PWMIN_GPIO_Port GPIOB
#define LED_Indicator_Pin GPIO_PIN_6
#define LED_Indicator_GPIO_Port GPIOC
#define DC_Ch1_Pin GPIO_PIN_8
#define DC_Ch1_GPIO_Port GPIOA
#define DC_Ch2_Pin GPIO_PIN_9
#define DC_Ch2_GPIO_Port GPIOA
#define BLDC_3C_Pin GPIO_PIN_10
#define BLDC_3C_GPIO_Port GPIOA
#define EN_BLDC3_Pin GPIO_PIN_11
#define EN_BLDC3_GPIO_Port GPIOA
#define EN_BLDC2_Pin GPIO_PIN_5
#define EN_BLDC2_GPIO_Port GPIOB
#define EN_BLDC1_Pin GPIO_PIN_6
#define EN_BLDC1_GPIO_Port GPIOB
#define Roll_PWMIN_Pin GPIO_PIN_9
#define Roll_PWMIN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
