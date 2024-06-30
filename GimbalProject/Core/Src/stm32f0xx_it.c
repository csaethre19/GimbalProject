/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "USART.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "BLDCMotor.h"
#include "DCMotor.h"
#include "PWM_input.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
volatile int heyDMAinterruptcalled = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;
extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim17;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel 4, 5, 6 and 7 interrupts.
  */
void DMA1_Channel4_5_6_7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_5_6_7_IRQn 0 */

  /* USER CODE END DMA1_Channel4_5_6_7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c2_tx);
  HAL_DMA_IRQHandler(&hdma_i2c2_rx);
  /* USER CODE BEGIN DMA1_Channel4_5_6_7_IRQn 1 */
	
  /* USER CODE END DMA1_Channel4_5_6_7_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break, update, trigger and commutation interrupts.
  */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 0 */
	//PID_execute();
	Sample_MpuMoving();
  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 1 */

  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global and DAC channel underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	doPIDLoop();
	//Sample_YawSensor();
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM15 global interrupt.
  */
void TIM15_IRQHandler(void)
{
  /* USER CODE BEGIN TIM15_IRQn 0 */
	if (__HAL_TIM_GET_FLAG(&htim15, TIM_FLAG_CC1) != RESET) // Check if Capture Compare 1 interrupt occurred
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim15, TIM_IT_CC1) != RESET) // Check if the interrupt is enabled
        {
            uint32_t captureValue = HAL_TIM_ReadCapturedValue(&htim15, TIM_CHANNEL_1);
            if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_14) != GPIO_PIN_RESET)//Checking if logic level of PWM pin is high
            {
                // Positive edge occurred on channel 1
                //uint32_t timerValueBuffer = __HAL_TIM_GET_COUNTER(&htim15); // Get the timer value buffer
								process_eventTime(captureValue, 0, 1);
            }
            else
            {
                // Negative edge occurred on channel 1
                //uint32_t timerValueBuffer = __HAL_TIM_GET_COUNTER(&htim15); // Get the timer value buffer
                process_eventTime((int)captureValue, 1, 1);
            }
            __HAL_TIM_CLEAR_FLAG(&htim15, TIM_FLAG_CC1); // Clear the interrupt flag
        }
    }

    if (__HAL_TIM_GET_FLAG(&htim15, TIM_FLAG_CC2) != RESET) // Check if Capture Compare 2 interrupt occurred
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim15, TIM_IT_CC2) != RESET) // Check if the interrupt is enabled
        {
            uint32_t captureValue = HAL_TIM_ReadCapturedValue(&htim15, TIM_CHANNEL_2);
            if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_15) != GPIO_PIN_RESET)//Checking if logic level of PWM pin is high
            {
                // Positive edge occurred on channel 2
                //uint32_t timerValueBuffer = __HAL_TIM_GET_COUNTER(&htim15); // Get the timer value buffer
								process_eventTime((int)captureValue, 0, 2);
            }
            else
            {
                // Negative edge occurred on channel 2
                //uint32_t timerValueBuffer = __HAL_TIM_GET_COUNTER(&htim15); // Get the timer value buffer
                process_eventTime((int)captureValue, 1, 2);
            }
            __HAL_TIM_CLEAR_FLAG(&htim15, TIM_FLAG_CC2); // Clear the interrupt flag
        }
    }
  /* USER CODE END TIM15_IRQn 0 */
  HAL_TIM_IRQHandler(&htim15);
  /* USER CODE BEGIN TIM15_IRQn 1 */

  /* USER CODE END TIM15_IRQn 1 */
}

/**
  * @brief This function handles TIM17 global interrupt.
  */
void TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM17_IRQn 0 */
	if (__HAL_TIM_GET_FLAG(&htim17, TIM_FLAG_CC1) != RESET) // Check if Capture Compare 1 interrupt occurred
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim17, TIM_IT_CC1) != RESET) // Check if the interrupt is enabled
        {
            uint32_t captureValue = HAL_TIM_ReadCapturedValue(&htim17, TIM_CHANNEL_1);
            if (HAL_GPIO_ReadPin(GPIOB , GPIO_PIN_9) != GPIO_PIN_RESET)//Checking if logic level of PWM pin is high
            {
                // Positive edge occurred on channel 1 of tim17
                //uint32_t timerValueBuffer = __HAL_TIM_GET_COUNTER(&htim17); // Get the timer value buffer
                process_eventTime((int)captureValue, 0, 3);
            }
            else
            {
                // Negative edge occurred on channel 1 of tim17
                //uint32_t timerValueBuffer = __HAL_TIM_GET_COUNTER(&htim17); // Get the timer value buffer
                process_eventTime((int)captureValue, 1, 3);
            }
            __HAL_TIM_CLEAR_FLAG(&htim17, TIM_FLAG_CC1); // Clear the interrupt flag
        }
    }
	
  /* USER CODE END TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim17);
  /* USER CODE BEGIN TIM17_IRQn 1 */

  /* USER CODE END TIM17_IRQn 1 */
}

/**
  * @brief This function handles I2C2 global interrupt.
  */
void I2C2_IRQHandler(void)
{
  /* USER CODE BEGIN I2C2_IRQn 0 */

  /* USER CODE END I2C2_IRQn 0 */
  if (hi2c2.Instance->ISR & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR)) {
    HAL_I2C_ER_IRQHandler(&hi2c2);
  } else {
    HAL_I2C_EV_IRQHandler(&hi2c2);
  }
  /* USER CODE BEGIN I2C2_IRQn 1 */

  /* USER CODE END I2C2_IRQn 1 */
}


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
