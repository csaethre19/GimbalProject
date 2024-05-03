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
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "USART.h"
#include "MPU6050.h"
#include "I2C.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "BLDCMotor.h"
#include "DCMotor.h"
#include "PWM_input.h"
#include "HMC5883.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RX_BUFFER_SIZE 128
#define CMD_BUFFER_SIZE 16 // Adjust as necessary for your command length
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t rx_data[RX_BUFFER_SIZE];
volatile uint32_t rx_index = 0;
char cmdBuffer[CMD_BUFFER_SIZE];
uint32_t cmdBufferPos = 0;
volatile MPU6050_t mpu_moving;
volatile MPU6050_t mpu_stationary;
volatile HMC5883_t mag_moving;
volatile int usePWM;//usePWM decides if PWM determines desired angles
volatile int useADC;//useADC decides if Analog input determines desired angles
volatile int doPID = 0;
volatile int pitch_PWM;
volatile int roll_PWM;
volatile int yaw_PWM;
volatile int BurstReadState = 0;
volatile int mpu_moving_newdata = 0;
volatile int mpu_moving_readburstcheapcalled = 0;
volatile int8_t bufferData;
volatile double FREquencycounter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void init_PitchMotor();
void init_RollMotor();
void init_YawMotor();

void enablePWMIN();
void disablePWMIN();
int map(int value, int fromLow, int fromHigh, int toLow, int toHigh);
int constrain(int value, int minVal, int maxVal);
void Custom_StartupRoutine();
void Sample_MpuMoving();
void BurstReadCheap_StateMachine();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_ADC_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM15_Init();
  MX_TIM17_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
	Custom_StartupRoutine();
	uint32_t prevtime;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(doPID){
			BLDC_PID(&mpu_moving, &mpu_stationary);
			//PID_execute();
			doPID = 0;
		}
		if(mpu_moving_newdata){
			mpu_moving_newdata = 0;
			KFilter_2(&mpu_moving);
		}

		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
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

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x0000020B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
	//this line doesnt do shit
	//HAL_NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 3, 0);
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
	//HAL_NVIC_SetPriority(TIM1_IRQn, 2,2);
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 3;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 79;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */
	//HAL_NVIC_SetPriority(TIM15_IRQn, 2, 0);
  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 7;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */
	//HAL_NVIC_SetPriority(TIM17_IRQn, 2, 0);
  /* USER CODE END TIM17_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 7;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim17, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Indicator_GPIO_Port, LED_Indicator_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(EN_BLDC3_GPIO_Port, EN_BLDC3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EN_BLDC2_Pin|EN_BLDC1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Indicator_Pin */
  GPIO_InitStruct.Pin = LED_Indicator_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Indicator_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_BLDC3_Pin */
  GPIO_InitStruct.Pin = EN_BLDC3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EN_BLDC3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_BLDC2_Pin EN_BLDC1_Pin */
  GPIO_InitStruct.Pin = EN_BLDC2_Pin|EN_BLDC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief This function handles USART3 and USART4 global interrupts.
  */
void USART3_4_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_4_IRQn 0 */
	double value = 0;
	char* endPtr;
	
	if ((USART3->ISR & USART_ISR_RXNE) != 0) {
		char ch = USART3->RDR;
		USART_Transmit_Byte(ch);
		
		if (ch == '\r') // Command delimiter
    {
				cmdBuffer[cmdBufferPos] = '\0'; // Null-terminate the string
					//processCommand(cmdBuffer); // Process the buffered command
			//BEGIN: ANGLE CONTROL, RETRIEVE CURRENT ANGLE----------------------------------------------
				if (strcmp(cmdBuffer, "p") == 0){
					USART_Transmit_Float(mpu_moving.KalmanAnglePitch, 2);
					USART_Transmit_Newline();
				}
				else if(strcmp(cmdBuffer, "r") == 0){
					USART_Transmit_Float(mpu_moving.KalmanAngleRoll, 2);				
					USART_Transmit_Newline();
				}
				else if(strcmp(cmdBuffer, "y") == 0){

					USART_Transmit_Float(mpu_moving.KalmanAngleYaw, 2);
					USART_Transmit_Newline();
					}//BEGIN: ANGLE CONTROL, SET TARGET ANGLE-------------------------------------------
				else if(strncmp(cmdBuffer, "P", 1) == 0){

					char* extractedString = &cmdBuffer[1];
					value = strtod(extractedString, &endPtr);
					if(endPtr != extractedString){
						USART_Transmit_String("P=");
						USART_Transmit_Float(value, 2);
						set_desiredPitch(value); 
						USART_Transmit_Newline();						
					}else{
					USART_Transmit_String("ParseFail");						
					}
				}
				else if(strncmp(cmdBuffer, "R", 1) == 0){

					char* extractedString = &cmdBuffer[1];
					value = strtod(extractedString, &endPtr);
					if(endPtr != extractedString){
						USART_Transmit_String("R=");
						USART_Transmit_Float(value, 2);
						set_desiredRoll(value);
						USART_Transmit_Newline();						
					}else{
						USART_Transmit_String("ParseFail");						
					}
				}
				else if(strncmp(cmdBuffer, "Y", 1) == 0){

					char* extractedString = &cmdBuffer[1];
					value = strtod(extractedString, &endPtr);
					if(endPtr != extractedString){
						USART_Transmit_String("Y=");
						USART_Transmit_Float(value, 2);
						set_desiredYaw(value);
						USART_Transmit_Newline();						
					}else{
						USART_Transmit_String("ParseFail");						
					}
				}//BEGIN: INPUT CONFIGURATION-------------------------------------------------------
				else if(strcmp(cmdBuffer, "pwm1") == 0){
					
						USART_Transmit_String("PWM_EN");
						USART_Transmit_Newline();
						enablePWMIN();
					
				}
				else if(strcmp(cmdBuffer, "pwm0") == 0){
					
						USART_Transmit_String("PWM_DIS");
						USART_Transmit_Newline();
						disablePWMIN();
					
				}///BEGIN MODE CONTROL-------------------------------------------(absolute==1  relative==0)
				else if(strcmp(cmdBuffer, "A1") == 0){
					set_operationModeYaw(1);
					USART_Transmit_String("Yaw=Abs");
					USART_Transmit_Newline();
				}
				else if(strcmp(cmdBuffer, "A0") == 0){
					set_operationModeYaw(0);
					USART_Transmit_String("Yaw=Rel");
					USART_Transmit_Newline();
				}
				else if(strcmp(cmdBuffer, "B1") == 0){
					set_operationModeRollPitch(1);
					USART_Transmit_String("RP=Abs");
					USART_Transmit_Newline();
				} 
				else if(strcmp(cmdBuffer, "B0") == 0){
					set_operationModeRollPitch(0);
					USART_Transmit_String("RP=Rel");
					USART_Transmit_Newline();
				} 
				else {
					// Command not recognized
					USART_Transmit_String("ERROR");
				}
						
          cmdBufferPos = 0; // Reset the buffer for the next command

		}
    else
    {
				cmdBuffer[cmdBufferPos++] = ch; // Store the character and move the position
    }
		
	}	
	USART3->ISR = 0;
  /* USER CODE END USART3_4_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_4_IRQn 1 */

  /* USER CODE END USART3_4_IRQn 1 */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  if (huart->Instance == USART3)
  {
    rx_index++; // Move to the next position in the buffer
    if (rx_index >= RX_BUFFER_SIZE)
    {
      // Buffer overflow handling
      // Reset index to start overwriting data
      rx_index = 0;
    }
    HAL_UART_Receive_IT(&huart3, &rx_data[rx_index], 1); // Restart UART reception with interrupt
  }
}

void init_PitchMotor()
{
  double PI = 3.1415926535897932;
	double Angle1 = 0;
	
	//Angle1 Conversion
	Angle1 = (double)Angle1 * PI;
	Angle1 = Angle1 / 180;
	Angle1 = sin(Angle1);

	Angle1 = Angle1 * 500;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
	Angle1 = Angle1 + 500;
	Angle1 = Angle1 * 0.7;

  TIM_OC_InitTypeDef sConfigOC;
  
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.Pulse = Angle1;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); 

	//sConfigOC.Pulse = Angle2;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

	//sConfigOC.Pulse = Angle3;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); 
	return;	
}

void init_RollMotor()
{
	double PI = 3.1415926535897932;
	double Angle1 = 0;
	
	//Angle1 Conversion
	Angle1 = (double)Angle1 * PI;
	Angle1 = Angle1 / 180;
	Angle1 = sin(Angle1);

	Angle1 = Angle1 * 500;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
	Angle1 = Angle1 + 500;

  TIM_OC_InitTypeDef sConfigOC;
  
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.Pulse = Angle1;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);  
	return;
}

void init_YawMotor(){
		int value = 1000;

    TIM_OC_InitTypeDef sConfigOC;
  
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.Pulse = 1000;
		//begin outputting dutycycle = 100% on DC_Ch1 & DC_Ch2
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  

    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  
		return;
}
	

void PID_execute(){
	//Update Desired Angles
	//USART CAN ALWAYS UPDATE DESIRED VALUES
	if(usePWM == 1){//PWM provides values from 1000 to 2000, map to these ranges
		int yawbuffer = provide_channel(1);
		int pitchbuffer  = provide_channel(2);
		int rollbuffer   = provide_channel(3);
		//filter out garbage values
		if(pitchbuffer < 900){}
		else if(pitchbuffer  > 2100){} 
		else  {
			pitchbuffer = map(pitchbuffer, 1000,2000,-70,70);
			pitch_PWM = pitchbuffer;
			set_desiredPitch(pitchbuffer);
		}
		if(rollbuffer < 900){}
		else if(rollbuffer  > 2100){} 
		else  {
			rollbuffer = map(rollbuffer, 1000,2000,-70,70);
			roll_PWM = rollbuffer;
			set_desiredRoll(rollbuffer);
		}
		if(yawbuffer < 900){}
		else if(yawbuffer  > 2100){} 
		else  {
			yawbuffer = map(yawbuffer, 1000,2000,-70,70);
			yaw_PWM = yawbuffer;
			set_desiredYaw(yawbuffer);
		}
		
		//CURRENT IMPLEMENTATION ISSUE:
		//PWM only read correctly like 1/50th of the attempts, we filter out the bad ones (hopefully), this needs to be resolved
		//set_desiredPitch(pitchbuffer);
		//set_desiredRoll(rollbuffer);
		//set_desiredYaw(yawbuffer);
	}

	//GPIOC->ODR ^= GPIO_ODR_6;
	//Sample new IMU data

	//KFilter_2(&mpu_moving);
	//KFilter_2(&mpu_stationary);
	
	//Yaw PID
	
	//KFilter_2(&mpu_moving);
	//KFilter_2(&mpu_stationary);
	//BLDC_PID(&mpu_moving, &mpu_stationary);
	
	//Pitch & Roll PID
	//doPID = 1;
	//BLDC_PID(&mpu_moving, &mpu_stationary);
	
}


// Map function implementation used in PID_execute()
int map(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}


// Constrain function implementation used in PID_execute()
int constrain(int value, int minVal, int maxVal) {
    if (value < minVal) {
        return minVal;
    } else if (value > maxVal) {
        return maxVal;
    } else {
        return value;
    }
}

void doPIDLoop() {
	doPID = 1;
}

void enablePWMIN(){
	usePWM = 1;

	HAL_TIM_Base_Start_IT(&htim15);//enable timer 15 interrupt (pwm rise/fall edge -> Yaw&Pitch PWM input)
	HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim17);//enable timer 17 interrupt (pwm rise/fall edge -> Roll PWM input)
	HAL_TIM_IC_Start_IT(&htim17, TIM_CHANNEL_1);
	
}

void disablePWMIN(){
	usePWM = 0;
	HAL_TIM_Base_Stop(&htim15);
	HAL_TIM_Base_Stop(&htim17);
}



//This function call initializes the usage all peripherals,
void Custom_StartupRoutine() {
	//External Data Init-----------------------------------------------
	HAL_Delay(500);
	HAL_I2C_Init(&hi2c2);
	//HAL_UART_Receive_IT(&huart3, &rx_data[rx_index], 1);
	//HMC5883_Init(&mag_moving);
	MPU_Init(&mpu_moving, 0x68);
	//MPU_Init(&mpu_stationary, 0x69);

	//INPUT MODE SETUP-------------------------------------------------
	disablePWMIN();
	
	//MOTOR Setup
	init_PitchMotor();
	init_RollMotor();
	init_YawMotor();
	BLDC_PID_Init();
	BLDCEnable(1);
	BLDCEnable(2);
	//BLDCDisable(1);
	//BLDCDisable(1);
	//initDCOutput(1);
	
	//BLDCDisable(2);
	//BLDCDisable(1);
	set_desiredRoll(0.0f);
	set_desiredPitch(0.0f);
	set_operationModeRollPitch(1);

	HAL_TIM_Base_Start_IT(&htim1);//enable timer 1 interrupt (1  khz frequency)
	HAL_TIM_Base_Start_IT(&htim6);//enable timer 6 interrupt (200 hz frequency)
}

void Sample_MpuMoving() {
	//KFilter_2(&mpu_moving);
	
	if((BurstReadState == 0) && (mpu_moving_newdata == 0)){
		I2C_BurstRead_Cheap(mpu_moving.deviceAddr, ACC_XOUT_HIGH, 14);
		BurstReadState = 1;//reading from mpu_moving
		mpu_moving_readburstcheapcalled++;
	}
	
}

/*
	I2C2 interrupt service routine
	This function acts as the interrupt service routine when I2C2 has an interupt, 
	Its current use is to assess the flag that caused the interrupt, and clear the flag
	
	Additionally, the function will move through a state machine representing the byte being
	received from the device, reading from the I2C_RXDR register and placing the data in the appropriate location

	Each path through the state machine receives a pre-determined number of bytes, such that the state machine can generate a NACK flag
	when it knows the last useful byte has been received.
	STATE LIST:
	1 - 15 = mpu_moving
	21 - 35 = mpu_stationary
	51 - 5X = Hall_sensor
*/
void BurstReadCheap_StateMachine(){
	
	//check for an I2C2 failure (NACKF)
	if (I2C2->ISR & I2C_ISR_NACKF){
					// If a NACK is received, exit with error
					return; // Error code for NACK
					if((BurstReadState > 0) & (BurstReadState < 16)){//Reading from MPU_Moving failed, report this error
						USART_Transmit_Byte(71);//transmit G = 0d071 to user --> Moving IMU not working
						BurstReadState = 0;
					}
	}
	if(I2C2->ISR & (I2C_ISR_RXNE)){//a byte is present in the read buffer, process it.
		bufferData = I2C2->RXDR;
		
		switch(BurstReadState){
			case(1):{
				mpu_moving.accel_xhigh = bufferData;
				BurstReadState++;
				return;
			}
			case(2):{
				mpu_moving.accel_xlow = bufferData;
				BurstReadState++;
				return;
			}
			case(3):{
				mpu_moving.accel_yhigh = bufferData;
				BurstReadState++;
				return;
			}
			case(4):{
				mpu_moving.accel_ylow = bufferData;
				BurstReadState++;
				return;
			}
			case(5):{
				mpu_moving.accel_zhigh = bufferData;
				BurstReadState++;
			}
			case(6):{
				mpu_moving.accel_zlow = bufferData;
				BurstReadState++;
				return;
			}
			case(7):{
				//bufferData contains Temperature High bits
				BurstReadState++;
				return;
			}
			case(8):{
				//bufferData contains Temperature Low bits
				BurstReadState++;
				return;
			}
			case(9):{
				BurstReadState++;
				return;
			}
			case(10):{
				mpu_moving.gyro_xhigh = bufferData;
				BurstReadState++;
				return;
			}
			case(11):{
				mpu_moving.gyro_xlow = bufferData;
				BurstReadState++;
				return;
			}
			case(12):{
				mpu_moving.gyro_yhigh = bufferData;
				BurstReadState++;
				return;
			}
			case(13):{
				mpu_moving.gyro_ylow = bufferData;
				BurstReadState++;
				return;
			}
			case(14):{
				mpu_moving.gyro_zhigh = bufferData;
				BurstReadState++;
				return;
			}
			case(15):{
				mpu_moving.gyro_zlow = bufferData;
				BurstReadState = 0;
				mpu_moving_newdata = 1;
				//last byte, set CR2_NACK
				I2C2->CR2 |= I2C_CR2_NACK;
				I2C2->CR2 |= I2C_CR2_STOP;
				return;
			}
			default:{ BurstReadState = 0; return;}
		}
		
		
	}
	return;
	
}
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
