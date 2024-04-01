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
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 128
#define Yaw_Motor 1;
#define Pitch_Motor 1;

//EVENTUALLY INCLUDED IN OTHER FILES< HERE FOR NOW
//BLDCMotor defines
#define PI 3.1415926535897932;
#define Pitch_TimARR 1000;
#define Roll_TimARR 1000;
#define Pitch 1;
#define Roll 2;
#define Pitch_1A TIM2->CCR1;
#define Pitch_1B TIM2->CCR2;
#define Pitch_1C TIM2->CCR3;
#define Roll_2A TIM3->CCR1;
#define Roll_2B TIM3->CCR2;
#define Roll_2C TIM3->CCR3;
//DCmotor defines
#define Yaw1_TimARR 1000;
#define Yaw 1;
#define Yaw1_Ch1 TIM1->CCR1;
#define Yaw1_Ch2 TIM1->CCR2;

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

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t rx_data[RX_BUFFER_SIZE];
volatile uint32_t rx_index = 0;
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
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void DCSetOutput(int Output, int MotorNum);
void initDCOutput(int MotorNum);
void BLDC_Output(double Angle1, int MotorNum);
void initBLDCOutput(int MotorNum);
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
  /* USER CODE BEGIN 2 */
	Init_LEDs();
	//Setup Motor Drivers
	//Motor Setup (Yaw & Pitch for now)
	initDCOutput( 1 );//initialize a state for Yaw Motor
	initBLDCOutput( 1 );//initialize a state for Pitch Motor
	
	
	
	//NVIC_SetPriority(USART3_4_IRQn, 1);
	//USART_Transmit_String("Hi");
	//USART_Transmit_Newline();
	
	 HAL_UART_Receive_IT(&huart3, &rx_data[rx_index], 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	int DCtracker = 0;
	int DC_Direction = 1;
	double BLDCtracker = 0;
  while (1)
  {
		GPIOC -> ODR ^= GPIO_ODR_6;
		//DCSetOutput(1000,1);
		//BLDC_Output(10,1);
		HAL_Delay(250);
		GPIOC -> ODR ^= GPIO_ODR_6;
		//DCSetOutput(500,1);
		//BLDC_Output(50,1);
		HAL_Delay(250);
		GPIOC -> ODR ^= GPIO_ODR_6;
		//DCSetOutput(-100,1);
		//BLDC_Output(100,1);
		HAL_Delay(250);
		GPIOC -> ODR ^= GPIO_ODR_6;
		//DCSetOutput(500,1);
		//BLDC_Output(500,1);
		HAL_Delay(250);
		GPIOC -> ODR ^= GPIO_ODR_6;
		TIM1->CCR1 = 500;
		TIM1->CCR2 = 500;
		TIM2->CCR1 = 500;
		TIM2->CCR2 = 500;
		TIM2->CCR3 = 500;
		//DCSetOutput(500,1);
		//BLDC_Output(350,1);
		HAL_Delay(250);
		
		/*
		DCSetOutput(DCtracker, 1);
		BLDC_Output(BLDCtracker, 1);
		
		DCtracker += DC_Direction;
		BLDCtracker += 1;
		if((DCtracker > 999) || (DCtracker < 999)) {DC_Direction -= 2 * DC_Direction; GPIOC -> ODR ^= GPIO_ODR_6;}
		if(BLDCtracker > 359.99) BLDCtracker = 0;
		HAL_Delay(10);
		/*
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
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_11;
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
  htim2.Init.Prescaler = 7;
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
  htim3.Init.Prescaler = 7;
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

  /*Configure GPIO pin : LED_Indicator_Pin */
  GPIO_InitStruct.Pin = LED_Indicator_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Indicator_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//BLDC & DC MOTOR METHODS TEMPORARY LOCATION
/*
	DCSetOutput takes in a value from -1000 to 1000
	Based on this, the DC motor is set to rotate CW or CCW from 0% to 100% power
*/
void DCSetOutput(int Output, int MotorNum)
{
	if((Output < -1000) || (Output > 1000)) return;
	
	if(MotorNum == 1){//Yaw = DC_1
		if(Output > 0){//CW? Direction instructed, Ch2 not 100% duty cycle
			TIM1->CCR1 = 1000;
			//Max Spin Rate = Yaw1_TimARR * 0      ie. Output = 1000 -> (1 - (Output / 1000)) = 0
			//Min Spin Rate = Yaw1_TimARR * 0.999  ie. Output = 0001 -> (1 - (Output / 1000)) = 0.999
			TIM1->CCR2 = 1000 * (1 - (Output / 1000)); 
		}
		else if(Output < 0){//CCW? Direction instructed, Ch1 not 100% duty cycle
			TIM1->CCR2 = 1000;
			//Max Spin Rate = Yaw1_TimARR * 0      ie. Output = 1000 -> (1 - (Output / 1000)) = 0
			//Min Spin Rate = Yaw1_TimARR * 0.999  ie. Output = 0001 -> (1 - (Output / 1000)) = 0.999
			TIM1->CCR1 = 1000 * (1 - (Output / 1000)); 
		}
		else{//Output = 0
			TIM1->CCR1 = 1000;
			TIM1->CCR2 = 1000;
		}
	}
}

void initDCOutput(int MotorNum)
{
	if(MotorNum == 1){//put the provided DC motor into its brake state;
		TIM1->CCR1 = 1000;//Duty Cycle = 100%
		TIM1->CCR2 = 1000;//Duty Cycle = 100%
	}
}

/*
	This implementation of BLDCMotor control takes in a value from 0-359 as the "desired angle"
	The determination of what this "desired angle" is currently intended to be performed elsewhere
	This code simply drives the signals delivered to the motors,
	as well as defaults to some hopefully harmless power up values
	
	PLANNED ADDITIONS:
	1. Handle the PID loop through simple function calls within this method
	2. Deal with the current measurement appropriately (Pitch/Roll Current ADC input)
	3.?
*/

/*
	Takes in a double Angle1 and a motor number and modifies the PWM outputs to the respective BLDC motor
*/
void BLDC_Output(double Angle1, int MotorNum)
{
	if((Angle1 < 0) || (Angle1 > 360)) return;
	
	int Angle2 = Angle1 + 120;
	int Angle3 = Angle1 + 240;
	Angle1 = fmod(Angle1, 360);
	Angle2 = fmod(Angle2, 360);
	Angle2 = fmod(Angle3, 360);
	
	
	//Angle1 Conversion
	Angle1 = (double)Angle1 * PI;
	Angle1 = Angle1 / 180;
	Angle1 = sin(Angle1);
	//Angle2 Conversion
	Angle2 = (double)Angle2 * PI;
	Angle2 = Angle2 / 180;
	Angle2 = sin(Angle2);
	//Angle3 Conversion
	Angle3 = (double)Angle3 * PI;
	Angle3 = Angle3 / 180;
	Angle3 = sin(Angle3);

	
	if(MotorNum == 1){
		Angle1 = Angle1 * Pitch_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle1 = Angle1 + Pitch_TimARR;
		Angle2 = Angle2 * Pitch_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle2 = Angle2 + Pitch_TimARR;
		Angle3 = Angle3 * Pitch_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle3 = Angle3 + Pitch_TimARR;
		//0.7 chosen to decrease amount of power delivered to motor, adjust after testing
		TIM2->CCR1 = Angle1 * 0.7;
		TIM2->CCR2 = Angle2 * 0.7;
		TIM2->CCR3 = Angle3 * 0.7;
		return;
	}
	else if(MotorNum == 2){
		Angle1 = Angle1 * Roll_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle1 = Angle1 + Roll_TimARR;
		Angle2 = Angle2 * Roll_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle2 = Angle2 + Roll_TimARR;
		Angle3 = Angle3 * Roll_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle3 = Angle3 + Roll_TimARR;
		TIM3->CCR1 = Angle1 * 0.7;
		TIM3->CCR2 = Angle2 * 0.7;
		TIM3->CCR3 = Angle3 * 0.7;
		return;
	}
	
	return;
}

/*
Give some random initial value to the Duty cycle of the BLDC motors
*/
void initBLDCOutput(int MotorNum)
{
	int Angle1 = 0;
	int Angle2 = Angle1 + 120;
	int Angle3 = Angle1 + 240;
	
	//Angle1 Conversion
	Angle1 = (double)Angle1 * PI;
	Angle1 = Angle1 / 180;
	Angle1 = sin(Angle1);
	//Angle2 Conversion
	Angle2 = (double)Angle2 * PI;
	Angle2 = Angle2 / 180;
	Angle2 = sin(Angle2);
	//Angle3 Conversion
	Angle3 = (double)Angle3 * PI;
	Angle3 = Angle3 / 180;
	Angle3 = sin(Angle3);

	
	if(MotorNum == 1){
		Angle1 = Angle1 * Pitch_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle1 = Angle1 + Pitch_TimARR;
		Angle2 = Angle2 * Pitch_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle2 = Angle2 + Pitch_TimARR;
		Angle3 = Angle3 * Pitch_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle3 = Angle3 + Pitch_TimARR;	
		TIM2->CCR1 = Angle1 * 0.7;
		TIM2->CCR2 = Angle2 * 0.7;
		TIM2->CCR3 = Angle3 * 0.7;
		return;
	}
	else if(MotorNum == 2){
		Angle1 = Angle1 * Roll_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle1 = Angle1 + Roll_TimARR;
		Angle2 = Angle2 * Roll_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle2 = Angle2 + Roll_TimARR;
		Angle3 = Angle3 * Roll_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle3 = Angle3 + Roll_TimARR;
		TIM3->CCR1 = Angle1 * 0.7;
		TIM3->CCR2 = Angle2 * 0.7;
		TIM3->CCR3 = Angle3 * 0.7;
		return;
	}
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
