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
#include "AS5600.h"
#include "I2C.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "BLDCMotor.h"
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
volatile AS5600_t yaw_sense;
volatile HMC5883_t mag_moving;
volatile int usePWM;//usePWM decides if PWM determines desired angles
volatile int useADC;//useADC decides if Analog input determines desired angles
volatile int doPID = 0;
volatile int pitch_PWM;
volatile int roll_PWM;
volatile int yaw_PWM;
volatile int BurstReadState = 0;
volatile int mpu_moving_newdata = 0;
volatile int yaw_sense_newdata = 0;
volatile int8_t bufferData;
volatile double FREquencycounter = 0;
volatile int button_press_count;
volatile uint8_t i2c_queue = 0;
volatile int uselesscounter = 0;
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
void Calibration();
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
	FREquencycounter++;
	Custom_StartupRoutine();
	//LED flash #1, startup complete
	float counttracker = 0;
	while(counttracker < 15){//flashy light shows chip is functioning
		GPIOC->ODR |= GPIO_ODR_6;
		HAL_Delay(100);
		GPIOC->ODR &= ~GPIO_ODR_6;
		HAL_Delay(100);
		counttracker++;
	}
	
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	int dirtrack;
	counttracker =180;
  while (1)
  {
		//BLDC_Output(counttracker,2);//
		//counttracker += 5;

		//if(counttracker > 350) counttracker = 1;

		//HAL_Delay(1);
		
		if(doPID){
			BLDC_PID(&mpu_moving, &mpu_stationary);
			YAW_PID(&yaw_sense);
			PID_execute();
			doPID = 0;
		}
		
		if(mpu_moving_newdata){
			mpu_moving_newdata = 0;
			Mahony_update(&mpu_moving);
			
		}
		
		if(yaw_sense_newdata){
			yaw_sense_newdata = 0;
			AS5600_Process_Angle(&yaw_sense);
		}

		FREquencycounter++;
		
		//-----------------------I2C queue handling stuff---------------//
		if(i2c_queue > 5){
			i2c_queue = 0;
			Sample_YawSensor();
		}
		//------------------------BUTTON STUFF--------------------------//
		//Short Press < 3 seconds = LED lights
		//Long Press > 3 seconds = Calibration requested
		if(GPIOC->IDR &= GPIO_IDR_13){
			button_press_count--;
			GPIOC->ODR &= ~GPIO_ODR_6;
		}
		else{
			button_press_count++;
			GPIOC->ODR |= GPIO_ODR_6;
		}
		
		
		
		/*
		if(button_press_count > 2000){
			//wait for button to be released to begin calibration
			GPIOC->ODR |= GPIO_ODR_6;
			if(~(GPIOC->IDR &= GPIO_IDR_13)){
				if(BurstReadState == 0){
					BurstReadState = 999;//A blocking I2C operation is occuring, don't start another I2C operation
					AS5600_Set_Zero(&yaw_sense);
					BurstReadState = 0;
					//DO SOME  IMU CALIBRATION-----
					
					//LED flash #1, cal complete
					counttracker = 0;
					while(counttracker < 30){//flashy light shows chip is functioning
						GPIOC->ODR |= GPIO_ODR_6;
						HAL_Delay(50);
						GPIOC->ODR &= ~GPIO_ODR_6;
						counttracker++;
					}
				}

			}
		}
		*/
		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI48;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hi2c2.Init.Timing = 0x00200C28;
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
  htim1.Init.Prescaler = 23;
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
  htim6.Init.Prescaler = 239;
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
  htim15.Init.Prescaler = 23;
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
  htim17.Init.Prescaler = 23;
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

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
	if(cmdBufferPos > 50){cmdBufferPos = 0;}
	double value = 0;
	char* endPtr;
	
	if ((USART3->ISR & USART_ISR_RXNE) != 0) {
		char ch = USART3->RDR;
		//USART_Transmit_Byte(ch);
		
		if (ch == '\r') // Command delimiter
    {
				cmdBuffer[cmdBufferPos] = '\0'; // Null-terminate the string
					//processCommand(cmdBuffer); // Process the buffered command
			//BEGIN: ANGLE CONTROL, RETRIEVE CURRENT ANGLE----------------------------------------------
				if (strcmp(cmdBuffer, "p") == 0){
					//USART_Transmit_Float(mpu_moving.outRoll, 2);//Roll and pitch calculations are flipped in struct
					//USART_Transmit_Newline();
				}
				else if(strcmp(cmdBuffer, "r") == 0){
					//USART_Transmit_Float(mpu_moving.outPitch, 2);//Roll and pitch calculations are flipped in struct outpitch = roll value		
					//USART_Transmit_Newline();
				}
				else if(strcmp(cmdBuffer, "y") == 0){
					//FIX THIS--------------------------------------------------
					////USART_Transmit_Float(mpu_moving.KalmanAngleYaw, 2);
					//USART_Transmit_Newline();
					}//BEGIN: ANGLE CONTROL, SET TARGET ANGLE-------------------------------------------
				else if(strncmp(cmdBuffer, "P", 1) == 0){

					char* extractedString = &cmdBuffer[1];
					value = strtod(extractedString, &endPtr);
					if(endPtr != extractedString){
						//USART_Transmit_String("P=");
						//USART_Transmit_Float(value, 2);
						set_desiredPitch(value); 
						//USART_Transmit_Newline();						
					}else{
					//USART_Transmit_String("ParseFail");						
					}
				}
				else if(strncmp(cmdBuffer, "R", 1) == 0){

					char* extractedString = &cmdBuffer[1];
					value = strtod(extractedString, &endPtr);
					if(endPtr != extractedString){
						//USART_Transmit_String("R=");
						//USART_Transmit_Float(value, 2);
						set_desiredRoll(value);
						//USART_Transmit_Newline();						
					}else{
						//USART_Transmit_String("ParseFail");						
					}
				}
				else if(strncmp(cmdBuffer, "Y", 1) == 0){

					char* extractedString = &cmdBuffer[1];
					value = strtod(extractedString, &endPtr);
					if(endPtr != extractedString){
						//USART_Transmit_String("Y=");
						//USART_Transmit_Float(value, 2);
						set_desiredYaw(value);
						//USART_Transmit_Newline();						
					}else{
						//USART_Transmit_String("ParseFail");						
					}
				}//BEGIN: INPUT CONFIGURATION-------------------------------------------------------
				else if(strcmp(cmdBuffer, "pwm1") == 0){
					
						//USART_Transmit_String("PWM_EN");
						//USART_Transmit_Newline();
						enablePWMIN();
					
				}
				else if(strcmp(cmdBuffer, "pwm0") == 0){
					
						//USART_Transmit_String("PWM_DIS");
						//USART_Transmit_Newline();
						disablePWMIN();
					
				}///BEGIN MODE CONTROL-------------------------------------------(absolute==1  relative==0)
				else if(strcmp(cmdBuffer, "A1") == 0){
					set_operationModeYaw(1);
					//USART_Transmit_String("Yaw=Abs");
					//USART_Transmit_Newline();
				}
				else if(strcmp(cmdBuffer, "A0") == 0){
					set_operationModeYaw(0);
					//USART_Transmit_String("Yaw=Rel");
					//USART_Transmit_Newline();
				}
				else if(strcmp(cmdBuffer, "B1") == 0){
					set_operationModeRollPitch(1);
					//USART_Transmit_String("RP=Abs");
					//USART_Transmit_Newline();
				} 
				else if(strcmp(cmdBuffer, "B0") == 0){
					set_operationModeRollPitch(0);
					//USART_Transmit_String("RP=Rel");
					//USART_Transmit_Newline();
				} 
				else {
					// Command not recognized
					//USART_Transmit_String("ParseFail");
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
  TIM_OC_InitTypeDef sConfigOC;
  
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.Pulse = 30;
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
  TIM_OC_InitTypeDef sConfigOC;
  
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.Pulse = 30;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);  

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);  
	return;
}
void init_YawMotor()
{
  TIM_OC_InitTypeDef sConfigOC;
  
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.Pulse = 30;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  

  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  

  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  
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
			yawbuffer = map(yawbuffer, 1000,2000,0,360);
			if(yawbuffer > 355) yawbuffer = 355;
			if(yawbuffer < 5) yawbuffer = 5;
			yaw_PWM = yawbuffer;
			set_desiredYaw(yawbuffer);
		}

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
	FREquencycounter++;
	HAL_I2C_Init(&hi2c2);
	FREquencycounter++;
	HAL_UART_Receive_IT(&huart3, &rx_data[rx_index], 1);
	FREquencycounter++;
	//while (uselesscounter < 10000000){uselesscounter++;}
	//HMC5883_Init(&mag_moving);
	MPU_Init(&mpu_moving, 0x68);
	FREquencycounter++;
	AS5600_Init(&yaw_sense, 0x36);
	FREquencycounter++;
	//---------------------DEFAULT Disable PWM control----------------------//
	disablePWMIN();
	//enablePWMIN();
	FREquencycounter++;
	//----------//-Deliver Power to Motors, in some default state------------//
	init_PitchMotor();
	init_RollMotor();
	init_YawMotor();
	FREquencycounter++;
	//---------------------Setup PID controller for brushless motors--------//
	BLDC_PID_Init();
	
	//--------------------------Enable Power to the Motors------------------//
	//
	//BLDCEnable(1);   //pitch = 1
	//BLDCEnable(2); //roll = 2
	//BLDCEnable(3); //yaw = 3
	//
	
	//---------------------------Disable Power to the Motors-----------------//
	///*
	BLDCDisable(1);
	BLDCDisable(2);
	BLDCDisable(3);
	//*/
	//----------------DEFAULT TO CENTERED PAYLOAD ORIENTATION---------------------//
	set_desiredRoll(0.0f);
	set_desiredPitch(0.0f);
	set_desiredYaw(40.0f);
	
	//----------------DEFAULT Roll & Pitch & Yaw to Aboslute Angle----------------//
	set_operationModeRollPitch(1);
	set_operationModeYaw(1);
	
	//----------------------ENABLE MOTOR PID----------------------//
	HAL_TIM_Base_Start_IT(&htim1);//enable timer 1 interrupt (1  khz frequency)
	//----------------------ENABLE MPU_MOVING SAMPLE--------------//
	HAL_TIM_Base_Start_IT(&htim6);//enable timer 6 interrupt (200 hz frequency)
}

void Sample_MpuMoving() {
	//KFilter_2(&mpu_moving);
	
	if((BurstReadState == 0) && (mpu_moving_newdata == 0)){
  	I2C_BurstRead_Cheap(mpu_moving.deviceAddr, ACC_XOUT_HIGH, 14);
		BurstReadState = 1;//reading from mpu_moving
	}
	else{i2c_queue |= 0b00000010;}//if second bit is set, do a sample mpu_moving asap
	
}

void Calibrate(){
	AS5600_Set_Zero(&yaw_sense);
}

void Sample_YawSensor() {
	
	if((BurstReadState == 0) && (yaw_sense_newdata == 0)){
		I2C_BurstRead_Cheap(yaw_sense.deviceAddr, ANG_h, 2);
		BurstReadState = 51;//reading from mpu_moving
	}
	else{i2c_queue |= 0b00000001;}//if first bit is set, do a sample yawsensor asap
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
	51 - 52 = Yaw Angle(Hall)_sensor
*/
void BurstReadCheap_StateMachine(){
	
	//check for an I2C2 failure (NACKF)
	if (I2C2->ISR & I2C_ISR_NACKF){
					// If a NACK is received, exit with error
					return; // Error code for NACK
					if((BurstReadState > 0) & (BurstReadState < 16)){//Reading from MPU_Moving failed, report this error
						//USART_Transmit_Byte(71);//transmit G = 0d071 to user --> Moving IMU not working
						BurstReadState = 0;
					}
	}
	if(I2C2->ISR & (I2C_ISR_RXNE)){//a byte is present in the read buffer, process it.
		bufferData = I2C2->RXDR;
		
		switch(BurstReadState){
			case(1):{//------------IMU Moving-----------//
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
				i2c_queue ++;
				//last byte, set CR2_NACK
				I2C2->CR2 |= I2C_CR2_NACK;
				I2C2->CR2 |= I2C_CR2_STOP;
				
				return;
			}
			case(51):{//----------AS5600 ANGLE Readout-----------//
				yaw_sense.angle_high = bufferData;
				BurstReadState++;
				return;
			}
			case(52):{
				yaw_sense.angle_low = bufferData;
				BurstReadState = 0;
				yaw_sense_newdata = 1;
				//last byte, set CR2_NACK
				I2C2->CR2 |= I2C_CR2_NACK;
				I2C2->CR2 |= I2C_CR2_STOP;
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
