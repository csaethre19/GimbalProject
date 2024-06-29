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
DMA_HandleTypeDef hdma_i2c2_rx;
<<<<<<< Updated upstream
=======
DMA_HandleTypeDef hdma_i2c2_tx;
>>>>>>> Stashed changes

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
<<<<<<< Updated upstream
volatile int doPIDCount = 0;
volatile int PID_execcount = 0;
volatile char I2C2_DMA_state = 0;
uint8_t I2C2_DMA_buffer[14];//this buffer is the memory address
volatile char Process_mpu_moving = 0;
volatile char mpu_moving_kfilter_complete = 1;
=======
volatile int BurstReadState = 0;
volatile int mpu_moving_newdata = 0;
volatile int yaw_sense_newdata = 0;
volatile int8_t bufferData;
volatile double FREquencycounter = 0;
volatile int button_press_count;

uint8_t I2C2_txBuffer[10];
uint8_t I2C2_rxBuffer[30];
volatile uint8_t i2c_queue = 0;




>>>>>>> Stashed changes
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
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
void enableADCIN();
void disableADCIN();
int map(int value, int fromLow, int fromHigh, int toLow, int toHigh);
int constrain(int value, int minVal, int maxVal);

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
  MX_DMA_Init();
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
	HAL_I2C_Init(&hi2c2);
	Init_LEDs();

	//HAL_UART_Receive_IT(&huart3, &rx_data[rx_index], 1);
	//HMC5883_Init(&mag_moving);
	MPU_Init(&mpu_moving, 0x68);
	//MPU_Init(&mpu_stationary, 0x69);

	
	//INPUT MODE SETUP
	disablePWMIN();
	disableADCIN();


	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	

	
	//MOTOR Setup
	init_YawMotor();
	initDCOutput(1);
	init_PitchMotor();
	init_RollMotor();
	//BLDCEnable(2);
	//BLDCEnable(1);
	
	BLDCDisable(2);
	BLDCDisable(1);
	set_desiredRoll(0.0f);
	set_desiredPitch(0.0f);
	set_operationMode(1);
	HAL_TIM_Base_Start_IT(&htim1);//timer 1 initiates grabbing data from MPU6050 devices
	//HAL_TIM_Base_Start_IT(&htim6);//timer 6 interrupt service routine calls PID_execute;
	//MOTOR TESTING CODE
	int DCtracker = 0;
	int DC_Direction = 5;
	double BLDCtracker = 1;
	int BLDC_Direction = 5;
	//BLDC_Output(1,1);
  while (1)
  {
		doPIDCount++;
		//BLDC_Output(BLDCtracker,1);
		//BLDCtracker += 1;
		//if(BLDCtracker > 360){ BLDCtracker = 1;};
		//HAL_Delay(100);
		if(doPID == 1){
			//KFilter_2(&mpu_moving);
			//KFilter_2(&mpu_stationary);
			
			BLDC_PID(&mpu_moving, &mpu_stationary);
			doPIDCount ++;
			doPID = 0;
			
			//DCSetOutput(DCtracker, 1);

			//DCtracker += DC_Direction;
			//BLDCtracker += BLDC_Direction;
			//if((DCtracker > 999) || (DCtracker < -999)) {DC_Direction -= 2 * DC_Direction;}
		}
		
		if(Process_mpu_moving == 1){
			//Put newly retrieved accel data into moving_MPU6050 struct
			//raise flag indicating required data for Kalman filter is ready
			uint8_t xahigh;
			uint8_t xalow;
			uint8_t yahigh;
			uint8_t yalow;
			uint8_t zahigh;
			uint8_t zalow;
			int16_t xa_raw;
			int16_t ya_raw;
			int16_t za_raw;
			
			xahigh = I2C2_DMA_buffer[0];
			xalow = I2C2_DMA_buffer[1];
			xa_raw = (int16_t)(xahigh << 8 | xalow);
			yahigh = I2C2_DMA_buffer[2];
			yalow = I2C2_DMA_buffer[3];
			ya_raw = (int16_t)(yahigh << 8 | yalow);
			zahigh = I2C2_DMA_buffer[4];
			zalow = I2C2_DMA_buffer[5];
			za_raw = (int16_t)(zahigh << 8 | zalow);
	
			mpu_moving.Accel_X_RAW = xa_raw;
			mpu_moving.Ax = (float)xa_raw/ACC_LSB_SENS;
			mpu_moving.Accel_Y_RAW = ya_raw;
			mpu_moving.Ay = (float)ya_raw/ACC_LSB_SENS;
			mpu_moving.Accel_Z_RAW = za_raw;
			mpu_moving.Az = (float)za_raw/ACC_LSB_SENS;
			
			uint8_t xhigh;
			uint8_t xlow;
			uint8_t yhigh;
			uint8_t ylow;
			uint8_t zhigh;
			uint8_t zlow;
			
			xhigh = I2C2_DMA_buffer[8];
			xlow = I2C2_DMA_buffer[9];
			float x_raw = (int16_t)(xhigh << 8 | xlow);
			float gyro_x = x_raw/GYRO_LSB_SENS;
			yhigh = I2C2_DMA_buffer[10];
			ylow = I2C2_DMA_buffer[11];
			float y_raw = (int16_t)(yhigh << 8 | ylow);
			float gyro_y = y_raw/GYRO_LSB_SENS;
			zhigh = I2C2_DMA_buffer[12];
			zlow = I2C2_DMA_buffer[13];
			float z_raw = (int16_t)(zhigh << 8 | zlow);
			float gyro_z = z_raw/GYRO_LSB_SENS;
			
			mpu_moving.Gyro_X_RAW = x_raw;
			mpu_moving.Gx = gyro_x;
			mpu_moving.Gyro_Y_RAW = y_raw;
			mpu_moving.Gy = gyro_y;
			mpu_moving.Gyro_Z_RAW = z_raw;
			mpu_moving.Gz = gyro_z;
			
			KFilter_2(&mpu_moving);
			
			
			Process_mpu_moving = 0;
			mpu_moving_kfilter_complete = 1;
		}
		
		//GPIOC->ODR ^= GPIO_ODR_6;
		//HMC5883_ReadRawData(&mag_moving);
		//KFilter_2(&mpu_moving);
		//KFilter_2(&mpu_stationary);
		//HAL_Delay(100);

		
		
		/*//PWM TESTING CODE
		GPIOC->ODR &= ~(GPIO_ODR_7 | GPIO_ODR_6 | GPIO_ODR_9);
		if(provide_channel(1) > 1500){GPIOC->ODR |= GPIO_ODR_6;}
		if(provide_channel(2) > 1500){GPIOC->ODR |= GPIO_ODR_7;}
		if(provide_channel(3) > 1500){GPIOC->ODR |= GPIO_ODR_9;}
		*/
		
		//MOTOR TESTING CODE
		
		//DCSetOutput(DCtracker, 1);
		//BLDC_Output(BLDCtracker, 1);
		//BLDC_Output(BLDCtracker, 2);
		
		//DCtracker += DC_Direction;
		//BLDCtracker += BLDC_Direction;
		//if((DCtracker > 999) || (DCtracker < -999)) {DC_Direction -= 2 * DC_Direction;}
		//if(BLDCtracker > 350) BLDC_Direction = -5;
		//if(BLDCtracker < 10)  BLDC_Direction = 5;
		//HAL_Delay(2);
		
		
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
  htim1.Init.Prescaler = 15;
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
  htim6.Init.Prescaler = 159;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
<<<<<<< Updated upstream
	hdma_i2c2_rx.Instance = DMA1_Channel5;
    hdma_i2c2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c2_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c2_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_i2c2_rx) != HAL_OK) {
        Error_Handler();
    }

   __HAL_LINKDMA(&hi2c2, hdmarx, hdma_i2c2_rx);
=======

  /* DMA interrupt init */
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

>>>>>>> Stashed changes
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
  HAL_GPIO_WritePin(GPIOB, EN_BLDC2_Pin|EN_BLDC1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Indicator_Pin */
  GPIO_InitStruct.Pin = LED_Indicator_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Indicator_GPIO_Port, &GPIO_InitStruct);

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

/*
	*This functions handles the data retrieved by DMA processes on I2C2 line 
	*burst reads from slave devices store data directly in provided memory address,
	*this function manages this data based on a state machine, and initiates additional data
	*transfers if appropriate
*/
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	//List of states within I2C2_DMA_state:
	//States indiciate the most recent I2C data transfer that has been initiated
	//state 1: moving_MPU6050 Data Retrieval,
	//state 2: N/A
	//state 3: moving_QMC5883 Data Retrieval
	//state 4: stationary_MPU6050 Data Retrieval
	//state 5: N/A
	
	switch(I2C2_DMA_state)
	{
		case(1):{//moving_MPU6050 gyro data just retrieved
			//New data has been put into I2C2_DMA_buffer, 
			//tell main loop to process this new data using flag below
			Process_mpu_moving = 1;
			
			return;
			}
		case(2):{
			//not used
		
		}
		case(3):{//moving_QMC5883 Data just retrieved
			//not used
			
		}
		case(4):{//stationary_MPU6050 gyro data just retrieved
			//not used yet
			
		}
		case(5):{//stationary_MPU6050 Accel Data just retrieved
			//not used yet
			
		}
		default: return;
	
	}
	return;
}

/* @brief This function handles USART3 and USART4 global interrupts.
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
				if (strcmp(cmdBuffer, "rdpitch") == 0)
				{

					USART_Transmit_String("Angle pitch = ");
					USART_Transmit_Float(mpu_moving.KalmanAnglePitch, 2);
					USART_Transmit_Newline();
				}
				else if(strcmp(cmdBuffer, "rdroll") == 0){

					USART_Transmit_String("Angle roll = ");
					USART_Transmit_Float(mpu_moving.KalmanAngleRoll, 2);				
					USART_Transmit_Newline();
				}
				else if(strcmp(cmdBuffer, "rdyaw") == 0){

					USART_Transmit_String("Angle roll = ");
					USART_Transmit_Float(mpu_moving.KalmanAngleYaw, 2);
					USART_Transmit_Newline();
				}
				else if(strncmp(cmdBuffer, "wrpitch", 7) == 0){

					char* extractedString = &cmdBuffer[8];
					value = strtod(extractedString, &endPtr);
					if(endPtr != extractedString){
						USART_Transmit_String("wrpitch: Set pitch to ");
						USART_Transmit_Float(value, 2);
						set_desiredPitch(value); 
						USART_Transmit_Newline();						
					}else{
					USART_Transmit_String("ERROR: Parsing failed!\n");						
					}
				}
				else if(strncmp(cmdBuffer, "wrroll", 6) == 0){

					char* extractedString = &cmdBuffer[7];
					value = strtod(extractedString, &endPtr);
					if(endPtr != extractedString){
						USART_Transmit_String("wrroll: Set roll to ");
						USART_Transmit_Float(value, 2);
						set_desiredRoll(value);
						USART_Transmit_Newline();						
					}else{
						USART_Transmit_String("ERROR: Parsing failed!\n");						
					}
				}
				else if(strncmp(cmdBuffer, "wryaw", 5) == 0){

					char* extractedString = &cmdBuffer[6];
					value = strtod(extractedString, &endPtr);
					if(endPtr != extractedString){
						USART_Transmit_String("wryaw: Set yaw to ");
						USART_Transmit_Float(value, 2);
						set_desiredYaw(value);
						USART_Transmit_Newline();						
					}else{
						USART_Transmit_String("ERROR: Parsing failed!\n");						
					}
				} else if((strcmp(cmdBuffer, "PWM") == 0) | (strcmp(cmdBuffer, "pwm") == 0)){
					
						USART_Transmit_String("Input set to PWM");
						USART_Transmit_Newline();
						enablePWMIN();
						disableADCIN();
					
				} else if((strcmp(cmdBuffer, "ADC") == 0) | (strcmp(cmdBuffer, "adc") == 0)){
					
						USART_Transmit_String("Input set to ADC");
						USART_Transmit_Newline();
					
						enableADCIN();
						disablePWMIN();
					
			}else if((strcmp(cmdBuffer, "PWM/ADC off") == 0) | (strcmp(cmdBuffer, "pwm/adc off") == 0)){
				
						USART_Transmit_String("PWM and ADC off!");
						USART_Transmit_Newline();
						disablePWMIN();
						disableADCIN();
				
			} else
				{
					// Command not recognized
					USART_Transmit_String("ERROR: Command not recognized\n");
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
	double Angle2 = Angle1 + 120;
	double Angle3 = Angle1 + 240;
	
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

	Angle1 = Angle1 * 500;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
	Angle1 = Angle1 + 500;
	Angle2 = Angle2 * 500;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
	Angle2 = Angle2 + 500;
	Angle3 = Angle3 * 500;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
	Angle3 = Angle3 + 500;
	Angle1 = Angle1 * 0.7;
	Angle2 = Angle2 * 0.7;
	Angle3 = Angle3 * 0.7;

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
	double Angle2 = Angle1 + 120;
	double Angle3 = Angle1 + 240;
	
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

		Angle1 = Angle1 * 1000;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle1 = Angle1 + 1000;
		Angle2 = Angle2 * 1000;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle2 = Angle2 + 1000;
		Angle3 = Angle3 * 1000;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle3 = Angle3 + 1000;
		Angle1 = Angle1 * 0.7;
		Angle2 = Angle2 * 0.7;
		Angle3 = Angle3 * 0.7;

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
	/*
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
	*/
	if(useADC == 1){ // init made ADC 12 bit so it goes up to 4096
		int maxADCValue = (1 << 12) - 1; // For a 12-bit ADC
		
		// PITCH - PA4
		ADC1->CHSELR = ADC_CHSELR_CHSEL4;
		ADC1->CR |= ADC_CR_ADSTART;
		while (ADC1->CR & ADC_CR_ADSTART); // Wait for conversion to complete
		int pitchADC = ADC1->DR;
		double pitchBuffer = map(pitchADC, 0, maxADCValue, 1000, 2000);
		pitchBuffer = constrain(pitchBuffer, 1000, 2000);
		float pitchAngle = map(pitchBuffer, 1000, 2000, -80, 80);
		
		//ROLL - PA5
		ADC1->CHSELR = ADC_CHSELR_CHSEL5;
		ADC1->CR |= ADC_CR_ADSTART;
		while (ADC1->CR & ADC_CR_ADSTART); // Wait for conversion to complete
		int rollADC = ADC1->DR;
		double rollBuffer = map(rollADC, 0, maxADCValue, 1000, 2000);
		rollBuffer = constrain(rollBuffer, 1000, 2000);
		float rollAngle = map(rollBuffer, 1000, 2000, -80, 80);
		
		//YAW - PA3
		ADC1->CHSELR = ADC_CHSELR_CHSEL3;
		ADC1->CR |= ADC_CR_ADSTART;
		while (ADC1->CR & ADC_CR_ADSTART); // Wait for conversion to complete
		int yawADC = ADC1->DR;
		double yawBuffer = map(yawADC, 0, maxADCValue, 1000, 2000);
		yawBuffer = constrain(yawBuffer, 1000, 2000);
		float yawAngle = map(yawBuffer, 1000, 2000, -1000, 1000);
    

    // Set desired angles
    set_desiredPitch(pitchAngle);
    set_desiredRoll(rollAngle);
    set_desiredYaw(yawAngle);	
		//set_desiredPitch(60);
		//set_desiredRoll(60);
		DCSetOutput(yawAngle,1);
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
	doPID = 1;
	PID_execcount ++;
	//BLDC_PID(&mpu_moving, &mpu_stationary);
	
}

void MPU_Moving_DataSample(){
	//initiate data retrieval from MPU_moving using dma
	if(mpu_moving_kfilter_complete == 1){
		uint16_t numberofbytes = 14;
		I2C_SetRegAddress(mpu_moving.deviceAddr, 0x43);
		I2C2->CR2 = 0; // Clear register
		I2C2->CR2 |= (mpu_moving.deviceAddr << 1); // Set the slave address 
		I2C2->CR2 |= (numberofbytes << 16); // setup burst read of 14 bytes
		I2C2->CR2 |= (1 << 10); // Set the RD_WRN bit for read operation
		
		HAL_I2C_Master_Receive_DMA(&hi2c2, mpu_moving.deviceAddr, I2C2_DMA_buffer, numberofbytes);
		I2C2->CR2 |= I2C_CR2_START; // Send the start condition
	}
	return;
	//KFilter_2(&mpu_moving);
	//KFilter_2(&mpu_stationary);
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

<<<<<<< Updated upstream
void enableADCIN(){
	useADC = 1;
	//FILL IN HOW TO STARTUP AND ENABLE ADC
=======

//This function call initializes the usage all peripherals,
void Custom_StartupRoutine() {
	//External Data Init-----------------------------------------------

	HAL_I2C_Init(&hi2c2);

	//HAL_UART_Receive_IT(&huart3, &rx_data[rx_index], 1);

	//while (uselesscounter < 10000000){uselesscounter++;}
	//HMC5883_Init(&mag_moving);
	//MPU_Init(&mpu_moving, 0x68);

	//AS5600_Init(&yaw_sense, 0x36);

	//---------------------DEFAULT Disable PWM control----------------------//
	disablePWMIN();
	//enablePWMIN();

	//----------//-Deliver Power to Motors, in some default state------------//
	init_PitchMotor();
	init_RollMotor();
	init_YawMotor();

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
	//HAL_TIM_Base_Start_IT(&htim1);//enable timer 1 interrupt (1  khz frequency)
	//----------------------ENABLE MPU_MOVING SAMPLE--------------//
	//HAL_TIM_Base_Start_IT(&htim6);//enable timer 6 interrupt (200 hz frequency)
}

void Sample_MpuMoving() {
	//KFilter_2(&mpu_moving);
	
	if((BurstReadState == 0) && (mpu_moving_newdata == 0)){
  	I2C_BurstRead_Cheap(mpu_moving.deviceAddr, ACC_XOUT_HIGH, 14);
		BurstReadState = 1;//reading from mpu_moving
	}
	else{i2c_queue |= 0b00000010;}//if second bit is set, do a sample mpu_moving asap
>>>>>>> Stashed changes
	
}

void disableADCIN(){
	useADC = 0;
	//FILL IN HOW TO STOP AND DISABLE ADC
	
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
