#include <stdlib.h>

#include "DCMotor.h"
#include "MPU6050.h"

#define Yaw1_TimARR 1000;
#define Yaw 1;
#define Yaw1_Ch1 TIM1->CCR1;
#define Yaw1_Ch2 TIM1->CCR2;

//Yaw PID variables
volatile int16_t yaw_error_integral = 0;    // Integrated yaw error signal
volatile int16_t yaw_error_derivative = 0;    // Derivated yaw error signal
volatile int16_t target_yaw = 0;        // Desired yaw angle target
volatile int16_t measured_yaw = 0;       // Measured yaw angle
volatile int16_t yaw_error = 0;             // Yaw error signal

//	DC motor driver,
//	Functions that drive the implementation of a single DC motor
//	Two PWM outputs



void DC_PID(MPU6050_t *targetOrientation, MPU6050_t *stationaryOrientation)

//	DCSetOutput takes in a value from -1000 to 1000
//	Based on this, the DC motor is set to rotate CW or CCW from 0% to 100% power

void DCSetOutput(int Output, int MotorNum)
{
	if((Output < -1000) || (Output > 1000)) return;
	
	if(MotorNum == 1){
		if(Output > 0){//CW? Direction instructed, Ch2 not 100% duty cycle
			TIM1->CCR1 = Yaw1_TimARR;
			//Max Spin Rate = Yaw1_TimARR * 0      ie. Output = 1000 -> (1 - (Output / 1000)) = 0
			//Min Spin Rate = Yaw1_TimARR * 0.999  ie. Output = 0001 -> (1 - (Output / 1000)) = 0.999
			TIM1->CCR2 = 1000 * (double)(1 - ((double)Output / 1000)); 
		}
		else if(Output < 0){//CCW? Direction instructed, Ch1 not 100% duty cycle
			TIM1->CCR2 = Yaw1_TimARR;
			//Max Spin Rate = Yaw1_TimARR * 0      ie. Output = 1000 -> (1 - (Output / 1000)) = 0
			//Min Spin Rate = Yaw1_TimARR * 0.999  ie. Output = 0001 -> (1 - (Output / 1000)) = 0.999
			TIM1->CCR1 = 1000 * (double)(1 + ((double)Output / 1000)); 
		}
		else{//Output = 0
			TIM1->CCR1 = Yaw1_TimARR;
			TIM1->CCR2 = Yaw1_TimARR;
		}
	}
}

void initDCOutput(int MotorNum)
{
	if(MotorNum == 1){//put the provided DC motor into its brake state;
		TIM1->CCR1 = Yaw1_TimARR;//Duty Cycle = 100%
		TIM1->CCR2 = Yaw1_TimARR;//Duty Cycle = 100%
	}
}