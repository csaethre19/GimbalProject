#include <stdlib.h>

#include "DCMotor.h"
#include "MPU6050.h"

#define Yaw1_TimARR 1000;
#define Yaw 1;
#define Yaw1_Ch1 TIM1->CCR1;
#define Yaw1_Ch2 TIM1->CCR2;

//Yaw PID variables
volatile int16_t current_yaw_instruction = 0;
volatile int16_t yaw_error_integral = 0;    // Integrated yaw error signal
volatile int16_t yaw_error_derivative = 0;    // Derivated yaw error signal
volatile int16_t target_yaw = 0;        // Desired yaw angle target
volatile int16_t measured_yaw = 0;       // Measured yaw angle
volatile int16_t yaw_error = 0;             // Yaw error signal
volatile uint8_t Kp_Yaw = 20;                // Proportional gain
volatile uint8_t Ki_Yaw = 10;                // Integral gain
volatile uint8_t Kd_Yaw = 10;                // Derivative gain
volatile int16_t relative0_absolute1_DC = 1;

//	DC motor driver,
//	Functions that drive the implementation of a single DC motor
//	Two PWM outputs

void set_desiredYaw(float desiredYaw){target_yaw = desiredYaw;}
void set_operationModeDC(int16_t setMode){relative0_absolute1_DC = setMode;}
void DC_PID(volatile MPU6050_t *targetOrientation, volatile MPU6050_t *stationaryOrientation){
	//CURRENT IMPLEMENTATION IS NON IDEAL : Known issues are as follows:
	//Yaw is exclusively absolute, no consideration of front of aircraft
	//clamping is done with untested conservative value
	//
	GPIOC->ODR ^= GPIO_ODR_9;
	float relativeYaw = targetOrientation->AngleYaw - stationaryOrientation->KalmanAngleYaw;
	
	
	if(relative0_absolute1_DC){//Absolute Position Mode Execute

		yaw_error = target_yaw - targetOrientation->KalmanAngleYaw;
		double pitch_motor_Offset = (double)(yaw_error * Kp_Yaw);
		
		if(pitch_motor_Offset > 100) pitch_motor_Offset = 100;
		if(pitch_motor_Offset < -100) pitch_motor_Offset = -100;

		int calculated_output = pitch_motor_Offset * (999 / 100);
		current_yaw_instruction = calculated_output;
		
		DCSetOutput(current_yaw_instruction, 1);//write new instructed angle to pitch BLDC motor;
	}
	else{//Relative Position Mode Execute
			//Not possible without relative yaw determination
		yaw_error = target_yaw - relativeYaw;
		double pitch_motor_Offset = (double)(yaw_error * Kp_Yaw);
		if(pitch_motor_Offset > 100) pitch_motor_Offset = 100;
		if(pitch_motor_Offset < -100) pitch_motor_Offset = -100;

		int calculated_output = pitch_motor_Offset * (999 / 100);
		current_yaw_instruction = calculated_output;
	
		DCSetOutput(current_yaw_instruction, 1);//write new instructed angle to pitch BLDC motor;
	}
	
	
}


//	DCSetOutput takes in a value from -1000 to 1000
//	Based on this, the DC motor is set to rotate CW or CCW from 0% to 100% power

void DCSetOutput(int Output, int MotorNum)
{
	if((Output < -1000) || (Output > 1000)) return;
	current_yaw_instruction = Output;
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