#include <stdlib.h>
#include <math.h>

#include "BLDCMotor.h"


#define PI 3.1415926535897932;

#define Pitch_TimARR 1000;
#define half_Pitch_TimARR 500;
#define Roll_TimARR 1000;
#define half_Roll_TimARR 500;
#define Pitch 1;
#define Roll 2;
#define Pitch_1A TIM2->CCR1;
#define Pitch_1B TIM2->CCR2;
#define Pitch_1C TIM2->CCR3;
#define Roll_2A TIM3->CCR1;
#define Roll_2B TIM3->CCR2;
#define Roll_2C TIM3->CCR3;

//PID TRACKER PITCH
volatile int16_t pitch_error_integral = 0;    // Integrated pitch error signal
volatile int16_t roll_error_integral = 0;    // Integrated roll error signal

volatile int16_t pitch_error_derivative = 0;    // Derivated pitch error signal
volatile int16_t roll_error_derivative = 0;    // Derivated roll error signal

volatile int16_t target_pitch = 0;        // Desired pitch angle target
volatile int16_t target_roll = 0;        // Desired roll angle target

volatile int16_t measured_roll = 0;       // Measured roll angle
volatile int16_t measured_pitch = 0;       // Measured pitch angle

volatile int16_t current_pitch_instruction = 0; //currently instructed motor angle for pitch axis
volatile int16_t current_roll_instruction = 0;

volatile int16_t pitch_error = 0;             // Pitch error signal
volatile int16_t roll_error = 0;             // Roll error signal
volatile uint8_t Kp_Roll = 1;                // Proportional gain
volatile uint8_t Ki_Roll = 1;                // Integral gain
volatile uint8_t Kd_Roll = 1;                // Derivative gain
volatile uint8_t Kp_Pitch = 10;                // Proportional gain
volatile uint8_t Ki_Pitch = 10;                // Integral gain
volatile uint8_t Kd_Pitch = 10;                // Derivative gain
//PID TRACKER ROLL


// This implementation of BLDCMotor control takes in a value from 0-359 as the "desired angle"
//	The determination of what this "desired angle" is currently intended to be performed elsewhere
//	This code simply drives the signals delivered to the motors,
//	as well as defaults to some hopefully harmless power up values
	
//	PLANNED ADDITIONS:
//	1. Handle the PID loop through simple function calls within this method
//	2. Deal with the current measurement appropriately (Pitch/Roll Current ADC input)
//	3.?



	//Takes in a double Angle1 and a motor number and modifies the PWM outputs to the respective BLDC motor

void set_desiredRoll(float desiredRoll){target_roll = desiredRoll;}
void set_desiredPitch(float desiredPitch){target_pitch = desiredPitch;}

void BLDC_PID(volatile MPU6050_t *targetOrientation, volatile MPU6050_t *stationaryOrientation){
	//CURRENT IMPLEMENTATION IS NON IDEAL : Known issues are as follows:
	//Proportional controller only
	//clamping is done with untested conservative value
	//No consideration of mechanical limits of system
	//ONLY ONE MPU6050 INPUT IS CONSIDERED ie. the motor will rip the gimbal to shreds if instructed
	//
	
	pitch_error = target_pitch - targetOrientation->KalmanAnglePitch;
	
	int pitch_motor_Offset = (int)(pitch_error * Kp_Pitch);
	
	if(pitch_motor_Offset > 50) pitch_motor_Offset = 50;
	if(pitch_motor_Offset < -50) pitch_motor_Offset = -50;
	
	current_pitch_instruction = current_pitch_instruction + pitch_motor_Offset;
	
	if(current_pitch_instruction > 360) current_pitch_instruction -= 360;
	if(current_pitch_instruction < 0) current_pitch_instruction += 360;
	BLDC_Output(current_pitch_instruction, 1);//write new instructed angle to pitch BLDC motor;
}

void BLDC_Output(double Angle1, int MotorNum)
{
	if((Angle1 < 0) || (Angle1 > 360)) return;
	
	double Angle2 = Angle1 + 120;
	double Angle3 = Angle1 + 240;
	if(Angle2 > 360) Angle2 -= 360;
	if(Angle3 > 360) Angle3 -= 360;
	
	
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
		Angle1 = Angle1 * half_Pitch_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle1 = Angle1 + half_Pitch_TimARR;
		Angle2 = Angle2 * half_Pitch_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle2 = Angle2 + half_Pitch_TimARR;
		Angle3 = Angle3 * half_Pitch_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle3 = Angle3 + half_Pitch_TimARR;
		//0.7 chosen to decrease amount of power delivered to motor, adjust after testing
		TIM2->CCR1 = Angle1 * 0.7;
		TIM2->CCR2 = Angle2 * 0.7;
		TIM2->CCR3 = Angle3 * 0.7;
		return;
	}
	else if(MotorNum == 2){
		Angle1 = Angle1 * half_Roll_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle1 = Angle1 + half_Roll_TimARR;
		Angle2 = Angle2 * half_Roll_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle2 = Angle2 + half_Roll_TimARR;
		Angle3 = Angle3 * half_Roll_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle3 = Angle3 + half_Roll_TimARR;
		//0.7 chosen to decrease amount of power delivered to motor, adjust after testing
		TIM3->CCR1 = Angle1 * 0.7;
		TIM3->CCR2 = Angle2 * 0.7;
		TIM3->CCR3 = Angle3 * 0.7;
		return;
	}
	
	return;
}


//Give some random initial value to the Duty cycle of the BLDC motors
//init_Pitch & init_Roll replace this function
void initBLDCOutput(int MotorNum)
{
	
}

void BLDCEnable(int MotorNum){
	if(MotorNum == 1)GPIOA->ODR |= GPIO_ODR_15;//enable pitch motor driver
	if(MotorNum == 2)GPIOA->ODR |= GPIO_ODR_14;//enable roll motor driver
}

void BLDCDisable(int MotorNum){
	if(MotorNum == 1)GPIOA->ODR &= ~GPIO_ODR_15;//disable pitch motor driver
	if(MotorNum == 2)GPIOA->ODR &= ~GPIO_ODR_14;//disable roll motor driver
}
