#include <stdlib.h>
#include <math.h>
#include "arm_math.h"

#include "BLDCMotor.h"

#define PI_180 0.01745329f;
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

volatile double target_pitch = 0;        // Desired pitch angle target
volatile double target_roll = 0;        // Desired roll angle target

volatile int16_t measured_roll = 0;       // Measured roll angle
volatile int16_t measured_pitch = 0;       // Measured pitch angle

volatile double current_pitch_instruction = 0; //currently instructed motor angle for pitch axis
volatile double current_roll_instruction = 0;

volatile double pitch_error = 0;             // Pitch error signal
volatile double roll_error = 0;             // Roll error signal
double Kp_Roll = 0.2;                // Proportional gain
volatile uint8_t Ki_Roll = 1;                // Integral gain
volatile uint8_t Kd_Roll = 1;                // Derivative gain
double Kp_Pitch = 0.2;                // Proportional gain
volatile uint8_t Ki_Pitch = 10;                // Integral gain
volatile uint8_t Kd_Pitch = 10;                // Derivative gain

volatile int16_t relative0_absolute1 = 1;

// This implementation of BLDCMotor control takes in a value from 0-359 as the "desired angle"
//	The determination of what this "desired angle" is currently intended to be performed elsewhere
//	This code simply drives the signals delivered to the motors,
//	as well as defaults to some hopefully harmless power up values
	
//	PLANNED ADDITIONS:
//	1.
//	2. Deal with the current measurement appropriately (Pitch/Roll Current ADC input)
//	3.?



	//Takes in a double Angle and a motor number and modifies the PWM outputs to the respective BLDC motor

void set_desiredRoll(float desiredRoll){target_roll = desiredRoll;}
void set_desiredPitch(float desiredPitch){target_pitch = desiredPitch;}
void set_operationMode(int16_t setMode){relative0_absolute1 = setMode;}
void BLDC_PID(volatile MPU6050_t *targetOrientation, volatile MPU6050_t *stationaryOrientation){
	//CURRENT IMPLEMENTATION IS NON IDEAL : Known issues are as follows:
	//Proportional controller only
	//ADD CONSIDERATION OF MECHANICAL LIMITS TO BOTH OPERATION MODES
	
	float relativePitch = targetOrientation->AnglePitch - stationaryOrientation->AnglePitch;
	float relativeRoll = targetOrientation->AngleRoll - stationaryOrientation->AngleRoll;
	if(relative0_absolute1){//Absolute Position Mode Execute
		// Need to add collision checking
		//PITCH MOTOR
		pitch_error = target_pitch - targetOrientation->KalmanAnglePitch;
		double pitch_motor_Offset = (double)pitch_error * Kp_Pitch;
		if(pitch_motor_Offset > 1) pitch_motor_Offset = 1;
		if(pitch_motor_Offset < -1) pitch_motor_Offset = -1;
		current_pitch_instruction = current_pitch_instruction + pitch_motor_Offset;
		if(current_pitch_instruction > 360) current_pitch_instruction -= 360;
		if(current_pitch_instruction < 0) current_pitch_instruction += 360;
		BLDC_Output(current_pitch_instruction, 1);//write new instructed angle to pitch BLDC motor;
		//ROLL MOTOR
		roll_error = target_roll - targetOrientation->KalmanAngleRoll;
		int roll_motor_Offset = (int)roll_error * Kp_Roll;
		if(roll_motor_Offset > 1) roll_motor_Offset = 1;
		if(roll_motor_Offset < -1) roll_motor_Offset = -1;
		current_roll_instruction = current_roll_instruction + roll_motor_Offset;
		if(current_roll_instruction > 360) current_roll_instruction -= 360;
		if(current_roll_instruction < 0) current_roll_instruction += 360;
		BLDC_Output(current_roll_instruction, 2);//write new instructed angle to roll BLDC motor;
	}
	else{//Relative Position Mode Execute
		//PITCH MOTOR
		pitch_error = target_pitch - relativePitch;
		double pitch_motor_Offset = (double)pitch_error * Kp_Pitch;
		if(pitch_motor_Offset > 1) pitch_motor_Offset = 1;
		if(pitch_motor_Offset < -1) pitch_motor_Offset = -1;
		current_pitch_instruction = current_pitch_instruction + pitch_motor_Offset;
		if(current_pitch_instruction > 360) current_pitch_instruction -= 360;
		if(current_pitch_instruction < 0) current_pitch_instruction += 360;
		BLDC_Output(current_pitch_instruction, 1);//write new instructed angle to pitch BLDC motor;
		//ROLL MOTOR
		roll_error = target_roll - relativeRoll;
		int roll_motor_Offset = (int)roll_error * Kp_Roll;
		if(roll_motor_Offset > 1) roll_motor_Offset = 1;
		if(roll_motor_Offset < -1) roll_motor_Offset = -1;
		current_roll_instruction = current_roll_instruction + roll_motor_Offset;
		if(current_roll_instruction > 360) current_roll_instruction -= 360;
		if(current_roll_instruction < 0) current_roll_instruction += 360;
		BLDC_Output(current_roll_instruction, 2);//write new instructed angle to roll BLDC motor;
	}
	
	
	
}

/*Send a signal from 0-360 to instruct an output angle of the BLDC motor
MotorNum -> 1 == PITCH(BLDC1),  2 == ROLL(BLDC2)
*/
void BLDC_Output(float Angle1, int MotorNum)
{
	if((Angle1 < 0) || (Angle1 > 360)) return;
	
	double Angle2 = Angle1 + 120;
	double Angle3 = Angle1 + 240;
	if(Angle2 > 360) Angle2 -= 360;
	if(Angle3 > 360) Angle3 -= 360;
	
	//Angle1 Conversion
	Angle1 = Angle1 * PI_180;
	Angle1 = arm_sin_f32(Angle1);
	//Angle2 Conversion
	Angle2 = Angle2 * PI_180;
	Angle2 = arm_sin_f32(Angle2);
	//Angle3 Conversion
	Angle3 = Angle3 * PI_180;
	Angle3 = arm_sin_f32(Angle3);
	
	if(MotorNum == 1){
		Angle1 = Angle1 * (double)half_Pitch_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle1 = Angle1 + (double)half_Pitch_TimARR;
		Angle2 = Angle2 * (double)half_Pitch_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle2 = Angle2 + (double)half_Pitch_TimARR;
		Angle3 = Angle3 * (double)half_Pitch_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle3 = Angle3 + (double)half_Pitch_TimARR;
		//0.7 chosen to decrease amount of power delivered to motor, adjust after testing
		
		TIM2->CCR1 = Angle1 * 0.7;
		TIM2->CCR2 = Angle2 * 0.7;
		TIM2->CCR3 = Angle3 * 0.7;
		return;
	}
	else if(MotorNum == 2){
		Angle1 = Angle1 * (double)half_Pitch_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle1 = Angle1 + (double)half_Pitch_TimARR;
		Angle2 = Angle2 * (double)half_Pitch_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle2 = Angle2 + (double)half_Pitch_TimARR;
		Angle3 = Angle3 * (double)half_Pitch_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle3 = Angle3 + (double)half_Pitch_TimARR;
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


void BLDCEnable(int MotorNum){
	if(MotorNum == 1)GPIOB->ODR |= GPIO_ODR_6;//enable pitch motor driver
	if(MotorNum == 2)GPIOB->ODR |= GPIO_ODR_5;//enable roll motor driver
}

void BLDCDisable(int MotorNum){
	if(MotorNum == 1)GPIOB->ODR &= ~GPIO_ODR_6;//disable pitch motor driver
	if(MotorNum == 2)GPIOB->ODR &= ~GPIO_ODR_5;//disable roll motor driver
}



