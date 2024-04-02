#include <stdlib.h>
#include <math.h>

#include "BLDCMotor.h"

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


// This implementation of BLDCMotor control takes in a value from 0-359 as the "desired angle"
//	The determination of what this "desired angle" is currently intended to be performed elsewhere
//	This code simply drives the signals delivered to the motors,
//	as well as defaults to some hopefully harmless power up values
	
//	PLANNED ADDITIONS:
//	1. Handle the PID loop through simple function calls within this method
//	2. Deal with the current measurement appropriately (Pitch/Roll Current ADC input)
//	3.?



	//Takes in a double Angle1 and a motor number and modifies the PWM outputs to the respective BLDC motor

void BLDC_Output(double Angle1, int MotorNum)
{
	if((Angle1 < 0) || (Angle1 > 360)) return;
	
	double Angle2 = Angle1 + 120;
	double Angle3 = Angle1 + 240;
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
		TIM3->CCR2 = Angle2 * 0.7;
		TIM3->CCR3 = Angle3 * 0.7;
		return;
	}
	else if(MotorNum == 2){
		Angle1 = Angle1 * Roll_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle1 = Angle1 + Roll_TimARR;
		Angle2 = Angle2 * Roll_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle2 = Angle2 + Roll_TimARR;
		Angle3 = Angle3 * Roll_TimARR;//sin(angle1) produces -1 -> 1. We need positive range of values from 0 -> max pwm duty cycle value
		Angle3 = Angle3 + Roll_TimARR;
		TIM2->CCR1 = Angle1 * 0.7;
		TIM3->CCR2 = Angle2 * 0.7;
		TIM3->CCR3 = Angle3 * 0.7;
		return;
	}
	
	return;
}


//Give some random initial value to the Duty cycle of the BLDC motors

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

