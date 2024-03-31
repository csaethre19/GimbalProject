#include <stdlib.h>

#include "BLDCMotor.h"

#define Pitch_TimARR 1000;
#define Roll_TimARR 1000;
#define Pitch 1;
#define Roll 2;
#define Pitch1_A TIM2->CCR1;
#define Pitch1_B TIM2->CCR2;
#define Pitch1_C TIM2->CCR3;
#define Roll2_A TIM3->CCR1;
#define Roll2_B TIM3->CCR2;
#define Roll2_C TIM3->CCR3;

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

void BLDC_Output(int Angle, int MotorNum)
{
	if((Angle < 0) || (Angle > 360)) return;
	
	if(MotorNum == Pitch){

	}
	return;
}

void initBLDCOutput(int Output, int MotorNum)
{
	if(MotorNum == Pitch){
		
	}
}