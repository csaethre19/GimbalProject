#include <stdlib.h>

#include "DCMotor.h"

#define Yaw1_TimARR 1000;
#define Yaw 1;
#define Yaw1_Ch1 TIM1->CCR1;
#define Yaw1_Ch2 TIM1->CCR2;

/*
	DC motor driver,
	Functions that drive the implementation of a single DC motor
	Two PWM outputs
*/

/*
	DCSetOutput takes in a value from -1000 to 1000
	Based on this, the DC motor is set to rotate CW or CCW from 0% to 100% power
*/
void DCSetOutput(int Output, int MotorNum)
{
	if((Output < -1000) || (Output > 1000)) return;
	
	if(MotorNum == Yaw){
		if(Output > 0){//CW? Direction instructed, Ch2 not 100% duty cycle
			Yaw1_Ch1 = Yaw1_TimARR;
			//Max Spin Rate = Yaw1_TimARR * 0      ie. Output = 1000 -> (1 - (Output / 1000)) = 0
			//Min Spin Rate = Yaw1_TimARR * 0.999  ie. Output = 0001 -> (1 - (Output / 1000)) = 0.999
			Yaw1_Ch2 = Yaw1_TimARR * (1 - (Output / 1000)); 
		}
		else if(Output < 0){//CCW? Direction instructed, Ch1 not 100% duty cycle
			Yaw1_Ch2 = Yaw1_TimARR;
			//Max Spin Rate = Yaw1_TimARR * 0      ie. Output = 1000 -> (1 - (Output / 1000)) = 0
			//Min Spin Rate = Yaw1_TimARR * 0.999  ie. Output = 0001 -> (1 - (Output / 1000)) = 0.999
			Yaw1_Ch1 = Yaw1_TimARR * (1 - (Output / 1000)); 
		}
		else{//Output = 0
			Yaw1_Ch1 = Yaw1_TimARR;
			Yaw1_Ch2 = Yaw1_TimARR;
		}
	}
}

void initDCOutput(int MotorNum)
{
	if(MotorNum == Yaw){//put the provided DC motor into its brake state;
		Yaw1_Ch1 = Yaw1_TimARR;//Duty Cycle = 100%
		Yaw1_Ch2 = Yaw1_TimARR;//Duty Cycle = 100%
	}
}