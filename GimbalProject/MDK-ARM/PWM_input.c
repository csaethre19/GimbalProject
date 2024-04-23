#include <stdlib.h>
#include <math.h>
#include "main.h"
#include "BLDCMotor.h"
#include "DCMotor.h"

//CHANNEL NUMBERS:
//YAW   = 1
//PITCH = 2
//ROLL  = 3

volatile int Yaw_RiseTime;
volatile int Yaw_FallTime;
volatile int Yaw_TimeHigh;

volatile int Pitch_RiseTime;
volatile int Pitch_FallTime;
volatile int Pitch_TimeHigh;

volatile int Roll_RiseTime;
volatile int Roll_FallTime;
volatile int Roll_TimeHigh;

static int map(int value, int fromLow, int fromHigh, int toLow, int toHigh);

//When an timer interrupts that detect and retrieve the rise/fall time of different PWM channels will forward
//that data to this function, which will handle all required logic
void process_eventTime(int eventTime, int rise0_or_faLL1, int PWM_channel)
{
	
	switch(PWM_channel)
	{
		case 1: {
			//GPIOC->ODR ^= GPIO_ODR_6;
			if(rise0_or_faLL1 == 0){//RISING EDGE
				Yaw_RiseTime = eventTime;
			}
			else{//FALLING EDGE, calculate high time now
				Yaw_FallTime = eventTime;
				int oldTimeHigh = Yaw_TimeHigh;
				if(Yaw_FallTime > Yaw_RiseTime){Yaw_TimeHigh = Yaw_FallTime - Yaw_RiseTime;}
				else{Yaw_TimeHigh = (Yaw_FallTime + 65535) - Yaw_RiseTime;}
				if(( !(Yaw_TimeHigh > 2000)) && ( !(Yaw_TimeHigh < 1000))){
					//int timehighchange = oldTimeHigh - Yaw_TimeHigh;
					//if((timehighchange < 5) || (timehighchange > -5)){
						int yawbuffer = Yaw_TimeHigh;
						yawbuffer = map(yawbuffer, 1000,2000,-70,70);
						set_desiredYaw(yawbuffer);
					//}
					//else {Roll_TimeHigh = oldTimeHigh;}//reject rapid changes in Time High values
				}
			}
			return;
		}
		case 2: {
			//GPIOC->ODR ^= GPIO_ODR_7;
			if(rise0_or_faLL1 == 0){//RISING EDGE
				Pitch_RiseTime = eventTime;
			}
			else{//FALLING EDGE, calculate high time now
				Pitch_FallTime = eventTime;
				int oldTimeHigh = Pitch_TimeHigh;
				if(Pitch_FallTime > Pitch_RiseTime){Pitch_TimeHigh = Pitch_FallTime - Pitch_RiseTime;}
				else{ Pitch_TimeHigh = (Pitch_FallTime + 65535) - Pitch_RiseTime;}
				if(( !(Pitch_TimeHigh > 2000)) && ( !(Pitch_TimeHigh < 1000))){
					//int timehighchange = oldTimeHigh - Pitch_TimeHigh;
					//if((timehighchange < 5) || (timehighchange > -5)){
						int pitchbuffer = Pitch_TimeHigh;
						pitchbuffer = map(pitchbuffer, 1000,2000,-70,70);
						set_desiredPitch(pitchbuffer);
					//}
					//else {Roll_TimeHigh = oldTimeHigh;}//reject rapid changes in Time High values
				}
			}
			return;
		}
		case 3: {
			//GPIOC->ODR ^= GPIO_ODR_9;
			if(rise0_or_faLL1 == 0){//RISING EDGE
				Roll_RiseTime = eventTime;
			}
			else{//FALLING EDGE, calculate high time now
				Roll_FallTime = eventTime;
				int oldTimeHigh = Roll_TimeHigh;
				if(Roll_FallTime > Roll_RiseTime){Roll_TimeHigh = Roll_FallTime - Roll_RiseTime;}
				else{Roll_TimeHigh = (Roll_FallTime + 65535) - Roll_RiseTime;}
				if(( !(Roll_TimeHigh > 2000)) && ( !(Roll_TimeHigh < 1000))){
					//int timehighchange = oldTimeHigh - Roll_TimeHigh;
					//if((timehighchange < 5) || (timehighchange > -5)){
						int rollbuffer = Roll_TimeHigh;
						rollbuffer = map(rollbuffer, 1000,2000,-70,70);
						set_desiredRoll(rollbuffer);
					//}
					//else {Roll_TimeHigh = oldTimeHigh;}//reject rapid changes in Time High values
				}
			}
			return;
		}
		default: return;
	}
	
	
}

//Caller receives the PWM duration of a requested PWM channel
//PWM channel numbers identified at top of this file
//Also its really obvious so just read through the code
int provide_channel(int PWM_channel){
	switch(PWM_channel)
	{
		case 1: return Yaw_TimeHigh;
		case 2: return Pitch_TimeHigh;
		case 3: return Roll_TimeHigh;
		default: return -1;
	}
}

static int map(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}


