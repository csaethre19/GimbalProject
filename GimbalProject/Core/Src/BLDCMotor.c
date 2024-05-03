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


//PID TRACKING DATA
volatile double target_pitch = 0;        // Desired pitch angle target
volatile double target_roll = 0;        // Desired roll angle target

volatile int16_t measured_roll = 0;       // Measured roll angle
volatile int16_t measured_pitch = 0;       // Measured pitch angle

volatile double current_pitch_instruction = 0; //currently instructed motor angle for pitch axis
volatile double current_roll_instruction = 0;

PIDController rollPID;
PIDController pitchPID;
PIDController yawPID;

volatile int16_t rpMode = 1;
volatile int16_t yMode = 0;

int lastPIDtime = 0;

float rollPID_output = 0;
float pitchPID_output = 0;

// This implementation of BLDCMotor control takes in a value from 0-359 as the "desired angle"
//	The determination of what this "desired angle" is currently intended to be performed elsewhere
//	This code simply drives the signals delivered to the motors,
//	as well as defaults to some hopefully harmless power up values
	
//	PLANNED ADDITIONS:
//	1.



	//Takes in a double Angle and a motor number and modifies the PWM outputs to the respective BLDC motor

void set_desiredRoll(float desiredRoll){target_roll = desiredRoll;}
void set_desiredPitch(float desiredPitch){target_pitch = desiredPitch;}
void set_operationModeRollPitch(int16_t setMode){rpMode = setMode;}
void set_operationModeYaw(int16_t setMode){yMode = setMode;}

void BLDC_PID(volatile MPU6050_t *targetOrientation, volatile MPU6050_t *stationaryOrientation){
	//CURRENT IMPLEMENTATION:
	//PI controller attempts to prevent oscillations observed in non-perfect tuning of MPU6050 data.
	//,
	//double pitch_preverror = pitch_error;
	//double roll_preverror = roll_error;
	double Ti = (double) (HAL_GetTick() - lastPIDtime);//Time increment
	double lastPIDtime = HAL_GetTick();
	
	rollPID.T = (float)Ti/1000;
	pitchPID.T = (float)Ti/1000;
	rollPID_output = PIDController_Update(&rollPID, target_roll, targetOrientation->KalmanAngleRoll);
	pitchPID_output = PIDController_Update(&pitchPID, target_pitch, targetOrientation->KalmanAnglePitch);
	//float relativePitch = targetOrientation->AnglePitch - stationaryOrientation->AnglePitch;
	//float relativeRoll = targetOrientation->AngleRoll - stationaryOrientation->AngleRoll;
	
	if(rpMode == 1){//Absolute Position Mode Execute
		// Need to add collision checking
		//PITCH MOTOR
		pitchPID_output = PIDController_Update(&pitchPID, target_pitch, targetOrientation->KalmanAnglePitch);
		current_pitch_instruction = current_pitch_instruction + pitchPID_output;
		if(current_pitch_instruction > 360) current_pitch_instruction -= 360;
		if(current_pitch_instruction < 0) current_pitch_instruction += 360;
		BLDC_Output(current_pitch_instruction, 1);//write new instructed angle to pitch BLDC motor;
		//ROLL MOTOR
		rollPID_output = PIDController_Update(&rollPID, target_roll, targetOrientation->KalmanAngleRoll);
		current_roll_instruction = current_roll_instruction + rollPID_output;
		if(current_roll_instruction > 360) current_roll_instruction -= 360;
		if(current_roll_instruction < 0) current_roll_instruction += 360;
		BLDC_Output(current_roll_instruction, 2);//write new instructed angle to roll BLDC motor;
	}
	/*else{//Relative Position Mode Execute
		//PITCH MOTOR
		pitch_error = target_pitch - relativePitch;
		double pitch_motor_Offset = (double)pitch_error * Kp_Pitch;
		if(pitch_motor_Offset > 100) pitch_motor_Offset = 100;
		if(pitch_motor_Offset < -100) pitch_motor_Offset = -100;
		current_pitch_instruction = current_pitch_instruction + pitch_motor_Offset;
		if(current_pitch_instruction > 360) current_pitch_instruction -= 360;
		if(current_pitch_instruction < 0) current_pitch_instruction += 360;
		BLDC_Output(current_pitch_instruction, 1);//write new instructed angle to pitch BLDC motor;
		//ROLL MOTOR
		roll_error = target_roll - relativeRoll;
		double roll_motor_Offset = (double)roll_error * Kp_Roll;
		if(roll_motor_Offset > 100) roll_motor_Offset = 100;
		if(roll_motor_Offset < -100) roll_motor_Offset = -100;
		current_roll_instruction = current_roll_instruction + roll_motor_Offset;
		if(current_roll_instruction > 360) current_roll_instruction -= 360;
		if(current_roll_instruction < 0) current_roll_instruction += 360;
		BLDC_Output(current_roll_instruction, 2);//write new instructed angle to roll BLDC motor;
	}
	*/
	
	
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

void BLDC_PID_Init(){
	rollPID.Kp = 0.2;
	rollPID.Ki = 0.00000;
	rollPID.Kd = 1;
	rollPID.tau = 0.02f;
	rollPID.limMin = -25.0f;
	rollPID.limMax = 25.0f;
	rollPID.limMinInt = -5.0f;
	rollPID.limMaxInt = 5.0f;
	rollPID.integrator = 0.0f;
	rollPID.prevError = 0.0f;
	rollPID.differentiator = 0.0f;
	rollPID.prevMeasurement = 0.0f;
	rollPID.out = 0.0f;
	
	
	pitchPID.Kp = 0.2;
	pitchPID.Ki = 0.0000;
	pitchPID.Kd = 1;
	pitchPID.tau = 0.02f;
	pitchPID.limMin = -25.0f;
	pitchPID.limMax = 25.0f;
	pitchPID.limMinInt = -5.0f;
	pitchPID.limMaxInt = 5.0f;
	pitchPID.integrator = 0.0f;
	pitchPID.prevError = 0.0f;
	pitchPID.differentiator = 0.0f;
	pitchPID.prevMeasurement = 0.0f;
	pitchPID.out = 0.0f;
	
	yawPID.Kp = 0.5;
	yawPID.Ki = 10;
	yawPID.Kd = 10;
	yawPID.tau = 0.02f;
	yawPID.limMin = 100.0f;
	yawPID.limMax = 100.0f;
	yawPID.limMinInt = -10.0f;
	yawPID.limMaxInt = 10.0f;
	yawPID.integrator = 0.0f;
	yawPID.prevError = 0.0f;
	yawPID.differentiator = 0.0f;
	yawPID.prevMeasurement = 0.0f;
	yawPID.out = 0.0f;
}

float PIDController_Update(PIDController * pid, float target_angle, float measured_angle){
	
	float error = target_angle - measured_angle;
	
	float proportional = pid->Kp * error;
	
	pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);
	
		/* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {
        pid->integrator = pid->limMaxInt;
    } else if (pid->integrator < pid->limMinInt) {
        pid->integrator = pid->limMinInt;
    }


	//Derivative (band-limited differentiator)
    pid->differentiator = -(2.0f * pid->Kd * (measured_angle - pid->prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);

	
	//Compute output and apply limits
    pid->out = proportional + pid->integrator + pid->differentiator;
    if (pid->out > pid->limMax) {
        pid->out = pid->limMax;
    } else if (pid->out < pid->limMin) {
        pid->out = pid->limMin;
    }

	/* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measured_angle;

	/* Return controller output */
    return pid->out;
}


