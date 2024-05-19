#include "main.h"
#include "MPU6050.h"
#include "HMC5883.h"

typedef struct {

	float Kp;
	float Ki;
	float Kd;
	
	float tau;
	
	//Output limits
	float limMin;
	float limMax;
	
	//Integrator limits
	float limMinInt;
	float limMaxInt;
	
	//Sample Time
	int T;
	
	//Controller "memory"
	float integrator;
	float prevError;
	float differentiator;
	float prevMeasurement;
	
	//controller output
	float out;
	
} PIDController;



void set_desiredRoll(float desiredRoll);
void set_desiredPitch(float desiredPitch);
void set_desiredYaw(float desiredYaw);
void set_operationModeRollPitch(int16_t setMode);
void set_operationModeYaw(int16_t setMode);

void YAW_PID(volatile HMC5883_t yawSense);

void BLDC_PID(volatile MPU6050_t *targetOrientation,volatile MPU6050_t *stationaryOrientation);

void BLDC_Output(float Angle1, int MotorNum);



void BLDCEnable(int MotorNum);
void BLDCDisable(int MotorNum);

void BLDC_PID_Init();
float PIDController_Update(PIDController * pid, float target_angle, float measured_angle);