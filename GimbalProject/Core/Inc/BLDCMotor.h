#include "main.h"
#include "MPU6050.h"



void set_desiredRoll(float desiredRoll);
void set_desiredPitch(float desiredPitch);
void set_operationMode(int16_t setMode);
void BLDC_PID(volatile MPU6050_t *targetOrientation,volatile MPU6050_t *stationaryOrientation);

void BLDC_Output(float Angle1, int MotorNum);



void BLDCEnable(int MotorNum);
void BLDCDisable(int MotorNum);