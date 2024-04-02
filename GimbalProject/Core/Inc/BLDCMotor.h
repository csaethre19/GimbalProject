#include "main.h"
#include "MPU6050.h"

void set_desiredRoll(float desiredRoll);
void set_desiredPitch(float desiredPitch);

void BLDC_PID(MPU6050_t *targetOrientation, MPU6050_t *stationaryOrientation);

void BLDC_Output(double Angle1, int MotorNum);

void initBLDCOutput(int MotorNum);
