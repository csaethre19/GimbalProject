#include "main.h"
#include "MPU6050.h"

void BLDC_PID(MPU6050_t *targetOrientation, MPU6050_t *stationaryOrientation)

void DCSetOutput(int Output, int MotorNum);

void initDCOutput(int MotorNum);
