#include <math.h>
#include "main.h"
#include "I2C.h"
#include "USART.h"


#define MPU6050_ADDR   0x68
#define WHO_AM_I       0x75

#define PWR_MGMT_1     0x6B
#define SMPLRT_DIV     0x19
#define CONFIG         0x1A

#define GYRO_CONFIG    0x1B
#define GYRO_LSB_SENS  65.5 // for +-500 deg/s
#define GYRO_XOUT_HIGH 0x43
#define GYRO_XOUT_LOW  0x44

#define GYRO_YOUT_HIGH 0x45
#define GYRO_YOUT_LOW  0x46

#define GYRO_ZOUT_HIGH 0x47
#define GYRO_ZOUT_LOW  0x48

#define ACC_CONFIG     0x1A
#define ACC_LSB_SENS   4096.0 // for +-8g
#define ACC_XOUT_HIGH  0x3B
#define ACC_XOUT_LOW   0x3C

#define ACC_YOUT_HIGH  0x3D
#define ACC_YOUT_LOW   0x3E

#define ACC_ZOUT_HIGH  0x3F
#define ACC_ZOUT_LOW   0x40

#define TEMP_OUT_HIGH  0x41
#define TEMP_OUT_LOW   0x42

typedef struct {
	
	uint16_t deviceAddr;
	
	int16_t Accel_X_RAW;
	int16_t Accel_Y_RAW;
	int16_t Accel_Z_RAW;
	float Ax; 
	float Ay;
	float Az;

	int16_t Gyro_X_RAW;
	int16_t Gyro_Y_RAW;
	int16_t Gyro_Z_RAW;
	float Gx;
	float Gy;
	float Gz;

	float KalmanAngleX;
	float KalmanAngleY;
	
} MPU6050_t;

void MPU_Init(void);

// TODO: add arg for a MPU6050_t that is passed by reference to hold all the data
void ReadGyroData(float* data);

void ReadAccelData(float* data);

int CalculateAccelPitchAngle(int16_t AccelX, int16_t AccelY, int16_t AccelZ);

int CalculateGyroPitchAngle(int16_t prevRate, int16_t currentRate, int samplePeriod);
