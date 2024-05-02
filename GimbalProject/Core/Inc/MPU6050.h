#ifndef MPU6050_H
#define MPU6050_H
/*
* 	MPU6050 IMU reading and filtering functionality.
*
* 	KalmanFilter() is the function that should be called by the main application to 
*		read and process data. 
*/
#include <math.h>
#include "main.h"
#include "I2C.h"
#include "USART.h"

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variable and Type Declarations
 *  -------------------------------------------------------------------------------------------------------------
 */
 
#define RAD_TO_DEG 57.295779513082320876798154814105


#define MPU6050_ADDR   0x68
#define WHO_AM_I       0x75

#define PWR_MGMT_1     0x6B
#define SMPLRT_DIV     0x19
#define CONFIG         0x1A

#define GYRO_CONFIG    0x1B
#define GYRO_LSB_SENS  32.8 // for +-1000 deg/s
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

// datastruct used to hold gyro/accel data that are read from MPU6050 device
// contains latest kalman filtered measurements 
typedef struct {
	
	uint16_t deviceAddr;
	
	uint8_t accel_xhigh;
	uint8_t accel_xlow;
	uint8_t accel_yhigh;
	uint8_t accel_ylow;
	uint8_t accel_zhigh;
	uint8_t accel_zlow;
	
	uint8_t gyro_xhigh;
	uint8_t gyro_xlow;
	uint8_t gyro_yhigh;
	uint8_t gyro_ylow;
	uint8_t gyro_zhigh;
	uint8_t gyro_zlow;
	
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
	
	float RateRoll;
	float RatePitch;
	float RateYaw;
	
	float AngleRoll;
	float AnglePitch;
	float AngleYaw;
	
	float KalmanAngleRoll;
	float KalmanAnglePitch;
	float KalmanAngleYaw;
	
	float KalmanAngleUncertaintyRoll;
	float KalmanAngleUncertaintyPitch;
	float KalmanAngleUncertaintyYaw;
	
	double dt;//Time since last KalmanFilter Execution
	double timer;//Time of last KalmanFilter Execution
	
} MPU6050_t;


// Kalman structure
typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

/*
	Initializes an MPU6050 IMU given a MPU6050_t type and a device address, specified by deviceAddr parameter.
	This function will wake up the device given the device address and an I2C line.
	Parameters for the device are set within the configuration registers such as
	full-scale range setting for gyroscope and accelerometer data, data sampling rate, 
	and enabling digital low pass filter setting. 
	If MPU6050 initialization is successfull, UART messages will print to console to confirm this.
*/
void MPU_Init(volatile MPU6050_t *dataStruct, uint16_t deviceAddr);


/*

*/
void QMC_Init(void);

/*
	Reads Gyroscope data for all three axes (X, Y, and Z) from specified MPU6050 device.
	Uses I2C read burst to get data and saves raw and converted values in dataStruct.
	Conversion is based on the raw value divided by the LSB sensitivity constant (macro defined in this file).
*/
void ReadGyroData(volatile MPU6050_t *dataStruct);

/*
	Reads Accelerometer data for all three axes (X, Y, and Z) from specified MPU6050 device.
	Uses I2C read burst to get data and saves raw and converted values in dataStruct.
	Conversion is based on the raw value divided by the LSB sensitivity constant (macro defined in this file).
	
	The Angle Roll and Angle Pitch are also calculated using all data from all three accelerometer axes and
	saved into dataStruct. 
*/
void ReadAccelData(volatile MPU6050_t *dataStruct);


/*
	Given X, Y, and Z accelerometer data calculates the roll angle. 
*/
float CalculateAngleRoll(float AccelX, float AccelY, float AccelZ);

/*
	Given X, Y, and Z accelerometer data calculates the roll pitch. 
*/
float CalculateAnglePitch(float AccelX, float AccelY, float AccelZ);


/*
	Reads gyroscope and accelerometer data from MPU6050 (reads will save applicable data in dataStruct).
	Calculates kalman filtered values for pitch and roll given the values saved in dataStruct for the 
	specified MPU6050 device. 
*/
void KalmanFilter(volatile MPU6050_t *dataStruct);

void KFilter_2(volatile MPU6050_t *dataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

#endif /* MPU6050_H */