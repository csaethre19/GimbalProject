#include "MPU6050.h"


void MPU_Init(volatile MPU6050_t *dataStruct, uint16_t deviceAddr)
{
	dataStruct->deviceAddr = deviceAddr;
	USART_Transmit_String("MPU6050 Address: ");
	USART_Transmit_Number(deviceAddr);
	USART_Transmit_Newline();
	
	// Check WHO_AM_I register
	I2C_SetRegAddress(deviceAddr, WHO_AM_I);
	int8_t whoAmI = I2C_ReadRegister(MPU6050_ADDR);
	int8_t expected_whoAmI = 0x68;
	if (whoAmI == expected_whoAmI) 	USART_Transmit_String("Successfully read WHO_AM_I"); 
	else USART_Transmit_String("Erorr: did not get expected WHO_AM_I"); 
	USART_Transmit_Newline();
	
	USART_Transmit_String("WHO_AM_I=");
	USART_Transmit_Number(whoAmI);
	USART_Transmit_Newline();
	
	// Wake up device and set clock to 8MHz
	I2C_WriteRegister(deviceAddr, PWR_MGMT_1, 0x00);
	I2C_SetRegAddress(deviceAddr, PWR_MGMT_1);
	int8_t pwr_mgmt = I2C_ReadRegister(deviceAddr);

if (pwr_mgmt == 0) 
	{
		USART_Transmit_String("MPU6050 Awake!");
		USART_Transmit_Newline();
	}
	
	// Setting full-scale range to +-500 degress/sec
	I2C_WriteRegister(deviceAddr, GYRO_CONFIG, 0x08);
	I2C_SetRegAddress(deviceAddr, GYRO_CONFIG);
	int8_t gyro_config = I2C_ReadRegister(deviceAddr);
	if (gyro_config == 0x08) USART_Transmit_String("Successfully configured GYRO to +-500 deg/s");
	USART_Transmit_Newline();
	
	// Setting accelerometer to +-8g
	I2C_WriteRegister(deviceAddr, ACC_CONFIG, 0x10);
	I2C_SetRegAddress(deviceAddr, ACC_CONFIG);
	int8_t acc_config = I2C_ReadRegister(deviceAddr);
	if (acc_config == 0x10) USART_Transmit_String("Successfully configured ACC to +-8g");	
	USART_Transmit_Newline();
	
	// Set SMPRT_DIV register to get 1kHz sample rate - when DLPF enabled Gyro Output Rate is 1kHz
	// sample rate = Gyro Output Rate / (1 + SMPLRT_DIV)
	I2C_WriteRegister(deviceAddr, SMPLRT_DIV, 0x00);
	I2C_SetRegAddress(deviceAddr, SMPLRT_DIV); 
	int8_t sample_rate_div = I2C_ReadRegister(deviceAddr);
	USART_Transmit_String("sample rate: ");
	USART_Transmit_Number(1 / (1 + sample_rate_div));
	USART_Transmit_String(" kHz");
	USART_Transmit_Newline();
	
	// Configure DLPF for balanced noise performance - setting to ~44Hz bandwidth
	I2C_WriteRegister(deviceAddr, CONFIG, 0x03);
	I2C_SetRegAddress(deviceAddr, CONFIG);
	int8_t config = I2C_ReadRegister(deviceAddr);
	if (config == 0x03) USART_Transmit_String("Enabled digital low pass filter");
	USART_Transmit_Newline();
	
	USART_Transmit_Newline();
	
	// Setting initial values for Kalman Filter parameters
	dataStruct->KalmanAnglePitch = 0.0;
	dataStruct->KalmanAngleRoll = 0.0;
	dataStruct->KalmanAngleUncertaintyPitch = 2*2;
	dataStruct->KalmanAngleUncertaintyRoll = 2*2;
}

void QMC_Init(void)
{
		I2C_WriteRegister(QMC_ADDR, 0x0B, 0x01);
		I2C_WriteRegister(QMC_ADDR, 0x09, 0x1D);
}

void ReadGyroData(volatile MPU6050_t *dataStruct)
{
	
	int8_t dataBuffer[6]; // reading X, Y, and Z
	
	I2C_ReadBurst(dataStruct->deviceAddr, GYRO_XOUT_HIGH, dataBuffer, 6); 
	
	int8_t xhigh = dataBuffer[0];
	int8_t xlow = dataBuffer[1];
	float x_raw = (int16_t)(xhigh << 8 | xlow);
	float gyro_x = x_raw/GYRO_LSB_SENS;
	dataStruct->Gyro_X_RAW = x_raw;
	dataStruct->Gx = gyro_x;
	dataStruct->RateRoll = gyro_x;
	
	
	int8_t yhigh = dataBuffer[2];
	int8_t ylow = dataBuffer[3];
	float y_raw = (int16_t)(yhigh << 8 | ylow);
	float gyro_y = y_raw/GYRO_LSB_SENS;
	dataStruct->Gyro_Y_RAW = y_raw;
	dataStruct->Gy = gyro_y;
	dataStruct->RatePitch = gyro_y;
	
	int8_t zhigh = dataBuffer[4];
	int8_t zlow = dataBuffer[5];
	float z_raw = (int16_t)(zhigh << 8 | zlow);
	float gyro_z = z_raw/GYRO_LSB_SENS;
	dataStruct->Gyro_Z_RAW = z_raw;
	dataStruct->Gz = gyro_z;
	dataStruct->RateYaw = gyro_z;

	
}

void ReadAccelData(volatile MPU6050_t *dataStruct)
{
	
	int8_t dataBuffer[6];
	
	I2C_ReadBurst(dataStruct->deviceAddr, ACC_XOUT_HIGH, dataBuffer, 6); 
	
	int8_t xhigh = dataBuffer[0];
	int8_t xlow = dataBuffer[1];
	int16_t x_raw = (int16_t)(xhigh << 8 | xlow);
	float accel_x = (float)x_raw/ACC_LSB_SENS;
	dataStruct->Accel_X_RAW = x_raw;
	dataStruct->Ax = accel_x;
	
	int8_t yhigh = dataBuffer[2];
	int8_t ylow = dataBuffer[3];
	int16_t y_raw = (int16_t)(yhigh << 8 | ylow);
	float accel_y = (float)y_raw/ACC_LSB_SENS;
	dataStruct->Accel_Y_RAW = y_raw;
	dataStruct->Ay = accel_y;
	
	
	int8_t zhigh = dataBuffer[4];
	int8_t zlow = dataBuffer[5];
	int16_t z_raw = (int16_t)(zhigh << 8 | zlow);
	float accel_z = (float)z_raw/ACC_LSB_SENS;
	dataStruct->Accel_Z_RAW = z_raw;
	dataStruct->Az = accel_z;

	
	dataStruct->AngleRoll = CalculateAngleRoll(accel_x, accel_y, accel_z);
	dataStruct->AnglePitch = CalculateAnglePitch(accel_x, accel_y, accel_z);
}

float CalculateAngleRoll(float AccelX, float AccelY, float AccelZ)
{
	return atan(AccelY/sqrt(AccelX*AccelX+AccelZ*AccelZ))*1/(3.142/180);
}

float CalculateAnglePitch(float AccelX, float AccelY, float AccelZ)
{
	return -atan(AccelX/sqrt(AccelY*AccelY+AccelZ*AccelZ))*1/(3.142/180);
}

void ReadMagData(volatile MPU6050_t *dataStruct)
{
	int8_t dataBuffer[4]; // reading X and Y
	
	I2C_ReadBurst(dataStruct->deviceAddr, 0x00, dataBuffer, 4); 
	
	int8_t xhigh = dataBuffer[0];
	int8_t xlow = dataBuffer[1];
	float x_raw = (int16_t)(xhigh << 8 | xlow);
	float mag_x = x_raw/MAG_LSB_SENS; 
	
	int8_t yhigh = dataBuffer[2];
	int8_t ylow = dataBuffer[3];
	float y_raw = (int16_t)(yhigh << 8 | ylow);
	float mag_y = y_raw/MAG_LSB_SENS; 
	
	// Calculating Yaw Angle
	dataStruct->AngleYaw = atan2(-mag_y, mag_x);
}

void KalmanFilter(volatile MPU6050_t *dataStruct)
{
	ReadGyroData(dataStruct);
	ReadAccelData(dataStruct);
	// Not using Magnetometer right now!
	//ReadMagData(dataStruct);
	
	// ROLL KALMAN:
	// State Prediction: predicting new angle by integrating rate of change using RateRoll and time step of 0.004
	dataStruct->KalmanAngleRoll = dataStruct->KalmanAngleRoll + dataStruct->RateRoll*0.004;
	
	// Uncertainty Prediction: updating uncertainty of angle estimate by fixed amount (0.004 * 0.004 * 4 * 4 is arbitrary)
	dataStruct->KalmanAngleUncertaintyRoll = dataStruct->KalmanAngleUncertaintyRoll + 0.004 * 0.004 * 4 * 4;
	
	// Kalman Gain Calculation: balancing the estimated state's uncertainty with the measurement's uncertainty (3*3 is used as measurement noise variance)
	float kalmanGainRoll = dataStruct->KalmanAngleUncertaintyRoll * 1/(1*dataStruct->KalmanAngleUncertaintyRoll + 3 * 3);
	
	// State Update: correcting the predicted state with the new information which incorporates AngleRoll accelerometer-based angle
	dataStruct->KalmanAngleRoll = dataStruct->KalmanAngleRoll + kalmanGainRoll * (dataStruct->AngleRoll-dataStruct->KalmanAngleRoll);
	
	// Uncertainty Update:
	dataStruct->KalmanAngleUncertaintyRoll = (1-kalmanGainRoll) * dataStruct->KalmanAngleUncertaintyRoll;
	
	// PITCH KALMAN:
	// State Prediction: predicting new angle by integrating rate of change using RatePitch and time step of 0.004
	dataStruct->KalmanAnglePitch = dataStruct->KalmanAnglePitch + dataStruct->RatePitch*0.004;
	// Uncertainty Prediction: updating uncertainty of angle estimate by fixed amount (0.004 * 0.004 * 4 * 4 is arbitrary)
	dataStruct->KalmanAngleUncertaintyPitch = dataStruct->KalmanAngleUncertaintyPitch + 0.004 * 0.004 * 4 * 4;
	
	// Kalman Gain Calculation: balancing the estimated state's uncertainty with the measurement's uncertainty (3*3 is used as measurement noise variance)
	float kalmanGainPitch = dataStruct->KalmanAngleUncertaintyPitch * 1/(1*dataStruct->KalmanAngleUncertaintyPitch + 3 * 3);
	
	// State Update: correcting the predicted state with the new information which incorporates AnglePitch accelerometer-based angle
	dataStruct->KalmanAnglePitch = dataStruct->KalmanAnglePitch + kalmanGainPitch * (dataStruct->AnglePitch-dataStruct->KalmanAnglePitch);
	
	// Uncertainty Update:
	dataStruct->KalmanAngleUncertaintyPitch = (1-kalmanGainPitch) * dataStruct->KalmanAngleUncertaintyPitch;
	
	// YAW KALMAN:
	// State Prediction: predicting new angle by integrating rate of change using RatePitch and time step of 0.004
	//dataStruct->KalmanAngleYaw = dataStruct->KalmanAngleYaw+ dataStruct->RateYaw*0.004;
	// Uncertainty Prediction: updating uncertainty of angle estimate by fixed amount (0.004 * 0.004 * 4 * 4 is arbitrary)
	//dataStruct->KalmanAngleUncertaintyYaw = dataStruct->KalmanAngleUncertaintyYaw + 0.004 * 0.004 * 4 * 4;
	
	// Kalman Gain Calculation: balancing the estimated state's uncertainty with the measurement's uncertainty (3*3 is used as measurement noise variance)
	//float kalmanGainYaw = dataStruct->KalmanAngleUncertaintyYaw * 1/(1*dataStruct->KalmanAngleUncertaintyYaw + 3 * 3);
	
	// State Update: correcting the predicted state with the new information which incorporates AnglePitch accelerometer-based angle
	//dataStruct->KalmanAngleYaw = dataStruct->KalmanAngleYaw + kalmanGainPitch * (dataStruct->AngleYaw-dataStruct->KalmanAngleYaw);
	
	// Uncertainty Update:
	//dataStruct->KalmanAngleUncertaintyYaw = (1-kalmanGainYaw) * dataStruct->KalmanAngleUncertaintyYaw;
	
	
	USART_Transmit_String("Pitch: ");
	USART_Transmit_Float(dataStruct->KalmanAnglePitch, 3);
	USART_Transmit_Newline();
	
	USART_Transmit_String("Roll : ");
	USART_Transmit_Float(dataStruct->KalmanAngleRoll, 3);
	USART_Transmit_Newline();
	
	//USART_Transmit_String("Yaw: ");
	//USART_Transmit_Float(dataStruct->KalmanAngleYaw, 3);
	//USART_Transmit_Newline();
	
	USART_Transmit_Newline();
	
	
}
