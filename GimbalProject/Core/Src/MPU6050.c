#include "MPU6050.h"


void MPU_Init(MPU6050_t *dataStruct, uint16_t deviceAddr)
{
	dataStruct->deviceAddr = deviceAddr;
	
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

void ReadGyroData(MPU6050_t *dataStruct)
{
	USART_Transmit_String("Reading Gyroscope Data");
	USART_Transmit_Newline();
	
	int8_t dataBuffer[6]; // reading X, Y, and Z
	
	I2C_ReadBurst(dataStruct->deviceAddr, GYRO_XOUT_HIGH, dataBuffer, 6); 
	
	int8_t xhigh = dataBuffer[0];
	int8_t xlow = dataBuffer[1];
	float x_raw = (int16_t)(xhigh << 8 | xlow);
	float gyro_x = x_raw/GYRO_LSB_SENS;
	dataStruct->Gyro_X_RAW = x_raw;
	dataStruct->Gx = gyro_x;
	dataStruct->RateRoll = gyro_x;
	
	USART_Transmit_String("X_RAW: ");
	USART_Transmit_Number(x_raw);
	USART_Transmit_Newline();
	
	USART_Transmit_String("X: ");
	USART_Transmit_Float(gyro_x, 4);
	USART_Transmit_Newline();
	
	int8_t yhigh = dataBuffer[2];
	int8_t ylow = dataBuffer[3];
	float y_raw = (int16_t)(yhigh << 8 | ylow);
	float gyro_y = y_raw/GYRO_LSB_SENS;
	dataStruct->Gyro_Y_RAW = y_raw;
	dataStruct->Gy = gyro_y;
	dataStruct->RatePitch = gyro_y;
	
	USART_Transmit_String("Y_RAW: ");
	USART_Transmit_Number(y_raw);
	USART_Transmit_Newline();
	
	USART_Transmit_String("Y: ");
	USART_Transmit_Float(gyro_y, 4);
	USART_Transmit_Newline();
	
	int8_t zhigh = dataBuffer[4];
	int8_t zlow = dataBuffer[5];
	float z_raw = (int16_t)(zhigh << 8 | zlow);
	float gyro_z = z_raw/GYRO_LSB_SENS;
	dataStruct->Gyro_Z_RAW = z_raw;
	dataStruct->Gz = gyro_z;
	
	USART_Transmit_String("Z_RAW: ");
	USART_Transmit_Number(z_raw);
	USART_Transmit_Newline();
	
	USART_Transmit_String("Z: ");
	USART_Transmit_Float(gyro_z, 4);
	USART_Transmit_Newline();
	
	USART_Transmit_Newline();
	
}

void ReadAccelData(MPU6050_t *dataStruct)
{
	USART_Transmit_String("Reading Accelerometer Data");
	USART_Transmit_Newline();
	
	int8_t dataBuffer[6];
	
	I2C_ReadBurst(dataStruct->deviceAddr, ACC_XOUT_HIGH, dataBuffer, 6); 
	
	int8_t xhigh = dataBuffer[0];
	int8_t xlow = dataBuffer[1];
	int16_t x_raw = (int16_t)(xhigh << 8 | xlow);
	float accel_x = (float)x_raw/ACC_LSB_SENS;
	dataStruct->Accel_X_RAW = x_raw;
	dataStruct->Ax = accel_x;
	
	USART_Transmit_String("X_RAW: ");
	USART_Transmit_Number(x_raw);
	USART_Transmit_Newline();
	
	USART_Transmit_String("X: ");
	USART_Transmit_Float(accel_x, 4);
	USART_Transmit_Newline();
	
	int8_t yhigh = dataBuffer[2];
	int8_t ylow = dataBuffer[3];
	int16_t y_raw = (int16_t)(yhigh << 8 | ylow);
	float accel_y = (float)y_raw/ACC_LSB_SENS;
	dataStruct->Accel_Y_RAW = y_raw;
	dataStruct->Ay = accel_y;
	
	USART_Transmit_String("Y_RAW: ");
	USART_Transmit_Number(y_raw);
	USART_Transmit_Newline();
	
	USART_Transmit_String("Y: ");
	USART_Transmit_Float(accel_y, 4);
	USART_Transmit_Newline();
	
	int8_t zhigh = dataBuffer[4];
	int8_t zlow = dataBuffer[5];
	int16_t z_raw = (int16_t)(zhigh << 8 | zlow);
	float accel_z = (float)z_raw/ACC_LSB_SENS;
	dataStruct->Accel_Z_RAW = z_raw;
	dataStruct->Az = accel_z;
	
	USART_Transmit_String("Z_RAW: ");
	USART_Transmit_Number(z_raw);
	USART_Transmit_Newline();
	
	USART_Transmit_String("Z: ");
	USART_Transmit_Float(accel_z, 4);
	USART_Transmit_Newline();
	
	USART_Transmit_Newline();
	
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

void KalmanFilter(MPU6050_t *dataStruct)
{
	ReadGyroData(dataStruct);
	ReadAccelData(dataStruct);
	
	dataStruct->KalmanAngleRoll = dataStruct->KalmanAngleRoll + dataStruct->RateRoll*0.004;
	dataStruct->KalmanAngleUncertaintyRoll = dataStruct->KalmanAngleUncertaintyRoll + 0.004 * 0.004 * 4 * 4;
	float kalmanGainRoll = dataStruct->KalmanAngleUncertaintyRoll * 1/(1*dataStruct->KalmanAngleUncertaintyRoll + 3 * 3);
	dataStruct->KalmanAngleRoll = dataStruct->KalmanAngleRoll + kalmanGainRoll * (dataStruct->AngleRoll-dataStruct->KalmanAngleRoll);
	
	//USART_Transmit_String("KalmanAngleRoll: ");
	//USART_Transmit_Float(dataStruct->KalmanAngleRoll, 4);
	//USART_Transmit_Newline();
	
	dataStruct->KalmanAnglePitch = dataStruct->KalmanAnglePitch + dataStruct->RatePitch*0.004;
	dataStruct->KalmanAngleUncertaintyPitch = dataStruct->KalmanAngleUncertaintyPitch + 0.004 * 0.004 * 4 * 4;
	float kalmanGainPitch = dataStruct->KalmanAngleUncertaintyPitch * 1/(1*dataStruct->KalmanAngleUncertaintyPitch + 3 * 3);
	dataStruct->KalmanAnglePitch = dataStruct->KalmanAnglePitch + kalmanGainPitch * (dataStruct->AnglePitch-dataStruct->KalmanAnglePitch);
	
	//USART_Transmit_String("KalmanAnglePitch: ");
	//USART_Transmit_Float(dataStruct->KalmanAnglePitch, 4);
	//USART_Transmit_Newline();
	
	//USART_Transmit_Newline();
}
