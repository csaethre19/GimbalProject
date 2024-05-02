#include "MPU6050.h"
#include "arm_math.h"

/*.Q_angle = 0.005f,
        .Q_bias = 0.003f,
        .R_measure = 0.0001f,*/
Kalman_t KalmanPitch = {//KalmanX
        .Q_angle = 0.005f,
        .Q_bias = 0.009f,
        .R_measure = 0.0005f
};

Kalman_t KalmanRoll = {//KalmanY
        .Q_angle = 0.005f,
        .Q_bias = 0.009f,
        .R_measure = 0.0005f,
};

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
	
	// Setting full-scale range to +-1000 degress/sec
	I2C_WriteRegister(deviceAddr, GYRO_CONFIG, 0x10);
	I2C_SetRegAddress(deviceAddr, GYRO_CONFIG);
	int8_t gyro_config = I2C_ReadRegister(deviceAddr);
	if (gyro_config == 0x08) USART_Transmit_String("Successfully configured GYRO to +-1000 deg/s");
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
	I2C_WriteRegister(dataStruct->deviceAddr, 0x1B, 0x8);
	I2C_WriteRegister(dataStruct->deviceAddr, 0x1A, 0x05);
	I2C_WriteRegister(dataStruct->deviceAddr, 0x1C, 0x10);
	// Setting initial values for Kalman Filter parameters
	dataStruct->KalmanAnglePitch = 0.0;
	dataStruct->KalmanAngleRoll = 0.0;
	dataStruct->KalmanAngleUncertaintyPitch = 9;
	dataStruct->KalmanAngleUncertaintyRoll = 9;
}


void ReadGyroData(volatile MPU6050_t *dataStruct)
{
	uint8_t xhigh;
	uint8_t xlow;
	uint8_t yhigh;
	uint8_t ylow;
	uint8_t zhigh;
	uint8_t zlow;
	int8_t dataBuffer[6]; // reading X, Y, and Z 
	
	I2C_WriteRegister(dataStruct->deviceAddr, 0x1B, 0x8);
	
	I2C_ReadBurst(dataStruct->deviceAddr, GYRO_XOUT_HIGH, dataBuffer, 6); 
	xhigh = dataBuffer[0];
	xlow = dataBuffer[1];
	float x_raw = (int16_t)(xhigh << 8 | xlow);
	float gyro_x = x_raw/GYRO_LSB_SENS;
	yhigh = dataBuffer[2];
	ylow = dataBuffer[3];
	float y_raw = (int16_t)(yhigh << 8 | ylow);
	float gyro_y = y_raw/GYRO_LSB_SENS;
	zhigh = dataBuffer[4];
	zlow = dataBuffer[5];
	float z_raw = (int16_t)(zhigh << 8 | zlow);
	float gyro_z = z_raw/GYRO_LSB_SENS;

	
	dataStruct->Gyro_X_RAW = x_raw;
	dataStruct->Gx = gyro_x;
	dataStruct->Gyro_Y_RAW = y_raw;
	dataStruct->Gy = gyro_y;
	dataStruct->Gyro_Z_RAW = z_raw;
	dataStruct->Gz = gyro_z;
	
	//dataStruct->RateRoll = gyro_x;
	//dataStruct->RatePitch = gyro_y;
	//dataStruct->RateYaw = gyro_z;
}

void ReadAccelData(volatile MPU6050_t *dataStruct)
{
	float accel_x = 0;
	float accel_y = 0;
	float accel_z = 0;
	uint8_t xhigh;
	uint8_t xlow;
	uint8_t yhigh;
	uint8_t ylow;
	uint8_t zhigh;
	uint8_t zlow;
	int16_t x_raw;
	int16_t y_raw;
	int16_t z_raw;
	int8_t dataBuffer[6];
	
	I2C_WriteRegister(dataStruct->deviceAddr, 0x1A, 0x05);
	I2C_WriteRegister(dataStruct->deviceAddr, 0x1C, 0x10);
	I2C_ReadBurst(dataStruct->deviceAddr, ACC_XOUT_HIGH, dataBuffer, 6); 
	xhigh = dataBuffer[0];
	xlow = dataBuffer[1];
	x_raw = (int16_t)(xhigh << 8 | xlow);
	accel_x = (float)x_raw/ACC_LSB_SENS;
	yhigh = dataBuffer[2];
	ylow = dataBuffer[3];
	y_raw = (int16_t)(yhigh << 8 | ylow);
	accel_y = (float)y_raw/ACC_LSB_SENS;
	zhigh = dataBuffer[4];
	zlow = dataBuffer[5];
	z_raw = (int16_t)(zhigh << 8 | zlow);
	accel_z = (float)z_raw/ACC_LSB_SENS;
	
	dataStruct->Accel_X_RAW = x_raw;
	dataStruct->Ax = accel_x;
	dataStruct->Accel_Y_RAW = y_raw;
	dataStruct->Ay = accel_y;
	dataStruct->Accel_Z_RAW = z_raw;
	dataStruct->Az = accel_z;
	
	//dataStruct->AngleRoll = CalculateAngleRoll(accel_x, accel_y, accel_z);
	//dataStruct->AnglePitch = CalculateAnglePitch(accel_x, accel_y, accel_z);
}

float CalculateAngleRoll(float AccelX, float AccelY, float AccelZ)
{
	float stepvar = sqrt((AccelX*AccelX) + (AccelZ*AccelZ));
	stepvar = AccelY / stepvar;
	stepvar = atan(stepvar);
	stepvar = stepvar / (3.141592/180);
	return stepvar;
	//return atan(AccelY/sqrt((AccelX*AccelX)+(AccelZ*AccelZ)))*1/(3.141592/180);
}

float CalculateAnglePitch(float AccelX, float AccelY, float AccelZ)
{
	float stepvar = sqrt((AccelY*AccelY) + (AccelZ*AccelZ));
	stepvar = AccelX / stepvar;
	stepvar = -atan(stepvar);
	stepvar = stepvar / (3.141592/180);
	return stepvar;
	//return -atan(AccelX/sqrt((AccelY*AccelY)+(AccelZ*AvccelZ)))*1/(3.141592/180);
}


void KalmanFilter(volatile MPU6050_t *dataStruct)
{
	
	
	ReadGyroData(dataStruct);
	
	ReadAccelData(dataStruct);

	// Not using Magnetometer right now!
	//ReadMagData(dataStruct);

	
	dataStruct->KalmanAngleRoll = dataStruct->KalmanAngleRoll + (0.004 * dataStruct->RateRoll);
	dataStruct->KalmanAngleUncertaintyRoll = dataStruct->KalmanAngleUncertaintyRoll + (0.004 * 0.004 * 3 * 3);
	float KalmanGainRoll = dataStruct->KalmanAngleUncertaintyRoll * (1/(dataStruct->KalmanAngleUncertaintyRoll + (9)));
	dataStruct->KalmanAngleRoll = dataStruct->KalmanAngleRoll + (KalmanGainRoll * (dataStruct->AngleRoll - dataStruct->KalmanAngleRoll));
	dataStruct->KalmanAngleUncertaintyRoll = (1 - KalmanGainRoll) * dataStruct->KalmanAngleUncertaintyRoll;
	
	dataStruct->KalmanAnglePitch = dataStruct->KalmanAnglePitch + (0.004 * dataStruct->RatePitch);
	dataStruct->KalmanAngleUncertaintyPitch = dataStruct->KalmanAngleUncertaintyPitch + (0.004 * 0.004 * 3 * 3);
	float KalmanGainPitch = dataStruct->KalmanAngleUncertaintyPitch * (1/(dataStruct->KalmanAngleUncertaintyPitch + (9)));
	dataStruct->KalmanAnglePitch = dataStruct->KalmanAnglePitch + (KalmanGainPitch * (dataStruct->AnglePitch - dataStruct->KalmanAnglePitch));
	dataStruct->KalmanAngleUncertaintyPitch = (1 - KalmanGainPitch) * dataStruct->KalmanAngleUncertaintyPitch;
	

	//USART_Transmit_String("AxelY: ");
	//USART_Transmit_Float(dataStruct->Ay, 3);
	//USART_Transmit_Float(dataStruct->KalmanAnglePitch, 3);
	//USART_Transmit_Newline();
	
	//USART_Transmit_String("  AxelX: ");
	//USART_Transmit_Float(dataStruct->Ax, 3);
	//USART_Transmit_Float(dataStruct->KalmanAngleRoll, 3);
	//USART_Transmit_Newline();
	
	//USART_Transmit_String("  AxelZ: ");
	//USART_Transmit_Float(dataStruct->Az, 3);
	
	//USART_Transmit_String("Yaw: ");
	//USART_Transmit_Float(dataStruct->KalmanAngleYaw, 3);
	//USART_Transmit_Newline();
	
	//USART_Transmit_Newline();
}

void KFilter_2(volatile MPU6050_t *DataStruct){

	
	
	//ReadGyroData(DataStruct);
	//ReadAccelData(DataStruct);
	
		//New data present in raw low, high accel values, update accel data
		int16_t x_raw = (int16_t)(DataStruct->accel_xhigh << 8 | DataStruct->accel_xlow);
		float accel_x = (float)x_raw/ACC_LSB_SENS;
		int16_t y_raw = (int16_t)(DataStruct->accel_yhigh << 8 | DataStruct->accel_ylow);
		float accel_y = (float)y_raw/ACC_LSB_SENS;
		int16_t z_raw = (int16_t)(DataStruct->accel_zhigh << 8 | DataStruct->accel_zlow);
		float accel_z = (float)z_raw/ACC_LSB_SENS;
		DataStruct->Accel_X_RAW = x_raw;
		DataStruct->Ax = accel_x;
		DataStruct->Accel_Y_RAW = y_raw;
		DataStruct->Ay = accel_y;
		DataStruct->Accel_Z_RAW = z_raw;
		DataStruct->Az = accel_z;
		//New Data present in raw low, high gyro values, update gyro data
		float gyrox_raw = (int16_t)(DataStruct->gyro_xhigh << 8 | DataStruct->gyro_xlow);
		float gyro_x = gyrox_raw/GYRO_LSB_SENS;
		float gyroy_raw = (int16_t)(DataStruct->gyro_yhigh << 8 | DataStruct->gyro_ylow);
		float gyro_y = gyroy_raw/GYRO_LSB_SENS;
		float gyroz_raw = (int16_t)(DataStruct->gyro_zhigh << 8 | DataStruct->gyro_zlow);
		float gyro_z = gyroz_raw/GYRO_LSB_SENS;
		DataStruct->Gyro_X_RAW = gyrox_raw;
		DataStruct->Gx = gyro_x;
		DataStruct->Gyro_Y_RAW = gyroy_raw;
		DataStruct->Gy = gyro_y;
		DataStruct->Gyro_Z_RAW = gyroz_raw;
		DataStruct->Gz = gyro_z;

		DataStruct->dt = (double) (HAL_GetTick() - DataStruct->timer) / 500;
		DataStruct->timer = HAL_GetTick();

		float pitch;
		arm_atan2_f32(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW, &pitch);
		pitch = pitch * RAD_TO_DEG;
		
    if ((pitch < -90 && DataStruct->KalmanAnglePitch > 90) || (pitch > 90 && DataStruct->KalmanAnglePitch < -90)) {
        KalmanPitch.angle = pitch;
        DataStruct->KalmanAnglePitch = pitch;
    } else {
        DataStruct->KalmanAnglePitch = Kalman_getAngle(&KalmanPitch, pitch, DataStruct->Gy, DataStruct->dt);
    }
		
		float roll;
		arm_atan2_f32(DataStruct->Accel_Y_RAW, DataStruct->Accel_Z_RAW, &roll);
		roll = -roll * RAD_TO_DEG;
		
		if (fabs(DataStruct->KalmanAngleRoll) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    if((roll < -90 && DataStruct->KalmanAngleRoll > 90) || (roll > 90 && DataStruct->KalmanAngleRoll < -90)) {
		 KalmanRoll.angle = roll;
		 DataStruct->KalmanAngleRoll = roll;
			
	  } else {
		 DataStruct->KalmanAngleRoll = Kalman_getAngle(&KalmanRoll, roll, DataStruct->Gx, DataStruct->dt);
	  }
		
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt) {
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

		//ensure reported values range from -180 <-> 180
		if (Kalman->angle > 180) {Kalman->angle = Kalman->angle  - 360;}
		if (Kalman->angle < -180) {Kalman->angle = Kalman->angle  + 360;}
    return Kalman->angle;
};

