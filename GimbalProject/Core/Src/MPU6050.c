#include "MPU6050.h"


void MPU_Init()
{
	// Check WHO_AM_I register
	I2C_SetRegAddress(MPU6050_ADDR, WHO_AM_I);
	int8_t whoAmI = I2C_ReadRegister(MPU6050_ADDR);
	int8_t expected_whoAmI = 0x68;
	if (whoAmI == expected_whoAmI) 	USART_Transmit_String("Successfully read WHO_AM_I"); 
	else USART_Transmit_String("Erorr: did not get expected WHO_AM_I"); 
	
	USART_Transmit_Newline();
	
	USART_Transmit_String("WHO_AM_I=");
	USART_Transmit_Number(whoAmI);
	USART_Transmit_Newline();
	
	USART_Transmit_Newline();
	
	// Wake up device and set clock to 8MHz
	I2C_WriteRegister(MPU6050_ADDR, PWR_MGMT_1, 0x00);
	I2C_SetRegAddress(MPU6050_ADDR, PWR_MGMT_1);
	int8_t pwr_mgmt = I2C_ReadRegister(MPU6050_ADDR);

if (pwr_mgmt == 0) 
	{
		USART_Transmit_String("MPU6050 Awake!");
		USART_Transmit_Newline();
	}
	
	// Setting full-scale range to +-500 degress/sec
	I2C_WriteRegister(MPU6050_ADDR, GYRO_CONFIG, 0x08);
	I2C_SetRegAddress(MPU6050_ADDR, GYRO_CONFIG);
	int8_t gyro_config = I2C_ReadRegister(MPU6050_ADDR);
	if (gyro_config == 0x08) USART_Transmit_String("Successfully configured GYRO to +-500 deg/s");
	
	USART_Transmit_Newline();
	
	// Setting accelerometer to +-8g
	I2C_WriteRegister(MPU6050_ADDR, ACC_CONFIG, 0x10);
	I2C_SetRegAddress(MPU6050_ADDR, ACC_CONFIG);
	int8_t acc_config = I2C_ReadRegister(MPU6050_ADDR);
	if (acc_config == 0x10) USART_Transmit_String("Successfully configured ACC to +-8g");	
	
	USART_Transmit_Newline();
	
	// Set SMPRT_DIV register to get 1kHz sample rate - when DLPF enabled Gyro Output Rate is 1kHz
	// sample rate = Gyro Output Rate / (1 + SMPLRT_DIV)
	I2C_WriteRegister(MPU6050_ADDR, SMPLRT_DIV, 0x00);
	I2C_SetRegAddress(MPU6050_ADDR, SMPLRT_DIV); 
	int8_t sample_rate_div = I2C_ReadRegister(MPU6050_ADDR);
	USART_Transmit_String("sample rate: ");
	USART_Transmit_Number(1 / (1 + sample_rate_div));
	USART_Transmit_String(" kHz");
	
	USART_Transmit_Newline();
	
	// Configure DLPF for balanced noise performance - setting to ~44Hz bandwidth
	I2C_WriteRegister(MPU6050_ADDR, CONFIG, 0x03);
	I2C_SetRegAddress(MPU6050_ADDR, CONFIG);
	int8_t config = I2C_ReadRegister(MPU6050_ADDR);
	if (config == 0x03) USART_Transmit_String("Enabled digital low pass filter");
	
	USART_Transmit_Newline();
}

int16_t ReadGyroX(void)
{
	int8_t dataBuffer[2];
	I2C_ReadBurst(MPU6050_ADDR, GYRO_XOUT_HIGH, dataBuffer, 2); // reading 4 bytes to get the high/low X and the high/low Y
	
	int8_t xhigh = dataBuffer[0];
	int8_t xlow = dataBuffer[1];
	int16_t x_raw = (int16_t)(xhigh << 8 | xlow);
	int16_t gyro_x = x_raw/GYRO_LSB_SENS;
	
	USART_Transmit_String("X_RAW: ");
	USART_Transmit_Number(x_raw);
	USART_Transmit_Newline();
	
	USART_Transmit_String("X: ");
	USART_Transmit_Number(gyro_x);
	USART_Transmit_Newline();
	
	return gyro_x;
}

void ReadGyroData(int16_t* data)
{
	USART_Transmit_String("Reading Gyroscope Data");
	USART_Transmit_Newline();
	
	int8_t dataBuffer[4];
	
	I2C_ReadBurst(MPU6050_ADDR, GYRO_XOUT_HIGH, dataBuffer, 4); 
	
	int8_t xhigh = dataBuffer[0];
	int8_t xlow = dataBuffer[1];
	int16_t x_raw = (int16_t)(xhigh << 8 | xlow);
	int16_t gyro_x = x_raw/GYRO_LSB_SENS;
	data[0] = gyro_x;
	
	USART_Transmit_String("X_RAW: ");
	USART_Transmit_Number(x_raw);
	USART_Transmit_Newline();
	
	USART_Transmit_String("X: ");
	USART_Transmit_Number(gyro_x);
	USART_Transmit_Newline();
	
	int8_t yhigh = dataBuffer[2];
	int8_t ylow = dataBuffer[3];
	int16_t y_raw = (int16_t)(yhigh << 8 | ylow);
	int16_t gyro_y = y_raw/GYRO_LSB_SENS;
	data[1] = gyro_y;
	
	USART_Transmit_String("Y_RAW: ");
	USART_Transmit_Number(y_raw);
	USART_Transmit_Newline();
	
	USART_Transmit_String("Y: ");
	USART_Transmit_Number(gyro_y);
	USART_Transmit_Newline();
	
}

void ReadAccelData(int16_t* data)
{
	USART_Transmit_String("Reading Accelerometer Data");
	USART_Transmit_Newline();
	
	int8_t dataBuffer[4];
	
	I2C_ReadBurst(MPU6050_ADDR, ACC_XOUT_HIGH, dataBuffer, 4); 
	
	int8_t xhigh = dataBuffer[0];
	int8_t xlow = dataBuffer[1];
	int16_t x_raw = (int16_t)(xhigh << 8 | xlow);
	int16_t accel_x = x_raw/GYRO_LSB_SENS;
	data[0] = accel_x;
	
	USART_Transmit_String("X_RAW: ");
	USART_Transmit_Number(x_raw);
	USART_Transmit_Newline();
	
	USART_Transmit_String("X: ");
	USART_Transmit_Number(accel_x);
	USART_Transmit_Newline();
	
	int8_t yhigh = dataBuffer[2];
	int8_t ylow = dataBuffer[3];
	int16_t y_raw = (int16_t)(yhigh << 8 | ylow);
	int16_t accel_y = y_raw/GYRO_LSB_SENS;
	data[1] = accel_y;
	
	USART_Transmit_String("Y_RAW: ");
	USART_Transmit_Number(y_raw);
	USART_Transmit_Newline();
	
	USART_Transmit_String("Y: ");
	USART_Transmit_Number(accel_y);
	USART_Transmit_Newline();
}
