#include "AS5600.h"
#include "arm_math.h"

void AS5600_Init(volatile AS5600_t *dataStruct, uint16_t deviceAddr)
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
	
}
