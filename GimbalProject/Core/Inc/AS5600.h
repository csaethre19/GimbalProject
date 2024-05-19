#ifndef AS5600_H
#define AS5600_H
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
 
 //not accurate
#define RAD_TO_DEG 57.295779513082320876798154814105

#define AS5600_ADDR 0x36

//Configuration Registers
#define ZCMO 0x00

#define ZPOS_l 0x02
#define ZPOS_h 0x01

#define MPOS_l 0x04
#define MPOS_h 0x03

#define MANG_l 0x06
#define MANG_h 0x05

#define CONF_l 0x08
#define CONF_h 0x07

//Ouptut Registers
#define RANG_l 0x0D
#define RANG_h 0x0C

#define ANG_l 0x0F
#define ANG_h 0x0E

//Status Registers
#define STATUS 0x0B
#define AGC 0x1A

//Configurations
#define PWMF_460 2b10
#define OUTS_PWM 2b10



//To change a configuration, read out the register, modify only the desired bits and write the new configuration. Blank fields may contain factory settings

// datastruct used to hold gyro/accel data that are read from MPU6050 device
// contains latest kalman filtered measurements 
typedef struct {
	
	uint16_t deviceAddr;
	uint16_t deviceAddrR;
  uint16_t deviceAddrW;
	
  uint16_t rawAngle;
	uint16_t angle;
	
	uint8_t agc;
	
	uint8_t magnetStatus;

} AS5600_t;

/*
	Initializes an MPU6050 IMU given a MPU6050_t type and a device address, specified by deviceAddr parameter.
	This function will wake up the device given the device address and an I2C line.
	Parameters for the device are set within the configuration registers such as
	full-scale range setting for gyroscope and accelerometer data, data sampling rate, 
	and enabling digital low pass filter setting. 
	If MPU6050 initialization is successfull, UART messages will print to console to confirm this.
*/
void AS5600_Init(volatile AS5600_t *dataStruct, uint16_t deviceAddr);

void AS5600_Set_Zero(volatile AS5600_t *dataStruct);

void AS5600_Magnet_Status(volatile AS5600_t *dataStruct);

void AS5600_Read_Angle(volatile AS5600_t *dataStruct);

#endif /* MPU6050_H */