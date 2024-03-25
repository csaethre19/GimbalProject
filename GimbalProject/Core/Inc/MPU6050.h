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
#define ACC_LSB_SENS   4096 // for +-8g
#define ACC_XOUT_HIGH  0x3B
#define ACC_XOUT_LOW   0x3C

#define ACC_YOUT_HIGH  0x3D
#define ACC_YOUT_LOW   0x3E

#define ACC_ZOUT_HIGH  0x3F
#define ACC_ZOUT_LOW   0x40

#define TEMP_OUT_HIGH  0x41
#define TEMP_OUT_LOW   0x42

void MPU_Init(void);

void ReadGyroData(int16_t* data);

void ReadAccelData(int16_t* data);
