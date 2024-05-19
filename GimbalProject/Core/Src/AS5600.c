#include "AS5600.h"
#include "arm_math.h"

void AS5600_Init(volatile AS5600_t *dataStruct, uint16_t deviceAddr)
{
	dataStruct->deviceAddr = deviceAddr;
	USART_Transmit_String("AS5600 Address: ");
	USART_Transmit_Number(deviceAddr);
	USART_Transmit_Newline();
	//Who-AM_I does not exist in this component
	/*// Check WHO_AM_I register
	I2C_SetRegAddress(deviceAddr, WHO_AM_I);
	int8_t whoAmI = I2C_ReadRegister(AS5600_ADDR);
	int8_t expected_whoAmI = 0x68;
	if (whoAmI == expected_whoAmI) 	USART_Transmit_String("Successfully read WHO_AM_I"); 
	else USART_Transmit_String("Erorr: did not get expected WHO_AM_I"); 
	USART_Transmit_Newline();
	
	USART_Transmit_String("WHO_AM_I=");
	USART_Transmit_Number(whoAmI);
	USART_Transmit_Newline();*/
	
	// Wake up device and read register change necessary bits and then write that back
	I2C_SetRegAddress(AS5600_ADDR, CONF_l);
	int8_t conf_l = I2C_ReadRegister(AS5600_ADDR);
	conf_l = 0b10100000;
	I2C_WriteRegister(AS5600_ADDR, CONF_l, conf_l);
	
	I2C_SetRegAddress(AS5600_ADDR, CONF_h);
	int8_t conf_h = I2C_ReadRegister(AS5600_ADDR);
	conf_h &= ~(1<<5);//watchdog off
	conf_h &= ~(1<<4);//fast filter = only slow filter
	conf_h &= ~(1<<3);
	conf_h &= ~(1<<2);
	conf_h &= ~(1<<1); //slow filter 16x
	conf_h &= ~(1<<0);
	I2C_WriteRegister(AS5600_ADDR, CONF_l, conf_l);
	//USART_Transmit_String("Init Finished: ");
	//USART_Transmit_Newline();
}

void AS5600_Set_Zero(volatile AS5600_t *dataStruct)
{
	//read the raw angle
	I2C_SetRegAddress(AS5600_ADDR, RANG_h);
	int8_t rang_h = I2C_ReadRegister(AS5600_ADDR);
	I2C_SetRegAddress(AS5600_ADDR, RANG_l);
	int8_t rang_l = I2C_ReadRegister(AS5600_ADDR);
	//combine angle to number
	int16_t rang = (int16_t)((rang_h << 8) | rang_l);
	//USART_Transmit_String("Raw Angle=");
	//USART_Transmit_Number(rang);
	//USART_Transmit_Newline();
	//write the raw angle to zpos
	I2C_WriteRegister(AS5600_ADDR, ZPOS_h, rang_h);
	I2C_WriteRegister(AS5600_ADDR, ZPOS_l, rang_l);
}

void AS5600_Magnet_Status(volatile AS5600_t *dataStruct)
{
	//read magnet register
	I2C_SetRegAddress(AS5600_ADDR, STATUS);
	int8_t status = I2C_ReadRegister(AS5600_ADDR);
	
	//set to 0 if hi, 1 if good, 2 if low
	if(status & (1<<3)){
		dataStruct->magnetStatus = 0;
		//USART_Transmit_String("Magnetic field is too large");
	  //USART_Transmit_Newline();
	}else if(status & (1<<5)){
    dataStruct->magnetStatus = 1;
		//USART_Transmit_String("Magnetic field is just right");
	  //USART_Transmit_Newline();
	}else{
		dataStruct->magnetStatus = 2;
		//USART_Transmit_String("Magnetic field is too small");
	  ///USART_Transmit_Newline();
	}
}

void AS5600_Read_Angle(volatile AS5600_t *dataStruct)
{
	int8_t rawAngle_h;
	int8_t rawAngle_l;
	int8_t angle_h;
	int8_t angle_l;
	int8_t dataBuffer[4];
	I2C_ReadBurst(dataStruct->deviceAddr, RANG_h, dataBuffer, 4);
	rawAngle_h = dataBuffer[0];
	rawAngle_l = dataBuffer[1];
	float rawAngle = (int16_t)(rawAngle_h << 8 | rawAngle_l);
	angle_h = dataBuffer[2];
	angle_l = dataBuffer[3];
	float angle = (int16_t)(angle_h << 8 | angle_l);
	
	//USART_Transmit_String("Angle: ");
	//USART_Transmit_Number(angle);
	//USART_Transmit_Newline();
	
	//USART_Transmit_String("Raw Angle: ");
	//USART_Transmit_Number(rawAngle);
	//USART_Transmit_Newline();
	dataStruct->angle = angle;
	dataStruct->rawAngle = rawAngle;
}


