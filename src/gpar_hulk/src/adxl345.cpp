/*
 * adxl345.cpp
 *
 *  Created on: 3 oct 2020
 *      Author: Luan Amaral
 */

#include "adxl345.h"
#include <stdio.h>
#include <typeinfo>

int adxl345::I2CRead(uint8_t device, uint8_t reg){
	// return wiringPiI2CRead(device);
    return wiringPiI2CReadReg8 (device, reg);
}

int adxl345::I2CRead16(uint8_t device, uint8_t reg){
	// return wiringPiI2CRead(device);
    return wiringPiI2CReadReg16	(device, reg);
}

int adxl345::I2CWrite(uint8_t device, uint8_t reg, uint8_t data){
	// return wiringPiI2CWrite(device, data );
    return wiringPiI2CWriteReg8 (device, reg, data);
}

bool adxl345::WaitAccDataReady(){
	uint8_t rx = 0;
	rx = I2CRead(ACC_ADDR, ADX_ACC_INTSOURCE);
		if( (rx&0b10000000) == 0b10000000)
			return true;
		return false;
}


adxl345::adxl345() {
	// TODO Auto-generated constructor stub
//	Init();

}

bool adxl345::Init(){
	uint8_t rx = 0;
	DeviceConnected = false;
	ADX_ACC_ADD = wiringPiI2CSetup (0x53);
		rx = I2CRead(ADX_ACC_ADD, ADX_ACC_WHOAMI);
		//if(rx == 0b11100101)
		if(rx - 229 == 0)
		{
			return false;
		}
		// All devices responded :)
		uint8_t tx;

		/*Accelerometer Configuration*/
		tx = 0b00001010; //NORMAL POWER, 100 Hz
		I2CWrite(ADX_ACC_ADD, ADX_ACC_BWRATE, tx);

		tx = 0b00001000; //NORMAL MEASUREMENT ON
		I2CWrite(ADX_ACC_ADD, ADX_ACC_POWERCTL, tx);

		tx = 0b00001001; //FULL RESOLUTION, +-4g , 4mg/LSB. 14-bit
		I2CWrite(ADX_ACC_ADD, ADX_ACC_DATAFORMAT, tx);

		DeviceConnected = true;

		return true;
}

/* IMPORTANTE --> Alinhamento dos eixos eh feito na hora da leitura! */
//BODY X = - GY_Y
//BODY Y = - GY_Z
//BODY Z = GY_X


void adxl345::ReadAcc(){
	while(WaitAccDataReady() == false);

    uint16_t x = I2CRead16(ADX_ACC_ADD, ADX_ACC_DATAX);
    uint16_t y = I2CRead16(ADX_ACC_ADD, ADX_ACC_DATAY);
    uint16_t z = I2CRead16(ADX_ACC_ADD, ADX_ACC_DATAZ);

	Imu.Acc.Z = z;
	Imu.Acc.X = x;
	Imu.Acc.Y = y;
}



bool adxl345::isDeviceConnected(){
	return DeviceConnected;
}
