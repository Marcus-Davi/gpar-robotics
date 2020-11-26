/*
 * mpu6050.cpp
 *
 *  Created on: 30 oct 2020
 *      Author: Luan Amaral
 */

#include "mpu6050.h"
#include <stdio.h>
#include <typeinfo>

int mpu6050::I2CRead(uint8_t device, uint8_t reg){
    return wiringPiI2CReadReg8 (device, reg);
}

int mpu6050::I2CRead16(uint8_t device, uint8_t reg){
    return wiringPiI2CReadReg16	(device, reg);
}

int mpu6050::I2CWrite(uint8_t device, uint8_t reg, uint8_t data){
    return wiringPiI2CWriteReg8 (device, reg, data);
}

bool mpu6050::WaitDataReady(){
	uint8_t rx = 0;
	rx = I2CRead(MPU_ADD, MPU_INTSTATUS);
		if( (rx&0b00000001) == 0b00000001)
			return true;
		return false;
}


mpu6050::mpu6050() {
	// TODO Auto-generated constructor stub
//	Init();

}

bool mpu6050::Init(){
	uint8_t rx = 0;
    uint8_t zero = 0;
 	DeviceConnected = false;
	MPU_ADD = wiringPiI2CSetup(0x68);
    rx = I2CRead(MPU_ADD, MPU_WHOAMI);
    
    if(rx != 0x68 ) 
    {
        return false;
    }
    // All devices responded :)
    uint8_t tx;

    /*Accelerometer Configuration*/
    tx = 00; 
    I2CWrite(MPU_ADD, MPU_SAMPLERATE, tx);

    tx = 0b00000000; 
    I2CWrite(MPU_ADD, MPU_CONFIG, tx);

    tx = 0b00000000; //ACC 400Hz
    I2CWrite(MPU_ADD, MPU_PWRMGMT, tx);

    tx = 0b00001000; //FULL SCALE 500 DEGREES/SEC
    I2CWrite(MPU_ADD, MPU_GYR_CONFIG, tx);

    tx = 0b00001000; //FULL SCALE 4g
    I2CWrite(MPU_ADD, MPU_ACC_CONFIG, tx);

    DeviceConnected = true;

    return true;
}

/* IMPORTANTE --> Alinhamento dos eixos eh feito na hora da leitura! */
//BODY X = - GY_Y
//BODY Y = - GY_Z
//BODY Z = GY_X


void mpu6050::ReadAcc(){
	while(WaitDataReady() == false);

    uint8_t x_high = I2CRead(MPU_ADD, MPU_ACC_DATAX);
    uint8_t x_low  = I2CRead(MPU_ADD, MPU_ACC_DATAX+1);
    uint8_t y_high = I2CRead(MPU_ADD, MPU_ACC_DATAY);
    uint8_t y_low  = I2CRead(MPU_ADD, MPU_ACC_DATAY+1);
    uint8_t z_high = I2CRead(MPU_ADD, MPU_ACC_DATAZ);
    uint8_t z_low  = I2CRead(MPU_ADD, MPU_ACC_DATAZ+1);

    uint16_t x = (x_high << 8) | x_low;
    uint16_t y = (y_high << 8) | y_low;
    uint16_t z = (z_high << 8) | z_low;

	Imu.Acc.Z = z;
	Imu.Acc.X = x;
	Imu.Acc.Y = y;
}

void mpu6050::ReadGyr(){
	//while(WaitDataReady() == false);

    uint8_t Gx_high = I2CRead(MPU_ADD, MPU_ACC_DATAX);
    uint8_t Gx_low  = I2CRead(MPU_ADD, MPU_ACC_DATAX+1);
    uint8_t Gy_high = I2CRead(MPU_ADD, MPU_ACC_DATAY);
    uint8_t Gy_low  = I2CRead(MPU_ADD, MPU_ACC_DATAY+1);
    uint8_t Gz_high = I2CRead(MPU_ADD, MPU_ACC_DATAZ);
    uint8_t Gz_low  = I2CRead(MPU_ADD, MPU_ACC_DATAZ+1);


    uint16_t Gx = (Gx_high << 8) | Gx_low;
    uint16_t Gy = (Gy_high << 8) | Gy_low;
    uint16_t Gz = (Gz_high << 8) | Gz_low;
    
	Imu.Gyr.Z = Gz;
	Imu.Gyr.X = Gx;
	Imu.Gyr.Y = Gy;
}

void mpu6050::ReadTemp(){
	while(WaitDataReady() == false);
    Imu.Temp = I2CRead16(MPU_ADD, MPU_TEM_DATA);

}

void mpu6050::ReadAll(){
    // while(WaitDataReady() == false);
    /*
    //ACELEROMETRO
    uint16_t x = (I2CRead(MPU_ADD, MPU_ACC_DATAX) << 8) | I2CRead(MPU_ADD, MPU_ACC_DATAX+1);    
    uint16_t y = (I2CRead(MPU_ADD, MPU_ACC_DATAY) << 8) | I2CRead(MPU_ADD, MPU_ACC_DATAY+1);
    uint16_t z = (I2CRead(MPU_ADD, MPU_ACC_DATAZ) << 8) | I2CRead(MPU_ADD, MPU_ACC_DATAZ+1);

	Imu.Acc.Z = z;
	Imu.Acc.X = x;
	Imu.Acc.Y = y;

    //GIROSCÓPIO

    uint16_t Gx = (I2CRead(MPU_ADD, MPU_ACC_DATAX) << 8) | I2CRead(MPU_ADD, MPU_ACC_DATAX+1);
    uint16_t Gy = (I2CRead(MPU_ADD, MPU_ACC_DATAY) << 8) | I2CRead(MPU_ADD, MPU_ACC_DATAY+1);
    uint16_t Gz = (I2CRead(MPU_ADD, MPU_ACC_DATAZ) << 8) | I2CRead(MPU_ADD, MPU_ACC_DATAZ+1);

	Imu.Gyr.Z = Gz;
	Imu.Gyr.X = Gx;
	Imu.Gyr.Y = Gy;

    //TEMPERATURA
    
    uint16_t t = (I2CRead(MPU_ADD, MPU_TEM_DATA) << 8) | I2CRead(MPU_ADD, MPU_TEM_DATA+1);

    Imu.Temp = t;
    /*
    */

    uint16_t x = I2CRead16(MPU_ADD, MPU_ACC_DATAX);
    uint16_t y = I2CRead16(MPU_ADD, MPU_ACC_DATAY);
    uint16_t z = I2CRead16(MPU_ADD, MPU_ACC_DATAZ);

    x = (x << 8) | x&0x00FF;
    y = (y << 8) | y&0x00FF;
    z = (z << 8) | z&0x00FF;

	Imu.Acc.Z = z;
	Imu.Acc.X = x;
	Imu.Acc.Y = y;

    //GIROSCÓPIO

    uint16_t Gx = I2CRead16(MPU_ADD, MPU_GYR_DATAX);
    uint16_t Gy = I2CRead16(MPU_ADD, MPU_GYR_DATAY);
    uint16_t Gz = I2CRead16(MPU_ADD, MPU_GYR_DATAZ);

    Gx = (Gx << 8) | Gx&0x00FF;
    Gy = (Gy << 8) | Gy&0x00FF;
    Gz = (Gz << 8) | Gz&0x00FF;

	Imu.Gyr.Z = Gz;
	Imu.Gyr.X = Gx;
	Imu.Gyr.Y = Gy;

    //TEMPERATURA
    
    uint16_t t = I2CRead16(MPU_ADD, MPU_TEM_DATA);
    Imu.Temp = (t << 8) | t&0x00FF;
    Imu.Temp = Imu.Temp/340*   +36.53;
    
}



bool mpu6050::isDeviceConnected(){
	return DeviceConnected;
}
