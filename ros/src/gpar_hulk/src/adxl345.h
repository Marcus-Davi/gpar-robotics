/*
 * GY80.h
 *
 *  Created on: 22 de mai de 2019
 *      Author: marcus
 */

#ifndef adxl345_H_
#define adxl345_H_

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdint.h>


//#define I2C_GY_MODULE I2C1

/*Accelerometer Registers*/
#define ADX_ACC_WHOAMI 0x00
#define ADX_ACC_BWRATE 0X2C
#define ADX_ACC_POWERCTL 0X2D
#define ADX_ACC_INTENABLE 0X2E
#define ADX_ACC_INTSOURCE 0X30
#define ADX_ACC_DATAFORMAT 0X31
#define ADX_ACC_DATAX 0X32
#define ADX_ACC_DATAY 0X34
#define ADX_ACC_DATAZ 0X36




/*General Parameters*/
#define ADX_ACC_RESOLUTION 4e-3 //4mg / LSB


class adxl345 {
	public:
	//Endereço dos dispositivos
	enum GY_DEVICE_ADDR {
		ACC_ADDR = 0x53,
	};

	//Vetor
	struct Ivector3 {
		int16_t X;
		int16_t Y;
		int16_t Z;
	};

	//Recebe os dados e calibrações
	struct IMUData {
		Ivector3 Acc;
		Ivector3 AccOffset;
        
	}Imu;

	int ADX_ACC_ADD;

private:
		

	int I2CRead(uint8_t device, uint8_t reg);
	int I2CRead16(uint8_t device, uint8_t reg);
	int I2CWrite(uint8_t device, uint8_t reg, uint8_t data);

	bool WaitAccDataReady();
	bool DeviceConnected = false;


public:

	adxl345();
	bool Init();
	void ReadAcc();
	bool isDeviceConnected();

	friend class IMU;

};

#endif /* adxl345_H_ */