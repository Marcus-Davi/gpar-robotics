/*
 * mpu6050.h
 *
 *  Created on: 30 oct 2020
 *      Author: Luan Amaral
 */

#ifndef mpu6050_H_
#define mpu6050_H_

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <stdint.h>


//#define I2C_GY_MODULE I2C1
// MUDAR OS VALORES DOS REGISTRADORES
#define MPU_SAMPLERATE 0X19 
#define MPU_WHOAMI 0X75 //DEFAULT VALUE 0X68
#define MPU_PWRMGMT 0x6B
#define MPU_CONFIG 0X1A
#define MPU_INTSTATUS 0X3A
#define MPU_ADRESS 0X68

/*Accelerometer Registers*/
#define MPU_ACC_CONFIG 0X1C
#define MPU_ACC_DATAX 0X3B
#define MPU_ACC_DATAY 0X3D
#define MPU_ACC_DATAZ 0X3F

/*Gyroscope Registers*/
#define MPU_GYR_CONFIG 0X1B
#define MPU_GYR_DATAX 0X43
#define MPU_GYR_DATAY 0X45
#define MPU_GYR_DATAZ 0X47

/*Temperature Registers*/
#define MPU_TEM_DATA 0x41




/*General Parameters*/
#define ADX_ACC_RESOLUTION 4e-3 //4mg / LSB


class mpu6050 {
	public:
	//Endereço dos dispositivos
	enum MPU_DEVICE_ADDR {
		MPU_ADDR = 0x68,
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
        Ivector3 Gyr;

        float Temp;

		Ivector3 AccOffset;
        Ivector3 GyrOffset;
        
	}Imu;

	int MPU_ADD;

private:
		

	int I2CRead(uint8_t device, uint8_t reg);
	int I2CRead16(uint8_t device, uint8_t reg);
	int I2CWrite(uint8_t device, uint8_t reg, uint8_t data);

	bool WaitDataReady();
	bool DeviceConnected = false;


public:

	mpu6050();
	bool Init();
	void ReadAcc();
    void ReadGyr();
    void ReadTemp();
    void ReadAll();
	bool isDeviceConnected();

	friend class IMU;

};

#endif /* mpu6050_H_ */