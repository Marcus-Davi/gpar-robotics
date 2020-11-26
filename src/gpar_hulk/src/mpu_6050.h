/*
COMENTAR
*/

#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <iostream>

#define Device_Address 0x68	//Endereço do MPU_6050

//Registradores que serão usados
#define PWR_MGMT_1   0x6B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define INT_ENABLE   0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47

typedef struct Sensor_Data{
	float ax = 0,ay = 0,az = 0;
	float gx = 0,gy = 0,gz = 0;
}Sensor_Data;


int fd = wiringPiI2CSetup(Device_Address);	
	
short read_data_mpu(int addr){
	short high_byte,low_byte,value;
	
	high_byte = wiringPiI2CReadReg8(fd, addr);
	low_byte = wiringPiI2CReadReg8(fd, addr+1);
	value = (high_byte << 8) | low_byte;

	return value;
}

class MPU_6050{
	private:
		Sensor_Data mpu;
	public:
		MPU_6050(){
			wiringPiI2CWriteReg8 (fd, SMPLRT_DIV, 0x00);	
			wiringPiI2CWriteReg8 (fd, PWR_MGMT_1, 0x01);	
			wiringPiI2CWriteReg8 (fd, CONFIG, 0);		
			wiringPiI2CWriteReg8 (fd, GYRO_CONFIG, 24);	
			wiringPiI2CWriteReg8 (fd, INT_ENABLE, 0x01);	
		}
			void read_data(); //Aqui já deixar os dados prontos para enviar para o usuário, fazendo as conversões.
			
			float read_ax();
			float read_ay();
			float read_az();
			float read_gx();
			float read_gy();
			float read_gz();				
};

void MPU_6050::read_data(){
	mpu.ax = read_data_mpu(ACCEL_XOUT_H)/16384.0;
	mpu.ay = read_data_mpu(ACCEL_YOUT_H)/16384.0;
	mpu.az = read_data_mpu(ACCEL_ZOUT_H)/16384.0;

	mpu.gx = read_data_mpu(GYRO_XOUT_H)/131.0;
	mpu.gy = read_data_mpu(GYRO_YOUT_H)/131.0;
	mpu.gz = read_data_mpu(GYRO_ZOUT_H)/131.0;	
}

float MPU_6050::read_ax(){
	return mpu.ax;
}

float MPU_6050::read_ay(){
	return mpu.ay;
}
float MPU_6050::read_az(){
	return mpu.az;
}

float MPU_6050::read_gx(){
	return mpu.gx;
}

float MPU_6050::read_gy(){
	return mpu.gy;
}

float MPU_6050::read_gz(){
	return mpu.gz;
}

