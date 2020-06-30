#include <iostream>
#include <sstream>
#include "string.h"
#include "serial/serial.h"

class Driver
{
	private:
	public:
	void set_motor_speed(int motor, int speed);
	int read_encoder_rpm(int encoder);
	float get_temperature();
	int read_motor_amps(int motor);
	int read_volts();
	
};

void Driver::set_motor_speed(int motor, int speed){	
	std::stringstream comando;

	comando<<"!G "<<motor<<" "<<speed<<"\r";
		
}
int Driver::read_encoder_rpm(int encoder){
	int rpm;
	return 0;
	}

	

