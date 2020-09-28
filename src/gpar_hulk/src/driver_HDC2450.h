#include <iostream>
#include <sstream>
#include "string.h"
#include "serial/serial.h"
#include <unistd.h>


class Driver
{
	private:	
		serial::Serial *porta_serial;
	public:
		void serial_verify(std::string porta);
		void set_speed(int vd_rpm, int ve_rpm);
	
	
};

void Driver::set_speed(int vd_rpm, int ve_rpm){	
	
	std::stringstream comando1,comando2;

	// Verificar qual motor é de qual roda

	comando1<<"!G 1 "<<vd_rpm<<"\r";
	porta_serial->write(comando1.str());
	
	usleep(1000);

	comando2<<"!G 2 "<<vd_rpm<<"\r";
	porta_serial->write(comando2.str());	
}

void Driver::serial_verify(std::string porta){
    		
  	porta_serial = new serial::Serial(porta,9600,serial::Timeout::simpleTimeout(1000));

	if(porta_serial->isOpen())
		std::cout<<"Porta serial conectada!"<<std::endl;

}
    
	

