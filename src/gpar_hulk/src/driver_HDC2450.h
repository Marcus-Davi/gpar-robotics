#include <iostream>
#include <sstream>
#include "string.h"
#include "serial/serial.h"


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

	comando1<<"!G 1 "<<vd_rpm<<" \r";
	comando2<<"!G 2 "<<ve_rpm<<" \r";

	porta_serial->write(comando1.str());
	porta_serial->write(comando2.str());		
}

void Driver::serial_verify(std::string porta){
     serial::Serial serial(porta,115200,serial::Timeout::simpleTimeout(1000));

     if(serial.isOpen() == "true"){
	std::cout<<"Porta serial conectada com sucesso!"<<std::endl;
        porta_serial = &serial; 
}
    else
	exit(-1);
}
    
	

