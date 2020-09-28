#include <iostream>
#include <sstream>
#include "string.h"
#include "serial/serial.h"
#include <unistd.h>
#include <stdlib.h>



class Driver
{
	private:	
		serial::Serial *porta_serial;
	public:
		void serial_verify(std::string porta);
		void set_speed(int vd_rpm, int ve_rpm);
		void read_speed();
	
	
};

void Driver::set_speed(int vd_rpm, int ve_rpm){	
	std::string msg;
	std::stringstream comando1,comando2;

	// Motor 1 roda direita e motor 2 roda esquerda, para esse programa!

	comando1<<"!G 1 "<<vd_rpm<<"\r";
	porta_serial->write(comando1.str());
		
	//precisamos ler duas vezes pois o driver envia o echo(mesma informação novamente) e além disso um +\r
	msg =  porta_serial->readline(100,"\r");
	msg = porta_serial->readline(100,"\r");
	

	comando2<<"!G 2 "<<ve_rpm<<"\r";
	porta_serial->write(comando2.str());	
	
	msg = porta_serial->readline(100,"\r");
	msg = porta_serial->readline(100,"\r");
}

void Driver::serial_verify(std::string porta){
    	
 	porta_serial = new serial::Serial(porta,11520,serial::Timeout::simpleTimeout(1000));

	if(porta_serial->isOpen())
		std::cout<<"Porta serial conectada!"<<std::endl;
		
}
    
void Driver::read_speed(){
	
	std::string resposta;
	
	porta_serial->write("?S\r");
	
	//Usamos dois pois uma das leituras serve para ler o echo, enquanto o outro a informação de fato
	resposta = porta_serial->readline(100,"\r");
	resposta = porta_serial->readline(100,"\r");
	
}
			
		
	
	
	



	

