#include <iostream>
#include <sstream>
#include "string.h"
#include "serial/serial.h"
#include <unistd.h>
#include <stdlib.h>
#include <cstdio>
#include "sensor_msgs/BatteryState.h"

struct Leitura_String{

    //Vão armazenar os dados das leituras
    int ve_rpm;
    int vd_rpm;
    
    float current_d;
    float current_e;
    
    int temp_MCU;
    int temp_motor1;
    int temp_motor2;
    
    float volt_internal;
    float volt_battery;
    float volt_output;

};

class Driver
{
	private:
                Leitura_String dados;	
		serial::Serial *porta_serial;
	public:		
		Driver();		
		
		void serial_open(std::string porta);

		void set_speed(int vd_rpm, int ve_rpm);
		void read_speed();
		int read_ve();
		int read_vd();	
		
		void read_current();
		float read_current_d();
		float read_current_e();
		
		
		void read_temp();
		int read_temp_MCU();
		int read_temp_motor1();
		int read_temp_motor2();
		
		void read_volt();
		float read_volt_int();
		float read_volt_bat();
		float read_volt_out();

		void read();
		
		~Driver();
	
};
	Driver::Driver(){
	std::cout<<"---HULK PRINCIPAL NODE--"<<std::endl;
	}
	
void Driver::serial_open(std::string porta){
     	porta_serial = new serial::Serial(porta,115200,serial::Timeout::simpleTimeout(1000));

	if(porta_serial->isOpen())
		std::cout<<"Porta serial conectada!"<<std::endl;
     
     }

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

void Driver::read_speed(){
	
	std::string resposta;
	
	porta_serial->write("?S\r");
	
	//Usamos dois pois uma das leituras serve para ler o echo, enquanto o outro a informação de fato
	resposta = porta_serial->readline(100,"\r");
	resposta = porta_serial->readline(100,"\r");
	
	sscanf(resposta.c_str(),"S=%d:%d\r",&dados.vd_rpm,&dados.ve_rpm);
}

int Driver::read_ve(){

	return dados.ve_rpm;
}			

int Driver::read_vd(){

        return dados.vd_rpm;
}	
void Driver::read_current(){
	std::string resposta;
	
	porta_serial->write("?A\r");
	
	//Usamos dois pois uma das leituras serve para ler o echo, enquanto o outro a informação de fato
	resposta = porta_serial->readline(100,"\r");
	resposta = porta_serial->readline(100,"\r");
	
	sscanf(resposta.c_str(),"A	=%f:%f\r",&dados.current_d,&dados.current_e);
	
}

float Driver::read_current_d(){
	return dados.current_d;
}

float Driver::read_current_e(){
	return dados.current_e;
}

void Driver::read_temp(){
	std::string resposta;
	
	porta_serial->write("?T\r");
	
	//Usamos dois pois uma das leituras serve para ler o echo, enquanto o outro a informação de fato
	resposta = porta_serial->readline(100,"\r");
	resposta = porta_serial->readline(100,"\r");
	
	sscanf(resposta.c_str(),"T=%d:%d:%d\r",&dados.temp_MCU,&dados.temp_motor1,&dados.temp_motor2);
}

int Driver::read_temp_MCU(){
	return dados.temp_MCU;
}

int Driver::read_temp_motor1(){
	return dados.temp_motor1;
}

int Driver::read_temp_motor2(){
	return dados.temp_motor2;
}	

void Driver::read_volt(){
	std::string resposta;
	
	porta_serial->write("?V\r");
	
	//Usamos dois pois uma das leituras serve para ler o echo, enquanto o outro a informação de fato
	resposta = porta_serial->readline(100,"\r");
	resposta = porta_serial->readline(100,"\r");
	
	sscanf(resposta.c_str(),"V=%f:%f:%f\r",&dados.volt_internal,&dados.volt_battery,&dados.volt_output);
}

float Driver::read_volt_int(){
	return dados.volt_internal/10;
}

float Driver::read_volt_bat(){
	return dados.volt_battery/10;
}

float Driver::read_volt_out(){
	return dados.volt_output/1000;
}
	

void Driver::read(){
	read_speed();
	read_current();
	read_temp();
	read_volt();
}

	Driver::~Driver(){
		std::cout<<"\nFinalizando Programa"<<std::endl;
		delete porta_serial;
		}



	

