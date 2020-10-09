#include "ros/ros.h"
#include <sstream>
#include "std_msgs/String.h"
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"
#include "driver_HDC2450.h"
#include "sensor_msgs/BatteryState.h"

void Velocidade_motor_rpm(const geometry_msgs::Twist::ConstPtr& velocidade);
void Dados_hulk();

//Variáveis Globais
const float PI = 3.141592654;
float L = 0.42; // distância entre as rodas
float R = 0.055; // raio das rodas
float vd_rad;
float ve_rad;
int vd_rpm = 0; // velocidade da roda direita em rpm
int ve_rpm = 0; // velocidade da roda esquerda em rpm

std::string porta = "/dev/ttyS0";

Driver HULK;

sensor_msgs::BatteryState hulk_dados;

int main(int argc, char **argv)
{
	std_msgs::String velocidade;
	
	ros::init(argc,argv,"hulk_node");
	ros::NodeHandle n;
	ros::NodeHandle n_private("~");

	ros::Subscriber sub = n.subscribe("/hulk_move/hulk_speed",1000,Velocidade_motor_rpm);

	ros::Publisher pub = n_private.advertise<std_msgs::String>("hulk_read_speed",1000);
	ros::Publisher pub2 = n_private.advertise<sensor_msgs::BatteryState>("hulk_battery_info",1000);

	n_private.getParam("porta_serial",porta);	
	
	HULK.serial_open(porta);
	
	ros::Rate freq(20);
	while(ros::ok()){

	std::stringstream msg;

	HULK.read();

	msg<<HULK.read_vd()<<","<<HULK.read_ve();
	velocidade.data = msg.str();
	
	Dados_hulk();

	pub.publish(velocidade);
	pub2.publish(hulk_dados);	

	
	ros::spinOnce();
	
	freq.sleep();
	}

return 0;
}

//Realizando o cáculo das velocidades de cada motor em rpm
void Velocidade_motor_rpm(const geometry_msgs::Twist::ConstPtr& velocidade){
	float v = velocidade->linear.x;
	float w = velocidade->angular.z;
		
	vd_rad = (v+w*L);
        ve_rad = (v-w*L);

	vd_rpm = (vd_rad*60/(2*PI*R));
	ve_rpm = (ve_rad*60/(2*PI*R));
	
	//std::cout<<"Velocidade roda direita = "<<vd_rpm<<"\nVelocidade roda esquerda = "<<ve_rpm<<std::endl;
	HULK.set_speed(vd_rpm,ve_rpm);
}

void Dados_hulk(){
	
	hulk_dados.voltage = HULK.read_volt_bat();
	hulk_dados.current = (HULK.read_current_d()*1000+HULK.read_current_e()*10);
	hulk_dados.percentage = HULK.read_volt_bat()/24;
	
}
	
	
	


