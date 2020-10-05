#include "ros/ros.h"
#include <sstream>
#include "std_msgs/String.h"
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"
#include "driver_HDC2450.h"



void Velocidade_motor_rpm(const geometry_msgs::Twist::ConstPtr& velocidade);

//Variáveis Globais
const float PI = 3.141592654;
float L = 0.42; // distância entre as rodas
float R = 0.055; // raio das rodas
float vd_rad;
float ve_rad;
int vd_rpm = 0; // velocidade da roda direita em rpm
int ve_rpm = 0; // velocidade da roda esquerda em rpm

std::string porta = "/dev/ttyS0";

Driver HULK(porta);

int main(int argc, char **argv)
{
	std_msgs::String velocidade;
	std_msgs::String dados_hulk;
	std::stringstream msg;
	
	ros::init(argc,argv,"hulk_node");
	ros::NodeHandle n;
	ros::NodeHandle n_private("~");

	ros::Subscriber sub = n.subscribe("velocidade_hulk",1000,Velocidade_motor_rpm);

	ros::Publisher pub = n_private.advertise<std_msgs::String>("leitura_velocidade",1000);
	ros::Publisher pub2 = n_private.advertise<std_msgs::String>("dados_bateria",1000);


	ros::Rate freq(20);

	while(ros::ok()){
	std::stringstream msg1,msg2;
	
	HULK.read_speed();

	msg1<<HULK.read_vd()<<","<<HULK.read_ve();
	
	velocidade.data = msg1.str();

	pub.publish(velocidade);
	
	HULK.read_current();
	HULK.read_temp();
	HULK.read_volt();
	msg2<<"A1:"<<HULK.read_current_d()<<" A2:"<<HULK.read_current_e()<<" T_MCU:"<<HULK.read_temp_MCU()<<" T_M1:"<<HULK.read_temp_motor1()<<" T_M2:"<<HULK.read_temp_motor2()<<" V_int:"<<HULK.read_volt_int()<<" V_bat:"<<HULK.read_volt_bat()<<" V_out:"<<HULK.read_volt_out();
	
	dados_hulk.data = msg2.str();
	
	pub2.publish(dados_hulk);
	
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

	
	
	


