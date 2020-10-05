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

std::string porta = "/dev/ttyACM0";

Driver HULK;

int main(int argc, char **argv)
{
	std_msgs::String velocidade;
	std::stringstream msg;
	
	ros::init(argc,argv,"hulk_node");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("velocidade_hulk",1000,Velocidade_motor_rpm);

	ros::Publisher pub = n.advertise<std_msgs::String>("leitura_velocidade_hulk",1000);

	HULK.serial_verify(porta);

	ros::Rate freq(20);

	while(ros::ok()){
	std::stringstream msg;

	HULK.read_speed();

	msg<<HULK.read_vd()<<","<<HULK.read_ve();
	
	velocidade.data = msg.str();

	pub.publish(velocidade);
	
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

	
	
	


