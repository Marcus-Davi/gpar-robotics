#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "std_msgs/String.h"
#include "driver_HDC2450.h"
#include "serial/serial.h"
#include "string.h"


static const float d_hulk = 0;
static const float raio_hulk = 0;
static const float f = 0;
static const float T = 0;

static double x=0,y=0,theta=0; //coordenadas


void movimento(const geometry_msgs::Twist::ConstPtr& move);
void serial_config(std::string porta, int rate);

ros::Subscriber sub1;
serial::Serial* pserial;
std::stringstream msg;
std_msgs::String msg1;

int main(int argc, char **argv)
{
	
	ros::init(argc,argv,"hulk_node");
	ros::NodeHandle n;
	sub1 = n.subscribe("velocidade_hulk",1,movimento);

	serial_config("/dev/ttyACM1",9600);

	ros::spin();

return 0;
}

void movimento(const geometry_msgs::Twist::ConstPtr& move){
	//Por enquanto, vou deixar ele enviando a informação recebida para serial de forma bruta, sem trabalhar com os dados.
	msg.str("");
	msg<<"Velocidade Linear X: "<<move->linear.x<<"\nVelocidade Angular Z: "<<move->angular.z<<"\r";
	pserial->write(msg.str());
	msg1.data = msg.str();		
	ROS_INFO("%s",msg1.data.c_str());
}

void serial_config(std::string porta, int rate){
	std::string porta_serial = porta;
	int baud_rate = rate;
	
	serial::Serial serial(porta_serial,baud_rate,serial::Timeout::simpleTimeout(1000));
	
	if(serial.isOpen())
 	ROS_INFO("Porta Serial aberta!");
	 else 
 	ROS_INFO("Problema ao abrir a porta!");
	pserial = &serial;
}
	
	
	
	


