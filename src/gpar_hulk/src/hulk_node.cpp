#include "ros/ros.h"
#include <sstream>
#include "std_msgs/String.h"
#include "serial/serial.h"


void serial_config(std::string porta, int rate);

ros::Subscriber sub1;
serial::Serial* pserial;
std::stringstream msg;
std_msgs::String msg1;


int main(int argc, char **argv)
{
	ros::init(argc,argv,"hulk_node");
	ros::NodeHandle n;

	serial_config("/dev/ttyACM1",9600);

	ros::Rate freq(1);

	while(ros::ok()){

	ROS_INFO("Hello");

	freq.sleep();
	}

return 0;
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
	
	
	
	


