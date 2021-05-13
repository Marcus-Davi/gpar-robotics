#include <iostream>
#include "ros/ros.h"
#include "mpu_6050.h"
#include "std_msgs/String.h"
#include <sstream>

MPU_6050 sensor;
std::stringstream ss;

void leitura_dados_sensor();

int main(int argc, char **argv){	

	ros::init(argc,argv,"mpu_node");

	ros::NodeHandle n("~");

	ros::Publisher pub = n.advertise<std_msgs::String>("mpu_data",1000);

	while(ros::ok()){
	std_msgs::String msg;
	
	leitura_dados_sensor();

	msg.data = ss.str();

	pub.publish(msg);
	
	}

return 0;
}

void leitura_dados_sensor(){
 
 ss.str("");

 sensor.read_data();
	
 ss<<"Gx = "<<sensor.read_gx()<<" /s "<<"Gy = "<<sensor.read_gy()<<" /s "<<"Gz = "<<sensor.read_gz()<<" /s "<<"Ax = "<<sensor.read_ax()<<" g "<<"Ay = "<<sensor.read_ay()<<" g "<<"Az = "<<sensor.read_az()<<" g ";



}

