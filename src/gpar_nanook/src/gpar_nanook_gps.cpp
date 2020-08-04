/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <sstream>
#include <serial/serial.h>
#include <sensor_msgs/NavSatFix.h>

sensor_msgs::NavSatFix ParseGPS(const std::string& msg);


   const unsigned char 	AlwaysLocateMode[]={"$PMTK225,8*23\r\n"};
   const unsigned char StandbyMode[] = {"$PMTK161,0*28\x0D\x0A"};
   const unsigned char SetHertz[] = {"$PMTK220,100*2F\r\n"};
   const unsigned char GGAMode[] = {"$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"};
   const unsigned char GLLMode[] = {"$PMTK314,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"};


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */

  ros::init(argc, argv, "mcuserial_node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle nh;

  //ros::NodeHandle n("~");

  std::string porta_serial;
  sensor_msgs::NavSatFix GpsData;


porta_serial = "/dev/ttyS0"; //Hardware UART


  serial::Serial mcu_serial(porta_serial,57600,serial::Timeout::simpleTimeout(1000));
if(mcu_serial.isOpen()){
 ROS_INFO("Porta Serial aberta!");
} else {
 ROS_INFO("Problema ao abrir a porta %s ! ela existe?",porta_serial.c_str());
return -1;
}



 ros::Publisher pub = nh.advertise< sensor_msgs::NavSatFix>("gps",100);
 mcu_serial.write(AlwaysLocateMode,sizeof(AlwaysLocateMode));
 ros::Duration(0.1).sleep();
 mcu_serial.write(GLLMode,sizeof(GLLMode));
 ros::Duration(0.1).sleep();
 mcu_serial.write(SetHertz,sizeof(SetHertz));


 std::string data;

 ros::Rate r(20);
	
	//Escrever algum código que verifique se o Nanook tá ok!
while(ros::ok()){

//	ROS_INFO("getting data");
	data = mcu_serial.readline(200,"\r"); //Le ate 100 bytes
//	ROS_INFO("got data!");
//	ROS_INFO("parsing");
GpsData = ParseGPS(data);
//ROS_INFO("parsed... publishing");

pub.publish(GpsData);
//ROS_INFO("published");








ros::spinOnce();
r.sleep();


}


  return 0;
}


sensor_msgs::NavSatFix ParseGPS(const std::string& msg){
	sensor_msgs::NavSatFix GPS;
//	ROS_INFO("msg = %s",msg.c_str());
	GPS.header.stamp = ros::Time::now();
	char* ptr;
	double lat,lat_d,lat_m;
	double lon,lon_d,lon_m;
//	ROS_INFO("getting GPGLL");
	ptr = strstr((char*)msg.c_str(),"GPGLL");
	if(!ptr){
		return GPS;
		ROS_INFO("GPGLL not found!");
	}
	ROS_INFO("Sentence : %s",ptr);
	ptr = strchr((char*)msg.c_str(),','); // look for comma

//	ROS_INFO("ptr OK %s.. atofing",ptr);
	lat = atof(++ptr);
//	ROS_INFO("atof ok",ptr);
		if(lat == 0){
			GPS.latitude = 0;
			GPS.longitude = 0;
			GPS.status.status = -1; //NO FIX!
		} else {
			ptr = strchr((char*)ptr,','); //procura 2a virgula ddmm.mmmm
			ptr = strchr((char*)++ptr,','); //procura 3a virgula dddmm.mmmm
			lon = atof(++ptr);
			lat_d = floorf(lat/100); //graus
			lat_m = (lat/100.0 - lat_d)*1.66666666666666; //isola os minutos, converte para décimos de graus (1 min = 1/60 grau)
//			//exemplo : min = 44.6567. 44.6567/60 = 0.744
			lon_d = floorf(lon/100); //graus
			lon_m = (lon/100.0 - lon_d)*1.6666666666666;//isola os minutos, converte para décimos de graus
			GPS.latitude = lat_d + lat_m;
			GPS.longitude = lon_d + lon_m;
			GPS.status.status = 0;
		}


//		ROS_INFO("4:: RETURNING GPS..");



	return GPS;

}


