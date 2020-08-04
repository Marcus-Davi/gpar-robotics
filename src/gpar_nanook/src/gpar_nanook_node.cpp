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
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
//#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

serial::Serial* mcu_serial_ptr;
static const float D_NANOOK = 0.405; //Diferente da prática ?
static const float R_NANOOK = 0.18/2.0; //Diferente da prática ?
static double x=0,y=0,theta=0;
static const float Freq = 20.0;
static const float Ts = 1/Freq;
static const int OffMagx = -79;
static const int OffMagy = 2;

void nanook_odom_reset(const std_msgs::String::ConstPtr& msg){
  if (msg->data == "reset"){
    x = 0;
    y = 0;
    theta = 0;
  }
  ROS_INFO("nanook coordinates reset");
}



void nanookMove(const geometry_msgs::Twist::ConstPtr& msg){

//ROS_INFO("detected msg!");
float v = msg->linear.x;
float w = msg->angular.z;

float vd_rad = (v+w*D_NANOOK);
float ve_rad = (v-w*D_NANOOK);
int vd = (vd_rad*60/(2*M_PI*R_NANOOK));
int ve = (ve_rad*60/(2*M_PI*R_NANOOK));

std::string msg_k64f = "M! " + std::to_string(vd) + "," + std::to_string(ve) + "\r";
//ROS_INFO("print = %s",msg_k64f.c_str());
mcu_serial_ptr->write(msg_k64f);
}

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
  ros::NodeHandle n("~");
  std::string porta_serial = "/dev/ttyACM0";
  std::string map_frame = "map";
  std::string odom_frame = "odom";
  bool publish_odom = false;


  if (n.getParam("serial_port",porta_serial))
	ROS_INFO("parametro 'serial_port' lido com sucesso : %s",porta_serial.c_str());
   else {
	ROS_INFO("parametro 'serial_port' n existe! setado : %s",porta_serial.c_str());
	}

  if (n.getParam("odom_frame",odom_frame)){
ROS_INFO("parametro 'odom_frame' lido com sucesso : %s",odom_frame.c_str());
  }  else {
ROS_INFO("parametro 'odom_frame' n existe! setado : %s",odom_frame.c_str());

  }

  if (n.getParam("map_frame",map_frame)){
ROS_INFO("parametro 'map_frame' lido com sucesso : %s",map_frame.c_str());
  }  else {
ROS_INFO("parametro 'map_frame' n existe! setado : %s",map_frame.c_str());
  }

   n.getParam("publish_odom",publish_odom);


  if (publish_odom){
ROS_INFO("odom tf will be published");
  }  else {
ROS_INFO("odom tf will not be published");
  }




  serial::Serial mcu_serial(porta_serial,115200,serial::Timeout::simpleTimeout(1000));
if(mcu_serial.isOpen()){
 ROS_INFO("Porta Serial aberta!");
} else {
 ROS_INFO("Problema ao abrir a porta %s ! ela existe?",porta_serial.c_str());
return -1;
}

mcu_serial_ptr = &mcu_serial;


 ros::Subscriber sub = nh.subscribe("nanook_move", 100, nanookMove);
 ros::Subscriber sub_rst = nh.subscribe("nanook_odom_reset", 100, nanook_odom_reset);
 ros::Publisher pub = nh.advertise<std_msgs::String>("sensors",100);

 std::string sensor_query = "S?\r";

 std_msgs::String leitura;

 ros::Rate r(Freq); //Pode ser melhor ?

 static tf2_ros::TransformBroadcaster br; //tf Broadcaster
 geometry_msgs::TransformStamped transformStamped;
 tf2::Quaternion Q_tf;
 geometry_msgs::Quaternion Q_msg;


 transformStamped.header.frame_id = "map";
 transformStamped.child_frame_id = "odom";

float vd_rpm = 0,ve_rpm = 0;
float vd = 0,ve = 0;
float v = 0,w = 0;
	//Escrever algum código que verifique se o Nanook tá ok!
while(ros::ok()){

mcu_serial_ptr->write(sensor_query);
leitura.data = mcu_serial_ptr->readline(100,"\r"); //Le ate 100 bytes
pub.publish(leitura);
ros::spinOnce();
// Odometry Computation
sscanf(leitura.data.c_str(),"%*d %*d %*d %*d %*d %*d %*d %*d %*d %f %f %*f %*f\r",&vd_rpm,&ve_rpm);
vd = vd_rpm *2*M_PI/60;
ve = ve_rpm *2*M_PI/60;

v = (vd+ve)*R_NANOOK*0.5; // rad/s -> m/s
w = (vd-ve)*R_NANOOK*0.5/D_NANOOK;
//ROS_INFO("v = %f, w = %f",v,w);
x = x + v*Ts*cos(theta);
y = y + v*Ts*sin(theta);
theta = theta + w*Ts;

//ROS_INFO("x = %f y = %f t = %f",x,y,theta);

 transformStamped.header.stamp = ros::Time::now();
 transformStamped.transform.translation.x = x;
 transformStamped.transform.translation.y = y;
 transformStamped.transform.translation.z = 0;
 Q_tf.setRPY(0,0,theta);
 Q_tf.normalize();
 Q_msg = tf2::toMsg(Q_tf);
 transformStamped.transform.rotation = Q_msg;
 if(publish_odom)
br.sendTransform(transformStamped);

r.sleep();


}

  return 0;
}
