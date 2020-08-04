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
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "geometry_msgs/Twist.h"

#include <termios.h>
#include <sstream>
#include <signal.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_SPACE 0x20


 
struct termios cooked,raw;
ros::Publisher chatter_pub;
int kfd = 0;
char c;     

void keyLoop();





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

  ros::init(argc, argv, "nanook_manual_node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */

  ros::NodeHandle nh;

   chatter_pub = nh.advertise<geometry_msgs::Twist>("nanook_move", 1000);

  boost::thread my_thread(keyLoop);

    ros::spin();

    my_thread.interrupt();
    my_thread.join();

  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();

  return 0;

}

void keyLoop()
{
  // get the console in raw mode 
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  geometry_msgs::Twist velocities;
  velocities.linear.x = 0;
  velocities.linear.y = 0;
  velocities.linear.z = 0;

  velocities.angular.x = 0;
  velocities.angular.y = 0;
  velocities.angular.z = 0;

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move nanook.");

while(ros::ok())
 {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }


    ROS_INFO("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_INFO("LEFT");
        velocities.linear.x = 0;
        velocities.angular.z = 0.2;
        break;
      case KEYCODE_R:
        ROS_INFO("RIGHT");
        velocities.linear.x = 0;
        velocities.angular.z = -0.2;
        break;
      case KEYCODE_U:
        ROS_INFO("UP");
        velocities.linear.x = 0.2;
        velocities.angular.z = 0;
        break;
      case KEYCODE_D:
        ROS_INFO("DOWN");
        velocities.linear.x = -0.2;
        velocities.angular.z = 0;
        break;

      case KEYCODE_SPACE:
        ROS_INFO("SPACE");
        velocities.linear.x = 0;
        velocities.angular.z = 0;
        break;
    }
   chatter_pub.publish(velocities);
 }

return;
    
}





