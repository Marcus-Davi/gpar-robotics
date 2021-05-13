#include <stdio.h>
#include "adxl345.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>
#include <stdint.h>
#include <ctime>
#include "ros/ros.h"

// #include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"

using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    int index = 0;

    // ros::Rate loop_rate(10);

    ros::Publisher pub = n.advertise<sensor_msgs::Imu>("/aceleracao", 1000);

    adxl345 adxl;
    if (!(adxl.Init()))
    {
        printf("ERRO!\n");
        return -1;
    }


    while (ros::ok())
    {
        adxl.ReadAcc();

        sensor_msgs::Imu acc;

        acc.header.seq = index;
        acc.header.frame_id = "Acelerometro";
        //acc.stamp = ros::Time::now();

        acc.linear_acceleration.x = (float)adxl.Imu.Acc.X;
        acc.linear_acceleration.y = (float)adxl.Imu.Acc.Y;
        acc.linear_acceleration.z = (float)adxl.Imu.Acc.Z;

        ROS_INFO("x: %f, y: %f, z: %f", acc.linear_acceleration.x,
                                        acc.linear_acceleration.y, 
                                        acc.linear_acceleration.z);
        pub.publish(acc);

        index++;

        //printf("x: %d, y: %d, z:%d\n", adxl.Imu.Acc.X, adxl.Imu.Acc.Y, adxl.Imu.Acc.Z);
        //printf("x: %f, y: %f, z: %f\n", acc.linear.x, acc.linear.y, acc.linear.z);
        // loop_rate.sleep();
    }
    return 0;
}

/*
int main(int argc, char **argv)
{
    uint8_t a=10;
    cout <<"Hello World\n";
}
*/
