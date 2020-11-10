#include <stdio.h>
#include "mpu6050.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>
#include <stdint.h>
#include <ctime>
#include "ros/ros.h"

// #include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"

#define alpha 0.9


using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    int index = 0;

    // ros::Rate loop_rate(10);

    ros::Publisher pub = n.advertise<sensor_msgs::Imu>("/aceleracao", 1000);

    mpu6050 mpu;
    if (!(mpu.Init()))
    {
        printf("ERRO!\n");
        return -1;
    }

    sensor_msgs::Imu acc;
    acc.header.frame_id = "Acelerometro";


    while (ros::ok())
    {
        mpu.ReadAll();
        acc.header.seq = index;
        
        //acc.stamp = ros::Time::now();
        acc.linear_acceleration.x = mpu.Imu.Acc.X;
        acc.linear_acceleration.y = mpu.Imu.Acc.Y;
        acc.linear_acceleration.z = mpu.Imu.Acc.Z;  

        // https://medium.com/@kalpeshnpatil/raspberry-pi-interfacing-with-mpu6050-motion-sensor-c9608cd5f59c
        if(acc.linear_acceleration.x > 32768){
            acc.linear_acceleration.x = acc.linear_acceleration.x - 65536;
        }
        if(acc.linear_acceleration.y > 32768){
            acc.linear_acceleration.y = acc.linear_acceleration.y - 65536;
        }
        if(acc.linear_acceleration.z > 32768){
            acc.linear_acceleration.z = acc.linear_acceleration.z - 65536;
        }

        acc.linear_acceleration.x = 19.6*(float)acc.linear_acceleration.x/16384.0;
        acc.linear_acceleration.y = 19.6*(float)acc.linear_acceleration.y/16384.0;
        acc.linear_acceleration.z = 19.6*(float)acc.linear_acceleration.z/16384.0;

        acc.angular_velocity.x = (float)mpu.Imu.Gyr.X;
        acc.angular_velocity.y = (float)mpu.Imu.Gyr.Y;
        acc.angular_velocity.z = (float)mpu.Imu.Gyr.Z;

        if(acc.angular_velocity.x > 32768){
            acc.angular_velocity.x = acc.angular_velocity.x - 65536;
        }
        if(acc.angular_velocity.y > 32768){
            acc.angular_velocity.y = acc.angular_velocity.y - 65536;
        }
        if(acc.angular_velocity.z > 32768){
            acc.angular_velocity.z = acc.angular_velocity.z - 65536;
        }

        acc.angular_velocity.x = (float)acc.angular_velocity.x/65.5 ;
        acc.angular_velocity.y = (float)acc.angular_velocity.y/65.5 ;
        acc.angular_velocity.z = (float)acc.angular_velocity.z/65.5 ;

        
        ROS_INFO("x: %.4f\t y: %.4f\t z: %.4f\t x': %.4f\t y': %.4f\t z': %.4f\t", 
                                        acc.linear_acceleration.x,
                                        acc.linear_acceleration.y, 
                                        acc.linear_acceleration.z,
                                        acc.angular_velocity.x,
                                        acc.angular_velocity.y,
                                        acc.angular_velocity.z);
        
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
