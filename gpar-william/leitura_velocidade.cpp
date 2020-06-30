#include "ros/ros.h"

#include "geometry_msgs/Twist.h"

void chatterCallback(const geometry_msgs::Twist::ConstPtr& velocidade) 

{ 
printf("\nVelocidade Linear: %.2f\nVelocidade Angular: %.2f\n----------", velocidade->linear.x,velocidade->angular.z);

} 

int main(int argc,char **argv) 

{ 
 
ros::init(argc, argv,"leitura_velocidade"); 

ros::NodeHandle n; 

ros::Subscriber sub = n.subscribe("velocidade_hulk",1,chatterCallback); 

ros::spin(); 

return 0; 

} 
