#include "ros/ros.h"

#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

#include "termios.h"

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_SPACE 0x20


int kfd = 0;
char c; 
struct termios cooked,raw;


class Hulk_move
{
	public:
	Hulk_move();
	void keyloop();	

	private:
	ros::Publisher pub;
	double linear,angular;

};
	
int main (int argc, char **argv){	

	ros::init(argc,argv,"hulk_move");
	
	Hulk_move movimento;    

	movimento.keyloop();

	return 0;
}

void Hulk_move::keyloop(){
	tcgetattr(kfd, &cooked);
  	memcpy(&raw, &cooked, sizeof(struct termios));
  	raw.c_lflag &=~ (ICANON | ECHO);
  	// Setting a new line, then end of file                         
 	 raw.c_cc[VEOL] = 1;
 	 raw.c_cc[VEOF] = 2;
 	 tcsetattr(kfd, TCSANOW, &raw);

  	puts("Reading from keyboard");
  	puts("---------------------------");
  	puts("Use arrow keys to move Hulk.");
	while(ros::ok())
 {	
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }
    //ROS_INFO("value: 0x%02X\n", c);
	switch(c){
	case KEYCODE_RIGHT:
		ROS_INFO("Move to right");
		angular=-0.2;
		linear=0.0;
	break;
	case KEYCODE_LEFT:
		ROS_INFO("Move to left");
		angular=0.2;
		linear=0.0;
	break;
	case KEYCODE_UP:
		ROS_INFO("Move to up");
		linear=0.2;
		angular=0.0;
	break;
	case KEYCODE_DOWN:
		ROS_INFO("Move to down");
		linear=-0.2;
		angular=0.0;
	break;
	case KEYCODE_SPACE: 
		ROS_INFO("Stop");
		linear=0;	
		angular=0;
		
	break;
 
	}
	geometry_msgs::Twist velocidade;

	velocidade.linear.x= linear;
	velocidade.angular.z = angular;

	pub.publish(velocidade);
}}

Hulk_move::Hulk_move(){
	linear= 0.0;
	angular = 0.0;
	ros::NodeHandle n("~");	

	pub = n.advertise<geometry_msgs::Twist>("hulk_speed",1000);
}



