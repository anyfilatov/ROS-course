#include "ros/ros.h"
#include <my_message/Message1.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <ctime>
#include <math.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "writer");

	ROS_INFO_STREAM("Writer is ready.\n");
	ROS_INFO_STREAM("Writer began the attack.\n");
	
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<my_message::Message1>("Name", 30);
	sleep(1);

	long p = 3;
	long q = 5;
	long m;
	m = p * q;
	long e = 7;


	srand( time( 0 ) );
	ros::Rate loop_rate(1);
	for(long i = 0;i < 20; i++){
		long x,y, o_x, o_y;
		x = 1 + rand() % 10;
		o_x = x;
		y = 1 + rand() % 10;
		o_y = y;
		x = powl(x, e);
		y = powl(y, e);
		x = x % m;
		y = y % m;
		my_message::Message1 message;
		message.x = x;
		message.y = y;
		pub.publish(message);

		ROS_INFO("the coordinates (%ld,%ld)",message.x ,message.y, o_x, o_y);
		loop_rate.sleep();
	}
	ros::spinOnce();
	return 0;
}