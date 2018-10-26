#include <ros/ros.h>
#include <my_message/Message1.h>
#include <ctime>
#include <math.h>


void reader(const my_message::Message1 & message)
{
	long p = 3;
	long q = 5;
	long fi = (p - 1) * (q - 1);
	long e = 7;
	long m;
	m = p * q;
	long d = 7;
	long x,y;
	x = powl(message.x, d);
	x = x % m;
	y = powl(message.y, d);
	y = y % m;
	ROS_INFO("Secret code: (%ld;%ld;%ld).\n", x, y);
}

int main(int argc, char **argv){
	ros::init(argc,argv,"reader");
	ROS_INFO("Reader is ready to defend\n");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("Name",30,reader);
	ros::spin();
	return 0;
}