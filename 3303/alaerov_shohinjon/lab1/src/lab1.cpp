#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
using namespace ros;
int main(int argc, char **argv)
{
	init(argc, argv, "publisher");
	NodeHandle n;
	Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	sleep(1);
	Rate loop_rate(10);
	for(int t=0; true; t++)
	{
		geometry_msgs::Twist pos;
		pos.linear.x=1;
		if((t%10>5))
			pos.angular.z=-1;
		else
			pos.angular.z=1;	
		ROS_INFO("Move to position:\n"
			"1) pos.linear: x=%f y=%f z=%f\n"
			"2) pos.angular: x=%f y=%f z=%f\n",
			pos.linear.x, pos.linear.y, pos.linear.z, pos.angular.x, pos.angular.y, pos.angular.z);
		pub.publish(pos);
		loop_rate.sleep();
	}
	return 0;
}
