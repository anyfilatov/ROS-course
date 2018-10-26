#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include <cstdlib>
#include <ctime>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lab1");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
	ros::Rate rate(1);
	
	std::srand(unsigned(std::time(0)));
	ROS_INFO("START");
    	while (ros::ok())
    	{
	    	
		geometry_msgs::Twist msg;
		msg.linear.x = std::rand() % 10 + 1;
		msg.angular.z = std::rand() % 10 + 1;
		
		pub.publish(msg);
		rate.sleep();
	}

	return 0;
}

