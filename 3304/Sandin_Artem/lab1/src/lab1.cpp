#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lab1");
	ros::NodeHandle node;
	ros::Publisher pub = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 500);
	ros::Rate rate(1);

	for(int i = 0; i < 35; i++)
	{
		geometry_msgs::Twist coord;
		if (i < 12) coord.linear.x = 2;
		else coord.linear.x = -2;
		if (i > 31) coord.angular.z = -std::abs(2 * cos(i / 10));	
		else if (i > 20) coord.angular.z = - 1,7 + std::abs(2 * cos(i / 4));
		else coord.angular.z = std::abs(2 * cos(i / 4));
		pub.publish(coord);
		rate.sleep();
	}

	ros::spinOnce();
	return 0;
}
