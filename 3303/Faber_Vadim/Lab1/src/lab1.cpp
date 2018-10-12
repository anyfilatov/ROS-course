#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <math.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lab1");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

	ros::Rate rate(1);

	for (int t=0; t<8; t++){
		geometry_msgs::Twist msg;
		msg.linear.x = 2.5 * (t/2);
		msg.angular.z = (M_PI/2)*2;

		ROS_INFO_STREAM("\nlinear.x=" << msg.linear.x << "\n"
			<< "angular.z=" << msg.angular.z);
		pub.publish(msg);
		rate.sleep();
	}
	ros::spinOnce();
	return 0;
}

