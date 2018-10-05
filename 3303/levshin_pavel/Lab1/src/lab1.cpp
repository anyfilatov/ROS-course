#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "Lab1");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	ros::Rate loop_rate(0.7);
	int f = 7;
	int sign = 1;
	float pi = 3.14;
	for (int t = 0; t <= 8*f; t++){

		geometry_msgs::Twist msg;
		msg.linear.x = M_PI/4;
		msg.angular.z = sign*M_PI/2;
		if (t % f != 0) sign = -sign;
		if (t % 4*f == 0 && t > 1) sign = -sign;

		ROS_INFO("[%4.2f] Move to position:\n1) msg.linear: x=%f y=%f z=%f\n2)msg.angular: x=%f y=%f z=%f\n",
				(float)t/(8*f)*100, msg.linear.x, msg.linear.y, msg.linear.z,
				msg.angular.x, msg.angular.y, msg.angular.z);
		pub.publish(msg);
		loop_rate.sleep();
	}
	ros::spinOnce();
	return 0;
}	