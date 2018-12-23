#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <stdlib.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lab1");

	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	ros::Rate loop_rate(10);
    
	srand(time(0));
   
	while (ros :: ok()) {
		geometry_msgs::Twist coord;		

		coord.linear.x = double(rand())/double(RAND_MAX);
		coord.angular.z = 2 * double(rand())/double(RAND_MAX) - 1;

		ROS_INFO("Move to position:\n"
			"1) coord.linear: x=%f\n"
			"2) coord.angular:  z=%f\n",
			coord.linear.x, coord.angular.z);
		pub.publish(coord);
		loop_rate.sleep();
	}

	return 0;
}
