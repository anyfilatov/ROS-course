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
    for (int i = 0; i < 20; i++)
    {
		geometry_msgs::Twist msg;
		msg.linear.x = std::rand() / RAND_MAX > 0.5 ? 1 : -1;
		msg.angular.z = std::rand() / RAND_MAX > 0.5 ? 1 : -1;
		ROS_INFO("Move to position:\n -linear: x=%f\ty=%f\tz=%f\n -angular: x=%f\ty=%f\tz=%f\n",
            msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z);
		pub.publish(msg);
		rate.sleep();
	}
	ros::spinOnce();
	return 0;
}
