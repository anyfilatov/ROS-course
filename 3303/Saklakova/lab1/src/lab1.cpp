#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main (int argc, char **argv){
	ros::init(argc, argv, "lab1");
	ros::NodeHandle n;
	
	ros::Publisher lab1_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	ros::Rate loop_rate(1);

	for (int count = 1; count < 15; count++){
		geometry_msgs::Twist pos;
		pos.linear.x = 2*(M_PI/2)*count;
		pos.angular.z = 10;
		ROS_INFO("Move to pos:\n 1) linear: x= %f y= %f z= %f\n 2) angular: x= %f y= %f z= %f\n", pos.linear.x, pos.linear.y, pos.linear.z, pos.angular.x, pos.angular.y, pos.angular.z);
		lab1_pub.publish(pos);
		loop_rate.sleep();	
	}
	ros::spinOnce();
	return 0;
}
