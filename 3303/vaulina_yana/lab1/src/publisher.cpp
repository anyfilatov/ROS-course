#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <math.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "publisher");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

	ros::Rate loop_rate(1);

	//double

	for (int t=0; t<10; t++){
		geometry_msgs::Twist pos;
		pos.linear.x = 5.5 * (t%2);
		pos.angular.z = (M_PI - M_PI/5) * ((t+1)%2);

		ROS_INFO("Move to position:\n"
			 "1) pos.linear: x=%f y=%f z=%f\n"
			 "2) pos.angular: x=%f y=%f z=%f\n",
			  pos.linear.x,pos.linear.y,pos.linear.z,
			  pos.angular.x,pos.angular.y,pos.angular.z);
		pub.publish(pos);
		loop_rate.sleep();
	}
	ros::spinOnce();
	return 0;
}
