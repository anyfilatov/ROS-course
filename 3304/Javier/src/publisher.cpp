#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
		ros:: init(argc, argv, "publisher");
		ros::NodeHandle n;
		ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
		
		ros::Rate loop_rate(1);
		double lengthOfPath = 0;
		double angularDirection = 0;
		double speed = 1;
		
		geometry_msgs::Twist pos;
		
		for(int t = 0; t < 20; t++) {
			
			pos.linear.x = speed;
			pos.angular.z = angularDirection;
			
			ROS_INFO("Move to position: \n"
				"1) pos.linear: x= %f y = %f z=%f\n"
				"2) pos.angular: x = %f y = %f z = %f\n" 
				"3) lengthOfPath = %f\n",
				pos.linear.x, pos.linear.y, pos.linear.z, pos.angular.x, pos.angular.z, 				lengthOfPath);
			
			if (t % 2 == 1) {
				angularDirection = 1.05;
				speed = 0;
			} else {
				angularDirection = 0;
				speed = 1;
			}	
			
			
			pub.publish(pos);
			lengthOfPath++;
			loop_rate.sleep();
			
		}
		ros::spinOnce();
		return 0;
}	