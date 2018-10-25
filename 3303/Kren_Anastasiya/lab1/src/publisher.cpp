#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int args,  char **argv)
{
		ros::init(args, argv, "publisher");
		ros::NodeHandle n;
		ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);


		ros::Rate loop_rate(1);
		for (int t = 1; true; t++){
				
				geometry_msgs::Twist pos;
				
				if(t % 2 != 0){
				 pos.linear.x = 2;
				}
				if(t % 2 == 0){
				 pos.angular.z = M_PI/3;
				}

				ROS_INFO("Move to positiom:\n"
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
