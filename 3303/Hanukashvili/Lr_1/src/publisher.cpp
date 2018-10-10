#include "ros/ros.h"
#include "geometry_msgs/Twist.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "publisher");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	ros::Rate loop_rate(1);
	//draw a heart	
		
	   for (int t=0; t<=16; t++){
		
		geometry_msgs::Twist pos;
		pos.linear.x = 1;
		if (t<5){
			pos.angular.z = 0.5;
		}
		else
		     if(t<8){
			pos.angular.z = 1;
			pos.linear.x = 0.9;
		}
		else
		     if(t==8)
			{pos.angular.z = -10;}
		else
		     if(t<12){
			pos.angular.z = 1;
			pos.linear.x = 0.9;
		}
		else
		     {
			pos.angular.z = 0.45;
			pos.linear.x = 0.9;
		}
		
		    
		
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
