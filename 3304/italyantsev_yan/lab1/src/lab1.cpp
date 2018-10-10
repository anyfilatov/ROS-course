#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lab1");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	ros::Rate loop_rate(10);
    
    geometry_msgs::Twist coord;
    coord.linear.x=1;    

	for(int i=0; i<410; i++)
	{
        if (i< 50)
        {
            coord.linear.x=0.6;
        }
        else if ((i>60 && i<107) || (i>160 && i<207) || (i>260 && i<307) || (i>360 && i<405))
        {
            coord.angular.z=1;
        }
        else if ((i>107 && i<160) || (i>207 && i<260) || (i>307 && i<360) || (i>407 && i<410))
        {
            coord.linear.x=0.6;
            coord.angular.z=0;
        }
	
		ROS_INFO("Move to position:\n"
			"1) coord.linear: x=%f\n"
			"2) coord.angular:  z=%f\n",
			coord.linear.x, coord.angular.z);
		pub.publish(coord);
		loop_rate.sleep();
	}
    
	ros::spinOnce();
	return 0;
}
