#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

void printStep(geometry_msgs::Twist pos){
	ROS_INFO("Move to positiom:\n"
		"1) pos.linear: x=%f y=%f z=%f\n"
		"2) pos.angular: x=%f y=%f z=%f\n",
		pos.linear.x,pos.linear.y,pos.linear.z,
		pos.angular.x,pos.angular.y,pos.angular.z);
}

void step(int x, int z, ros::Publisher& pub, ros::Rate& loop_rate)
{
	geometry_msgs::Twist pos;
	pos.linear.x = x;
 	pos.angular.z = z;
	printStep(pos);
	pub.publish(pos);
	loop_rate.sleep();
}

int main(int args,  char **argv)
{
		ros::init(args, argv, "publisher");
		ros::NodeHandle n;
		ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
		ros::Rate loop_rate(1);
		step(0, 2 * M_PI, pub, loop_rate);		
		step(2, 0, pub, loop_rate);
		step(0, -M_PI/2, pub, loop_rate);
		step(1, 0, pub, loop_rate);
		step(0, M_PI/2, pub, loop_rate);
		step(0, M_PI, pub, loop_rate);
		step(2, 0, pub, loop_rate);
		step(0, M_PI/2, pub, loop_rate);
		step(2, 0, pub, loop_rate);
		step(0, M_PI/2, pub, loop_rate);
		step(1, 0, pub, loop_rate);
		step(0, M_PI/2, pub, loop_rate);
		step(2, 0, pub, loop_rate);
		step(0, M_PI/2, pub, loop_rate);
		step(1, 0, pub, loop_rate);

		ros::spinOnce();
		return 0;
}
