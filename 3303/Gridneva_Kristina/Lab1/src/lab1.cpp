// ROS Lab_1

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

void move (double speed, double distance);
void rotate (double angular_speed, double angle);
double degrees2radians(double angle_in_degrees);

const double PI = 3.14159265359;
const double speed = 2.0;
const double angular_speed = 40.0;
const double angle = 175;
ros::Publisher velocity_publisher;

int main (int argc, char ** argv)
{
	ros::init(argc, argv, "shape_for_turtlesim");
	ros::NodeHandle n;
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	ROS_INFO("\n\n\n******START************\n");
	
	double distance = 7.4;
	ros::Rate loop_rate(10);
	while(ros::ok()){
	
	move(speed, distance);
	rotate(degrees2radians(angular_speed), degrees2radians(angle));
	ros::spinOnce();
	loop_rate.sleep();
	}
		
	return 0;
}

void move (double speed, double distance){

	geometry_msgs::Twist vel_msg; //type of msg
	//set linear velocity 
	vel_msg.linear.x =speed;	
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	//set angular velocity 
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z = 1.8;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(10);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		loop_rate.sleep();
	}while(current_distance<distance);
}

void rotate (double angular_speed, double relative_angle){

	geometry_msgs::Twist vel_msg; //type of msg
	//set linear velocity 
	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =1.8;
	//set angular velocity 
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z =abs(angular_speed);

	double current_angle = 0.0;
	double t0 = ros::Time::now().toSec();
	ros::Rate loop_rate(10);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = angular_speed * (t1-t0);
		loop_rate.sleep();
	}while(current_angle<relative_angle);
}

double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}
