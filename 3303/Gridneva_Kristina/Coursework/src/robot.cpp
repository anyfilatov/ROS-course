#include "ros/ros.h"
#include <ros/console.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelStates.h"
#include <fstream>
#include "string.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
 
using namespace std;
 
double PI = 3.1415926535897;

struct robot
{
	float p_x;	//position
	float p_y;
	float p_z;
	double roll;	//orientation
	double pitch;
	double yaw;
};
struct robot smart;
struct robot stupid;

void setPosition (struct robot *name, float p_x, float p_y, float p_z, double roll, double pitch, double yaw)
{
	name->p_x = p_x;
	name->p_y = p_y;
	name->p_z = p_z;
	name->roll = roll;
	name->pitch = pitch;
	name->yaw = yaw;
	ROS_INFO("robot: %.2f, %.2f, %.2f; rotation: %1.2f, %1.2f, %1.2f", name->p_x, name->p_y, name->p_z, name->roll, name->pitch, name->yaw); 
}


bool transformPoint(const tf::TransformListener &listener)  //listener tf smart_burger
{
	tf::StampedTransform transform;
	bool success = false;
	//do
	//{
		try{
		//listener.lookupTransform("world_smart", "smart_burger", ros::Time(0), transform);
		listener.lookupTransform("world_smart", "world_stupid", ros::Time(0), transform);
		ROS_INFO("smart position: %.2f, %.2f, %.2f; rotation: %.2f, %.2f, %.2f, %.2f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
		tf::Quaternion q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		setPosition(&smart, floor(transform.getOrigin().x()*100)/100, floor(transform.getOrigin().y()*100)/100, floor(transform.getOrigin().z()*100)/100, roll, pitch, yaw);
		success = true;
		}
		catch(tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();	
		}
		sleep(1);
	//}
	//while(!success);
	return success;
}
tf::StampedTransform transformPoint2(const tf::TransformListener &listener)  //listener tf own
{
	tf::StampedTransform transform;
	bool success = false;
	do
	{
		try{
		listener.lookupTransform("stupid_burger/odom", "stupid_burger/base_footprint", ros::Time(0), transform);
		ROS_INFO("my position: %.2f, %.2f, %.2f; rotation: %.2f, %.2f, %.2f, %.2f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
		tf::Quaternion q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		setPosition(&stupid, floor(transform.getOrigin().x()*100)/100, floor(transform.getOrigin().y()*100)/100, floor(transform.getOrigin().z()*100)/100, roll, pitch, yaw);
		success = true;
		}
		catch(tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();	
		}
		sleep(1);
	}
	while(!success);
	return transform;
}

void broadcaster ()
{
	tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::TransformListener listener;
	transform = transformPoint2(listener);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"world_stupid", "stupid_burger"));
	ROS_INFO("--------broadcaster-------");
}


void move (ros::Publisher cmd_vel_topic)
{
	geometry_msgs::Twist vel_msg;
	float cur_x = round(stupid.p_x);
	float cur_y = round(stupid.p_y);
	if((cur_x != 0) || (cur_y != 0))
	{
		vel_msg.linear.x = 0.05;
		
	}
	else {vel_msg.linear.x = 0;}
	
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;
	cmd_vel_topic.publish(vel_msg);
	ROS_INFO("cur_x: %.2f", cur_x);
	sleep(1);
	tf::TransformListener listener;
	transformPoint2(listener);
	sleep(1);
	
}

void rotate(float angle, ros::Publisher cmd_vel_topic, bool clockwise)
{
	float speed = 10.0;
	//Converting from angles to radians
	float angular_speed = speed*2*PI/360;
	float relative_angle = angle*2*PI/360;
	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x=0;
        vel_msg.linear.y=0;
        vel_msg.linear.z=0;
        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
	if (clockwise) {vel_msg.angular.z = abs(angular_speed)*(-1);}
	else {vel_msg.angular.z = abs(angular_speed);}
	
	//Setting the current time for distance calculus
	float t0 = ros::Time::now().toSec();
	float current_angle = 0;
	if (relative_angle != 0) {
		while (current_angle < relative_angle)
		{
			cmd_vel_topic.publish(vel_msg);
			float t1 = ros::Time::now().toSec();
			ROS_INFO("My t1: %.2f", t1);
			ROS_INFO("My t0: %.2f", t0);
			current_angle = angular_speed *(t1 - t0);
			ROS_INFO("My current_angle: %.2f", current_angle);
			ROS_INFO("My relative_angle: %.2f", relative_angle);
			sleep(1);
		}
	}
	vel_msg.angular.z = 0;
	cmd_vel_topic.publish(vel_msg);
	tf::TransformListener listener;
	tf::StampedTransform transform;
	transformPoint2(listener);
	//ros::spinOnce();
}

void update_angle(ros::Publisher cmd_vel_topic)
{
	if ((smart.yaw >0) && (stupid.yaw > 0))
	{	
		float dif = smart.yaw - stupid.yaw;
		if (dif > 0) {rotate(dif, cmd_vel_topic, false);}
		else {rotate(dif, cmd_vel_topic, true);}
	}
	if ((smart.yaw < 0) && (stupid.yaw < 0))
	{	
		float dif = abs(smart.yaw - stupid.yaw);
		if (dif > 0) {rotate(dif, cmd_vel_topic, true);}
		else {rotate(dif, cmd_vel_topic, false);}
	}
	if ((smart.yaw > 0) && (stupid.yaw < 0))
	{	
		float dif = smart.yaw - stupid.yaw;
		rotate(dif, cmd_vel_topic, false);
	}
	if ((smart.yaw < 0) && (stupid.yaw > 0))
	{	
		float dif = stupid.yaw - smart.yaw;
		rotate(dif, cmd_vel_topic, true);
	}		
	
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "stupid_rob");
	ros::NodeHandle n;
	ros::Rate r(100);
	ros::Publisher cmd_vel_topic = n.advertise<geometry_msgs::Twist>("stupid_burger/cmd_vel", 1000);
	tf::TransformListener listener;
	//tf::StampedTransform transform;
	bool go = false;
	ROS_INFO("-----------START----------");
	sleep(1);
	do
	{
		broadcaster();
		go = transformPoint(listener);
		if (go)
		{
			update_angle(cmd_vel_topic);
			move (cmd_vel_topic);
			r.sleep();
		}
		
	}
	while(round(stupid.p_x) != 0);
	
	ros::spin();
	return 0;
}
