#include "ros/ros.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelStates.h"
#include <fstream>
#include "string.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
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

tf::StampedTransform transformPoint2(const tf::TransformListener &listener)  //listener tf smart_burger
{
	tf::StampedTransform transform;
	bool success = false;
	do
	{
		try{
		listener.lookupTransform("smart_burger/odom", "smart_burger/base_footprint", ros::Time(0), transform);
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
	}
	while(!success);
	return transform;
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

void move (ros::Publisher cmd_vel_topic, float end_x, int quadrant)
{
	geometry_msgs::Twist vel_msg;
	tf::TransformListener listener;
	tf::StampedTransform transform;
	transform = transformPoint2(listener);
	float count = 0;
	float cur_x = floor(transform.getOrigin().x()*100)/100;
	if ((quadrant == 2) || (quadrant == 3)) {
	cur_x = -1*cur_x;
	end_x = -1*end_x;}
	while(cur_x < end_x)
	{
		vel_msg.linear.x = 0.5;
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;
		vel_msg.angular.x = 0;
        	vel_msg.angular.y = 0;
		vel_msg.angular.z = 0;
		cmd_vel_topic.publish(vel_msg);
		ROS_INFO("cur_x: %.2f", cur_x);
		sleep(1);
		transform = transformPoint2(listener);
		cur_x = floor(transform.getOrigin().x()*100)/100;
		if ((quadrant == 2) || (quadrant == 3)) {cur_x = -1*cur_x;}
	}
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;
	cmd_vel_topic.publish(vel_msg);
	transformPoint2(listener);
	
}

void broadcaster ()
{
	tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::TransformListener listener;
	transform = transformPoint2(listener);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"world_smart", "smart_burger"));
	
}

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg) //writer tf
{
	tf::TransformBroadcaster br;
	tf::Transform transform;	
	ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
	transform.setOrigin( tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z));
	ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"world_smart", "smart_burger"));
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	setPosition(&smart, floor(transform.getOrigin().x()*100)/100, floor(transform.getOrigin().y()*100)/100, floor(transform.getOrigin().z()*100)/100, roll, pitch, yaw);
	//ROS_INFO("roll, pitch, yaw=%1.2f  %1.2f  %1.2f", roll, pitch, yaw);
	// roll  --> rotate around vertical axis
      // pitch --> rotate around horizontal axis
      // yaw   --> rotate around depth axis
}

void broadcaster_worlds()
{
	tf::TransformBroadcaster br;
	tf::Transform transform;	
	transform.setOrigin( tf::Vector3(0.0,0.0, 0.0));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"world_smart", "world_stupid"));
	ROS_INFO("---------broadcaster worlds--------");
}


tf::StampedTransform transformPoint(const tf::TransformListener &listener)  //listener tf stupid_burger
{
	tf::StampedTransform transform;
	bool success = false;
	do
	{
		try{
		listener.lookupTransform("world_stupid", "stupid_burger", ros::Time(0), transform);
		ROS_INFO("origin: %.2f, %.2f, %.2f; rotation: %.2f, %.2f, %.2f, %.2f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
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

int quadrant(struct robot *name)
{
	int num = 0;
	if ((name->p_x > 0) && (name->p_y > 0)) {num = 1; /* I quadrant*/}
	if ((name->p_x < 0) && (name->p_y > 0)) {num = 2; /* II quadrant*/}
	if ((name->p_x < 0) && (name->p_y < 0)) {num = 3; /* III quadrant*/}
	if ((name->p_x > 0) && (name->p_y < 0)) {num = 4; /* IV quadrant*/}
	if ((name->p_x > 0) && (name->p_y == 0 || name->p_y == -0)) {num = 5; /*axis +OX*/}
	if ((name->p_x == 0 || name->p_x == -0) && (name->p_y > 0)) {num = 6; /*axis +OY*/}
	if ((name->p_x < 0) && (name->p_y == 0 || name->p_y == -0)) {num = 7; /*axis -OX*/}
	if ((name->p_x == 0 || name->p_x == -0) && (name->p_y < 0)) {num = 8; /*axis -OY*/}
	return num;
}

float direction2points(float x1, float y1, float x2, float y2)
{
	float res = sqrt(pow(x2 - x1, 2) + pow(y2 -y1, 2));
	return floor(res*100)/100;
}

float angle_rotation(struct robot *name, int num)
{
	//int num = quadrant(name);
	ROS_INFO("quadrant: %d", num);
	float angle =0;
	switch(num){
	case 1:
	  {
	  	float opposite = direction2points(0.0, name->p_y, name->p_x, name->p_y);
	  	float adjoining = direction2points(0.0, 0.0, 0.0, name->p_y);
	  	angle = atan(opposite / adjoining) * 180.0 / PI;
		ROS_INFO("angle: %.2f", angle);
	  	break;
	  }
	case 2:
	  {
	  	float opposite = direction2points(name->p_x, 0.0, name->p_x, name->p_y);
	  	float adjoining = direction2points(0, 0, name->p_x, 0);
	  	angle = 270 + (atan(opposite / adjoining) * 180.0 / PI);
		ROS_INFO("angle: %.2f", angle);
          	break;
	  }
	case 3:
	  {
	  	float opposite = direction2points(0.0, name->p_y, name->p_x, name->p_y);
	  	float adjoining = direction2points(0.0, 0.0, 0.0, name->p_y);
	  	angle = 180 + (atan(opposite / adjoining) * 180.0 / PI);
		ROS_INFO("angle: %.2f", angle);
	  	break;
	  }
	case 4:
	  {
	  	float opposite = direction2points(name->p_x, 0.0, name->p_x, name->p_y);
	  	float adjoining = direction2points(0, 0, name->p_x, 0);
	  	angle = 90 + (atan(opposite / adjoining) * 180.0 / PI);
		ROS_INFO("angle: %.2f", angle);
	  	break;
	  }
	case 5:
	  angle = 90;
	  break;
	case 6:
	  angle = 0;
	  break;
	case 7:
	  angle = 270;
	  break;
	case 8:
	  angle = 180;
	  break;
	}
	return angle;

}

void turn (int quadrant, ros::Publisher cmd_vel_topic)
{
	if (quadrant != 6)
	{	
		float cur_angle = floor((smart.yaw *180 / PI)*100) /100;
		ROS_INFO("cur_angle: %.2f", cur_angle);
		float angle = 90 - cur_angle;
		ROS_INFO("cur_angle to rotate: %.2f", angle);
		bool clockwise;
		if ((quadrant == 1) || (quadrant == 4) || (quadrant == 5) || (quadrant == 8)){clockwise = false;}
		if ((quadrant == 2) || (quadrant == 3) || (quadrant == 7)){clockwise = true;}
		rotate (angle, cmd_vel_topic,clockwise);
		ROS_INFO("---I rotete 1---------");
	}
	//float angle = 90 - angle_rotation(&smart, quadrant);
	switch(quadrant){
	case 1:
	  {
		float angle = 180 - angle_rotation(&smart, quadrant);
		rotate (angle, cmd_vel_topic, false);
		break;
	  }
	case 2:
	  {
		float angle = 90 + angle_rotation(&smart, quadrant);
		rotate (angle, cmd_vel_topic, true);
		break;
	  }
	case 3:
	  {
		float angle = 90 - angle_rotation(&smart, quadrant);
		rotate (angle, cmd_vel_topic, true);
		break;
	  }
	case 4:
	  {
		float angle1 = angle_rotation(&smart, quadrant);
		ROS_INFO("angle to rotate: %.2f", angle1);
		float angle = abs(90 - angle_rotation(&smart, quadrant));
		rotate (angle, cmd_vel_topic, false);
		ROS_INFO("cur_angle to rotate: %.2f", angle);
		break;
	  }
	case 5:
	  {
		float angle = 90;
		rotate (angle, cmd_vel_topic, false);
		break;
	  }
	case 6:
	  {
		float angle = 180;
		rotate (angle, cmd_vel_topic, true);
		break;
	  }
	case 7:
	  {
		float angle = 90;
		rotate (angle, cmd_vel_topic, true);
		break;
	  }
	case 8:
	  {
		float angle = 0;
		rotate (angle, cmd_vel_topic, true);
		break;
	  }
	}

}

void update_angle(int quadrant, ros::Publisher cmd_vel_topic)
{
	float betta;
	float cur_angle;	
	switch(quadrant){
	case 1:
	  {
		betta = angle_rotation(&smart, quadrant);
		cur_angle = floor((smart.yaw*180 / PI)*100 )/ 100 + 180;
		break;
	  }
	case 2:
	  {
		betta = angle_rotation(&smart, quadrant) - 270;
		cur_angle = floor((smart.yaw*180 / PI)*100 )/ 100 + 90;
		break;
	  }
	case 3:
	  {
		betta = angle_rotation(&smart, quadrant) - 180;
		cur_angle = floor((smart.yaw*180 / PI)*100 )/ 100;
		break;
	  }
	case 4:
	  {
		betta = angle_rotation(&smart, quadrant) -90;
		cur_angle = floor((smart.yaw*180 / PI)*100 )/ 100 - 90;
		break;
	  }
	
	}
	float gamma = 90 - betta;
	float dif = abs(gamma - cur_angle);
	if (gamma > cur_angle){ rotate(dif, cmd_vel_topic, false);}
	if (gamma < cur_angle){rotate(dif, cmd_vel_topic, true);}

}

void find(ros::Publisher cmd_vel_topic)
{
	float x2, end_x;
	int num = quadrant(&stupid);	
	float angle = angle_rotation(&stupid, num);
	rotate(angle, cmd_vel_topic, true);
	if (stupid.p_x < 0) {end_x = stupid.p_x + 1;}
	if(stupid.p_x > 0) {end_x = abs(stupid.p_x -1);}
	//if (stupid.p_x == 0){end_x = stupid.p_y;}
	move (cmd_vel_topic, end_x, num);
	ROS_INFO("---I finds him---------");
	num = quadrant(&smart);
	turn (num, cmd_vel_topic);
	ROS_INFO("---I rotate 2---------");
	//move (cmd_vel_topic, end_x);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "smart_rob");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    tf::TransformListener listener;
    tf::StampedTransform transform, transform2;
    ros::Publisher cmd_vel_topic = n.advertise<geometry_msgs::Twist>("smart_burger/cmd_vel", 1000);
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("tf", 1000);
    ros::Subscriber sub = n.subscribe("smart_burger/odom", 100, chatterCallback);
    ROS_INFO("-----------START----------");
    transform = transformPoint(listener);
    find(cmd_vel_topic);
    float end_x = 0;
    geometry_msgs::Twist vel_msg;
    float cur_x = round(transform2.getOrigin().x());
    do
   {
	broadcaster();
	transform2 = transformPoint2(listener);
	broadcaster_worlds();
	int num = quadrant(&smart);
	if ((num == 1) || (num == 2) || (num == 3) || (num == 4)){update_angle(num, cmd_vel_topic);}
	cur_x = round(transform2.getOrigin().x());
	if(cur_x != end_x)
	{
		vel_msg.linear.x = 0.5;
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;
		vel_msg.angular.x = 0;
        	vel_msg.angular.y = 0;
		vel_msg.angular.z = 0;
		cmd_vel_topic.publish(vel_msg);
		ROS_INFO("cur_x: %.2f", cur_x);
		sleep(1);
	}
	vel_msg.linear.x = 0;
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;
	cmd_vel_topic.publish(vel_msg);
	//transformPoint2(listener);
	ROS_INFO("cur_x: %.2f", cur_x);
   }while(cur_x != end_x); //cur_x != end_x
    ros::spin();
    return 0;
}
