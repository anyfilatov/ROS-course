#include "turtle.h"

using namespace std;
using namespace ros;

Turtle::Turtle(string name, int id, float r, float g, float b, float startX, float startY, float speed)
{
	this->name = name;
    this->speed = speed;
	pub = nh.advertise<geometry_msgs::Twist>("/" + name + "/cmd_vel", 10, true);
}

void Turtle::move(float linear, float angle)
{
	geometry_msgs::Twist msg;
	msg.linear.x = linear;
	msg.angular.z = angle;
	pub.publish(msg);
}

bool Turtle::isNearly(float x, float y, float dist)
{
	float distance = sqrt(pow(x, 2.0f) + pow(y, 2.0f));
	ROS_INFO("%3.2f %3.2f", x, y);
	return (distance < dist);
}

bool Turtle::isDestenation()
{
	return isNearly(partner.getOrigin().x(), partner.getOrigin().y(), 0.5);
}

tf::Transform Turtle::getTransform(string name)
{
    static tf::TransformListener listener;
    static tf::StampedTransform tr;
	tf::Transform transform;
	try
	{
		listener.waitForTransform(this->name, name, ros::Time(0), ros::Duration(1));
		listener.lookupTransform(this->name, name, ros::Time(0), tr);
	}	
	catch (tf::TransformException &ex)
	{
	//	Duration(1.0).sleep();
	}
	transform.setOrigin(tr.getOrigin());
	transform.setRotation(tr.getRotation());
	//ROS_INFO("%3.2f %3.2f", transform.getOrigin().x(), transform.getOrigin().x());
	return transform;
}

void Turtle::getPartnerTransform(string name){
	partner = getTransform(name);
}

void Turtle::getHomeTransform(){
	home = getTransform("odom");
}

float Turtle::calculateRotation(tf::Transform q_to){
    float x_cur = q_to.getOrigin().x();
    float y_cur = q_to.getOrigin().y();
    float a = sqrt(x_cur*x_cur + y_cur*y_cur);
    float alfa = acos(x_cur / a);
    if(y_cur <0){alfa = -alfa;}
    return alfa;
}

float Turtle::getAngle(tf::Transform q_to){
    float a = calculateRotation(q_to);
    if(a>M_PI){a -= 2*M_PI;}
    if(a< - M_PI){a += 2* M_PI;}
    return a;
}