#include "turtle.h"

using namespace std;
using namespace ros;

Turtle::Turtle(string name, int id, float r, float g, float b, float startX, float startY, float speed)
{
	partner_X = 0.0;
	partner_Y = 0.0;
	this->name = name;
    this->speed = speed;
	pub = nh.advertise<visualization_msgs::Marker>(name.c_str(), 10, true);
	marker.header.frame_id = "/map";
    marker.header.stamp = Time::now();
    marker.ns = "";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.pose.position.x = startX;
    marker.pose.position.y = startY;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
	geometry_msgs::Point p;
	p.x = 0;
	p.y = 0;
	p.z = 0;
	marker.points.push_back(p);
	marker.lifetime = Duration();
	pub.publish(marker);
	sendTransform(startX, startY);
}

void Turtle::move(float x, float y)
{
	
	float magnitude = sqrt(pow((x - marker.pose.position.x), 2.0f) + pow((y - marker.pose.position.y), 2.0f));
    marker.pose.position.x += (x - marker.pose.position.x)/magnitude * speed * (1/30.0f);
    marker.pose.position.y += (y - marker.pose.position.y)/magnitude * speed * (1/30.0f);
    pub.publish(marker);		
    sendTransform(marker.pose.position.x, marker.pose.position.y);
}

bool Turtle::isNearly(float x, float y)
{
	float distance = sqrt(pow((x - marker.pose.position.x), 2.0f) + pow((y - marker.pose.position.y), 2.0f));
	return (distance < 0.3f);
}

bool Turtle::isDestenation()
{
	return isNearly(partner_X, partner_Y);
}

void Turtle::getTransform(string name)
{
    static tf::TransformListener listener;
    static tf::StampedTransform tr;
	try
	{
		listener.waitForTransform("world", name, ros::Time(0), ros::Duration(1));
		listener.lookupTransform("world", name, ros::Time(0), tr);
 		partner_X = tr.getOrigin().x();
		partner_Y = tr.getOrigin().y();
	}	
	catch (tf::TransformException &ex)
	{
	//	Duration(1.0).sleep();
	}
}

void Turtle::sendTransform(float x, float y)
{
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x, y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
}