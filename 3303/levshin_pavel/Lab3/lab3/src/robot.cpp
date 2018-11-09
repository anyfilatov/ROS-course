#include "robot.h"

Robot::Robot(string name, string fn, int id, float r, float g, float b, float x, float y, float speed, float min_dist){
	this->name = name;
	this->friendName = fn;
    this->speed = speed;
	this->min_dist = min_dist;
	pub = nh.advertise<visualization_msgs::Marker>(name, 10, true);
	marker.header.frame_id = "/world";
    marker.header.stamp = Time::now();
    marker.ns = "";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
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
	saveTransform();
	
}

void Robot::moveTo(float x, float y){
    float distance = sqrt(pow((x - marker.pose.position.x), 2.0f) + pow((y - marker.pose.position.y), 2.0f));
    marker.pose.position.x += (x - marker.pose.position.x)/distance * speed / ((double)FPS);
    marker.pose.position.y += (y - marker.pose.position.y)/distance * speed / ((double)FPS);	
}

bool Robot::isNearThePoint(float x, float y){
    float squareDistance = pow((x - marker.pose.position.x), 2.0f) + pow((y - marker.pose.position.y), 2.0f);
	return (squareDistance < min_dist*min_dist);
}

bool Robot::isFriendReached(){
    return isNearThePoint(myFriend.getOrigin().x(), myFriend.getOrigin().y());
}

void Robot::getFriendTransform(){
    try
	{
		listener.waitForTransform(BASE_WORLD, friendName, ros::Time(0), ros::Duration(1.0));
		listener.lookupTransform(BASE_WORLD, friendName, ros::Time(0), myFriend);
	}	
	catch (tf::TransformException &ex)
	{
		ROS_INFO("My friend has not yet appeared!");
	}
}

void Robot::saveTransform(){
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(marker.pose.position.x, marker.pose.position.y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), BASE_WORLD, name));
}