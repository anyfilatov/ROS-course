#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "utilities.h"


bool call_for_help() {

	publish_coordinates(lost_robot);
	return get_robot_coordinates(finder_robot);
	
}

void follow(float speed) {
	
	get_robot_coordinates(finder_robot);
	calculate_new_coordinates(lost_robot, finder_robot, speed);
	ROS_INFO("My coordinates: ( %.2f ; %.2f ; %.2f )", lost_robot.x, lost_robot.y, lost_robot.z);
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "lost");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("lost_topic", 10, true);
    
    lost_robot.id = 0;
    lost_robot.name = "lost";
    lost_robot.x = 10;
    lost_robot.y = 10;
    lost_robot.z = 0;
    
	marker_pub.publish(create_marker(lost_robot));
	
	bool flag = false;
	do {
		flag = call_for_help();
		
		r.sleep();
	} while (!flag);
	
	r.sleep();
	
	ROS_INFO("He found me! I am following him now");
	
	bool stop = false;
	do {
		follow(1.0);
		marker_pub.publish(create_marker(lost_robot));
		r.sleep();
	} while (!stop);
	
	return 0;
}

visualization_msgs::Marker setMarker(float x, float y, float z){
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/my_map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "robots";
	marker.id = 0;#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "utilities.h"


bool call_for_help() {

	publish_coordinates(lost_robot);
	return get_robot_coordinates(finder_robot);
	
}

void follow(float speed) {
	
	get_robot_coordinates(finder_robot);
	calculate_new_coordinates(lost_robot, finder_robot, speed);
	ROS_INFO("My coordinates: ( %.2f ; %.2f ; %.2f )", lost_robot.x, lost_robot.y, lost_robot.z);
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "lost");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("lost_topic", 10, true);
    
    lost_robot.id = 0;
    lost_robot.name = "lost";
    lost_robot.x = 10;
    lost_robot.y = 10;
    lost_robot.z = 0;
    
	marker_pub.publish(create_marker(lost_robot));
	
	bool flag = false;
	do {
		flag = call_for_help();
		
		r.sleep();
	} while (!flag);
	
	r.sleep();
	
	ROS_INFO("He found me! I am following him now");
	
	bool stop = false;
	do {
		follow(1.0);
		marker_pub.publish(create_marker(lost_robot));
		r.sleep();
	} while (!stop);
	
	return 0;
}

	marker.type = visualization_msgs::Marker::POINTS;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
 	marker.pose.position.y = 0;
   	marker.pose.position.z = 0;
   	marker.pose.orientation.x = 0.0;
   	marker.pose.orientation.y = 0.0;
   	marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
	marker.scale.x = 1.0;
   	marker.scale.y = 1.0;
   	marker.scale.z = 1.0;
	marker.color.r = 0.0;
   	marker.color.g = 1.0;
   	marker.color.b = 0.0;
   	marker.color.a = 1.0;
	geometry_msgs::Point p;
	p.x = x;
	p.y = y;
	p.z = z;
	marker.points.push_back(p);
	return marker;
}

tf::StampedTransform transformPoint(const tf::TransformListener &listener){
	tf::StampedTransform transform;
	bool success = false;
	do
	{
		try{	
		listener.lookupTransform("world2", "finder", ros::Time(0), transform);
		ROS_INFO("origin: %.2f, %.2f, %.2f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
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

int main( int argc, char** argv )
{
	ros::init(argc, argv, "lost");
	ros::NodeHandle n;
	ros::Rate r(100);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("lost_topic", 10, true);
	tf::TransformBroadcaster broadcaster;
	tf::TransformListener listener;
	tf::StampedTransform transform;
	float start_x = -10;
	float start_y = -10;
	float start_z = 1;
	bool flag = false;
	float cur_x = start_x;
	float cur_y = start_y;
	float cur_z = start_z;
	marker_pub.publish(setMarker(start_x, start_y, start_z));
	broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(start_x, start_y, start_z)), ros::Time::now(), "world", "lost"));
	sleep(1);
	do
	{
		marker_pub.publish(setMarker(start_x, start_y, start_z));
		broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(start_x, start_y, start_z)), ros::Time::now(), "world", "lost"));
		ROS_INFO("HELP! I am at: %.2f, %.2f, %.2f", cur_x, cur_y, cur_z);
		transform = transformPoint(listener);
		
		r.sleep();
	}
	while((transform.getOrigin().x() != start_x) && (transform.getOrigin().y() != start_y));
	ROS_INFO("He found me! I am following him now");
	do
	{
		
		transform = transformPoint(listener);
		float cur_x = transform.getOrigin().x();
		float cur_y = transform.getOrigin().y();
		float cur_z = transform.getOrigin().z();
		marker_pub.publish(setMarker(cur_x, cur_y, cur_z));
		broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(start_x, start_y, start_z)), ros::Time::now(), "world", "lost"));
		ROS_INFO("My coordinates: %.2f, %.2f, %.2f", cur_x, cur_y, cur_z);
		
		
		r.sleep();
	}
	while(n.ok());
	ros::spinOnce();
	return 0;
}

