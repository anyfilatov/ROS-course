#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <stdlib.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

visualization_msgs::Marker setMarker(float x, float y, float z){
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/my_map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "robots";
	marker.id = 1;
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
	marker.color.r = 1.0;
       	marker.color.g = 0.0;
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
		listener.lookupTransform("world", "lost", ros::Time(0), transform);
		ROS_INFO("origin: %.2f, %.2f, %.2f; rotation: %.2f, %.2f, %.2f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z(),
		transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z());
		success = true;
		}
		catch(tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
		}
		sleep(1);
	}while(!success);
	return transform;
	
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "finder");
	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("finder_topic", 10, true);
	tf::TransformListener listener;
	ros::Rate r(1);
	float start_x = 0;
	float start_y = 0;
	float start_z = 1;
	marker_pub.publish(setMarker(start_x, start_y, start_z));
	sleep(1);
	tf::TransformBroadcaster broadcaster;
	broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(start_x, start_y, start_z)), ros::Time::now(), "world2", "finder"));
	tf::StampedTransform transform;
	float count = 0;
	float cur_x = 0;
	float cur_y = 0;
	float cur_z = 0;
	do
	{
		transform = transformPoint(listener);
		cur_x = transform.getOrigin().x() / 8 - count;
		cur_y = transform.getOrigin().y() / 8 - count;
		cur_z = transform.getOrigin().z();
		marker_pub.publish(setMarker(cur_x, cur_y, cur_z));
		ROS_INFO("My coordinates: %.2f, %.2f, %.2f", cur_x, cur_y, cur_z);
		broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(cur_x, cur_y, cur_z)), ros::Time::now(), "world2", "finder")); 
		r.sleep();
		count++;
	}
	while((cur_x != transform.getOrigin().x()) && (cur_y != transform.getOrigin().y()) );
	ROS_INFO("I found him!");
	sleep(1);
	count = 0;
	do
	{
		cur_x++;
		cur_y++;
		marker_pub.publish(setMarker(cur_x, cur_y, cur_z));
		ROS_INFO("My coordinates: %.2f, %.2f, %.2f", cur_x, cur_y, cur_z);
		broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(cur_x, cur_y, cur_z)), ros::Time::now(), "world2", "finder"));
		r.sleep();
	}
	while((cur_x != start_x) && (cur_y != start_y) );
	ros::spinOnce();	
	return 0;
}

