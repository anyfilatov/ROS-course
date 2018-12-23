#pragma once
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <string> 

struct Robot {
	int id;
	std::string name;
	float x;
    float y;
    float z;
} lost_robot, finder_robot, exit_point;


visualization_msgs::Marker create_marker(Robot &robot) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/point_on_map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "robots";
    marker.id = robot.id;
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
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    geometry_msgs::Point p;
    p.x = robot.x;
    p.y = robot.y;
    p.z = robot.z;
    marker.points.push_back(p);
    return marker;
}


bool get_robot_coordinates(Robot &robot) {
	static tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        listener.lookupTransform("world", robot.name, ros::Time(1), transform);
        robot.x = transform.getOrigin().x();
        robot.y = transform.getOrigin().y();
        robot.z = transform.getOrigin().z();
        ROS_INFO("Robot %s coordinates: ( %.2f ; %.2f ; %.2f )", robot.name, robot.x, robot.y, robot.z);
        return true;
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        return false;
    }
}


float get_next_coordinate(float current, float destination, float speed) {
    if (current + speed < destination) {
        return current + speed;
    } else if (current - speed > destination) {
        return current - speed;
    }
    return destination;
}


void calculate_new_coordinates(Robot &from, Robot &towards, float speed) {
    from.x = get_next_coordinate(from.x, towards.x, speed);
    from.y = get_next_coordinate(from.y, towards.y, speed);
    from.z = get_next_coordinate(from.z, towards.z, speed);
}

void publish_coordinates(Robot &robot) {
	tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(robot.x, robot.y, robot.z));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", robot.name));
}

