#pragma once
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"

using namespace std;

struct Robot {
    const char* name;
    float x;
    float y;
    float speed;
    float angle;
} lost_robot, finder_robot;

bool get_robot_coordinates(Robot &robot1, Robot &robot2) {
	static tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        listener.lookupTransform(robot2.name, robot1.name, ros::Time(0), transform);
        robot1.x = transform.getOrigin().x();
        robot1.y = transform.getOrigin().y();
        ROS_INFO("Robot <%s> coordinates: ( %.2f ; %.2f )", robot1.name, robot1.x, robot1.y);
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

void calculate_new_coordinates(Robot &robot, float dest_x, float dest_y, float radius) {
    float new_angle = atan2(dest_y - robot.y, dest_x - robot.x);
    if (new_angle != robot.angle) {
        robot.angle = get_next_coordinate(robot.angle, new_angle, 0.1);
    } else {
        if (sqrt(pow(dest_x - robot.x, 2) + pow(dest_y - robot.y, 2)) > radius) {
            robot.x = get_next_coordinate(robot.x, dest_x, robot.speed);
            robot.y = get_next_coordinate(robot.y, dest_y, robot.speed);
        }
    }

}


void publish_coordinates(Robot &robot) {
	tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(robot.x, robot.y, 0));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", robot.name));

}


void update_pose(geometry_msgs::Pose &pose, Robot &robot) {
    pose.position.x = robot.x;
    pose.position.y = robot.y;
    pose.orientation.z = sin(robot.angle / 2);
    pose.orientation.w = cos(robot.angle / 2);
}

bool is_equal(float x, float y, float dest_x, float dest_y, float radius) {
    return sqrt(pow(dest_x - x, 2) + pow(dest_y - y, 2)) <= radius;
}

bool is_finished(float x, float y, float last_x, float last_y, float radius) {
    static int counter = 0;

    if (sqrt(pow(last_x - x, 2) + pow(last_y - y, 2)) <= radius) {
        counter++;
    } else {
        counter = 0;
    }

    return counter >= 100;
}
