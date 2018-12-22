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
