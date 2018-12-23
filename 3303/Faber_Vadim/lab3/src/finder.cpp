#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "utilities.h"


int main(int argc, char **argv) {

	ros::init(argc, argv, "finder");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("finder_topic", 10, true);
    
    finder_robot.id = 1;
    finder_robot.name = "finder";
    finder_robot.x = 0;
    finder_robot.y = 0;
    finder_robot.z = 0;

    exit_point.x = 0;
    exit_point.y = 0;
    exit_point.z = 0;
    
	lost_robot.name = "lost";
	
	marker_pub.publish(create_marker(finder_robot));
	
	bool flag = false;
	while (!flag) {
		flag = get_robot_coordinates(lost_robot);
		r.sleep();
	}

    while (finder_robot.x != lost_robot.x || finder_robot.y != lost_robot.y || finder_robot.z != lost_robot.z) {
		calculate_new_coordinates(finder_robot, lost_robot, 1.0);
		
        ROS_INFO("Moving towards the lost robot..");

    	marker_pub.publish(create_marker(finder_robot));
    	
    	publish_coordinates(finder_robot);

        r.sleep();
    }

    ROS_INFO("Found the lost robot, moving towards the exit..");

    while (finder_robot.x != exit_point.x || finder_robot.y != exit_point.y || finder_robot.z != exit_point.z) {
        calculate_new_coordinates(finder_robot, exit_point, 1.0);

    	marker_pub.publish(create_marker(finder_robot));

        publish_coordinates(finder_robot);

        r.sleep();
    }

    ROS_INFO("I have finished");

    r.sleep();

    return 0;
}
