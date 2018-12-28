#include "ros/ros.h"
#include "Robot.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "assis");
	ros::Time::init();
	
	Robot *robot = new Robot("assis");

	while (ros::ok()){
		if (!robot->take_pos("lost")) {
			robot->broadcast_pos(3);
		} else {
			robot->move(-20, -20, 1.2);
				if (robot->isPosition(-20, -20)) {
				ROS_INFO("Assis robot end.");
			}
		}
	}
	return 0;
}
