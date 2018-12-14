#include "ros/ros.h"
#include "Robot.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "assis");
	ros::Time::init();
	ros::Rate loop(1);
	Robot *robot = new Robot("assis", 1, 0.0, 0.0, 1.0);

	while (ros::ok()){
		if (!robot->take_pos("lost")) {
			robot->broadcast_pos();
		} else {
			robot->move(0, 0);
		}
	}
	return 0;
}
