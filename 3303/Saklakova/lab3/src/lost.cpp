#include "ros/ros.h"
#include "Robot.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "lost");
	ros::Time::init();
	ros::Rate loop(1);
	Robot *robot = new Robot("lost", 0, 0.5, 0.0, 0.5);

	while (ros::ok()){
		if (robot->take_pos("assis")) {
			robot->broadcast_pos();
		} else {			
			robot->move(rand()%5, rand()%5);
		}
	}
	return 0;
}
