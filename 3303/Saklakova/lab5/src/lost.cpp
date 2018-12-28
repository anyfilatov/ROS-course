#include "ros/ros.h"
#include "Robot.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "lost");
	
	Robot *robot = new Robot("lost");
	sleep(1.0);
	
	while (ros::ok()){
		if (robot->take_pos("assis")) {
			robot->broadcast_pos(0);
			if (robot->isPosition(-20, -20)) {
		        	ROS_INFO("Lost robot end");
		        	robot->move(rand()%20, rand()%20);
            		}
		} else {			
			robot->move(rand()%10, rand()%10, 5);
		}
	}
	return 0;
}
