#include <ros/ros.h>
#include "message/coordinates.h"
#include <stdlib.g>

int main(int argc, char **argv) {
	ros::init(argc, argv, "publisher");

	ros::NodeHandler nh;

	ros::Publisher pub = nh.advertise<message::coordinates>("heaven", 1000);

	srand(time(0));

	ros::Duration duration(5);

	while(ros::ok()) {
		message::coordinates attack_point;
	
		attack_point.x = rand() % 5 + 1;
		attack_point.y = rand() % 5 + 1;

		ROS_INFO_STREAM("attack point = ( " << attack_point.x << " ; " << attack_point.y << " )");

		pub.publish(attack_point);

		duration.sleep();
	}
}
