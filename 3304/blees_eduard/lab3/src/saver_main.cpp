#include "lab3/saver_robot.h"

int main( int argc, char** argv )
{
	ros::init(argc, argv, "saver");
	SaveBot saver("lost", "saver", "saver_topic", -5, -5);
	saver.setColor(0.4, 0.1, 0.8);
	saver.start();
	return 0;
}
