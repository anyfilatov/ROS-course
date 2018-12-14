#include "lab5/saver_robot.h"

int main( int argc, char** argv )
{
	ros::init(argc, argv, "saver");
	SaveBot saver("saver", "lost", -5, -5, 0);
	saver.start();
	return 0;
}
