#include "lab5/lost_robot.h"

int main( int argc, char** argv )
{
	ros::init(argc, argv, "lost");
	LostBot lost("lost", "saver", 2, -2, 0);
	lost.start();
	return 0;
}
