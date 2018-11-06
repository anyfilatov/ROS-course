#include "lab3/lost_robot.h"

int main( int argc, char** argv )
{
	ros::init(argc, argv, "lost");
	LostBot lost("saver", "lost", "lost_topic", 1, -1);
	lost.setColor(0.9, 0.1, 0.1);
	lost.start();
	return 0;
}
