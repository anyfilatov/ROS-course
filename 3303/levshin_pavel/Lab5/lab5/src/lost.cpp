#include "lostrobot.h"

using namespace std; 

int main(int argc, char **argv){
	srand(time(0));
	ros::init(argc, argv, "lost");
	
	LostRobot lost("lost", "rescue", 1, rand() % (2 * DISPERSION) - DISPERSION, rand() % (2 * DISPERSION) - DISPERSION, 1.0f, 2.0f, 0.5f);
	lost.run();
	return 0;
}	