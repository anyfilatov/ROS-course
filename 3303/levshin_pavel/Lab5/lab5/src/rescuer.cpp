#include "rescuerrobot.h"

using namespace std; 

int main(int argc, char **argv){
	srand(time(0));
	ros::init(argc, argv, "rescue");
	RescuerRobot rescuer("rescue", "lost", 2, 0, 0, 2.0f, 4.0f, 0.5f);
	rescuer.run();
	return 0;
}	