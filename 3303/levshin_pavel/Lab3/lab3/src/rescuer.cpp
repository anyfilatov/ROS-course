#include "rescuerrobot.h"

using namespace std; 

int main(int argc, char **argv){
	srand(time(0));
	ros::init(argc, argv, "resquer");
	RescuerRobot rescuer("resquer", "lost", 2, 0.0f, 1.0f, 0.0f, 0, 0, 2.0f, 0.1f);
	rescuer.run();
	return 0;
}	