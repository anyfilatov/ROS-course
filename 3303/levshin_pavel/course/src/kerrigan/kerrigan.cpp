#include "kerrigan_robot.h"

using namespace std; 

int main(int argc, char **argv){
	srand(time(0));
	ros::init(argc, argv, "kerrigan");
	KerriganRobot kerrigan("kerrigan",
							1.0,
							1,
							0.353f, 0.0f, 0.616f,
							10, 0,
							1.0f, 0.5f);
	kerrigan.run();
	return 0;
}	