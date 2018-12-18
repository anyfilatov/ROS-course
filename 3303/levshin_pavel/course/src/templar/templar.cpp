#include "templarrobot.h"

using namespace std; 

int main(int argc, char **argv){
	srand(time(0));
	ros::init(argc, argv, "templar_" + std::string(argv[1]) + "_" + std::string(argv[2]));
	TemplarRobot templar("templar_" + std::string(argv[1]) + "_" + std::string(argv[2]),
							1.0,
							atoi(argv[1])*1000 + atoi(argv[2]),
							0.0f, 1.0f, 0.0f,
							atof(argv[3]), atof(argv[4]),
							1.0f, 0.5f);
	templar.run();
	return 0;
}	