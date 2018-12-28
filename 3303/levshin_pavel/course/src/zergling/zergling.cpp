#include "zerglingrobot.h"

using namespace std; 

int main(int argc, char **argv){
	srand(time(0));
	
	ros::init(argc, argv, "zergling_" + std::string(argv[1]) + "_" + std::string(argv[2]));

	ZerglingRobot zergling("zergling_" + std::string(argv[1]) + "_" + std::string(argv[2]),
							atoi(argv[1])*1000 + atoi(argv[2]),
							1.0f, 0.0f, 0.0f,
							atof(argv[3]), atof(argv[4]),
							2.0f, 0.5f);
	zergling.run();
	return 0;
}	