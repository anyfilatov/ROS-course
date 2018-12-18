#include "zerglings_spawner_robot.h"

using namespace std; 

int main(int argc, char **argv){
	srand(time(0));
	ros::init(argc, argv, "Zerglings_spawner");
	
	int id, health, cooldownSpawn;
	float x, y;
	ros::param::get("~id", id);
	ros::param::get("~x", x);
	ros::param::get("~y", y);
	ros::param::get("~health", health);
	ros::param::get("~cooldownSpawn", cooldownSpawn);

	ZerglingSpawnerRobot zergling_spawner(id, 
								health,
								cooldownSpawn,
                                tf::Vector3(x, y, 0), 
                                tf::tfVector4(0, 0, 0, 1),
                                tf::Vector3(2, 2, 0.01),
                                tf::tfVector4(0.545, 0.271, 0.007, 1), 2.0f, 0.1f);
	zergling_spawner.run();

	
	return 0;
}	