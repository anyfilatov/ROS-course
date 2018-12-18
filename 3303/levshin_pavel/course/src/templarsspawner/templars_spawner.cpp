#include "templars_spawner_robot.h"

using namespace std; 

int main(int argc, char **argv){
	srand(time(0));
	ros::init(argc, argv, "templar_spawner");
	
	int id, countWaves, countUnitInWave, cooldownSpawn;
	float x, y;
	ros::param::get("~id", id);
	ros::param::get("~x", x);
	ros::param::get("~y", y);
	ros::param::get("~countWaves", countWaves);
	ros::param::get("~countUnitInWave", countUnitInWave);
	ros::param::get("~cooldownSpawn", cooldownSpawn);

	TemplarsSpawnerRobot templar_spawner(id, 
                                countWaves, countUnitInWave, cooldownSpawn,
                                tf::Vector3(x, y, 0), 
                                tf::tfVector4(0, 0, 0, 1),
                                tf::Vector3(4, 6, 0.01),
                                tf::tfVector4(1, 1, 0, 1), 2.0f, 0.1f);
	templar_spawner.run();

	
	return 0;
}	