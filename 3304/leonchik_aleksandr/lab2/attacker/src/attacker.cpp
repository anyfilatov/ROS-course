#include <cstdlib>
#include <ctime>
#include <ros/ros.h>
#include <coordinate/Coordinate.h>

using namespace ros;

int min = 1;
int max = 1000;

double generateRandCoord()
{
	return (double)(min + rand() % (max - min));
}

int main(int argc, char **argv)
{
	init(argc, argv, "attacker");
	srand(time(NULL));
	NodeHandle n;

	Publisher pub = n.advertise<coordinate::Coordinate>("Rockets", 1000);

	coordinate::Coordinate dest;
	
	dest.x = generateRandCoord();
	dest.y = generateRandCoord();

	Rate loopRate(0.5);
	sleep(1);
	pub.publish(dest);

	loopRate.sleep();
	ROS_INFO("Rocket Launched to (%f, %f)", dest.x, dest.y);
}
