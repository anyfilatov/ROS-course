#include <cstdlib>
#include <ctime>
#include <ros/ros.h>
#include <coordinate/Coordinate.h>
using namespace ros;
int MinCoordinate = 0;
int MaxCoordinate = 1000;
double GenerateCoordinate()
{
	return (double)(MinCoordinate + rand() % (MaxCoordinate - MinCoordinate + 1));
}
int main(int argc, char **argv)
{
	init(argc, argv, "attacker");
	NodeHandle n;
	Publisher pub = n.advertise<coordinate::Coordinate>("Rockets", 1000);
	coordinate::Coordinate destination;
	srand(time(NULL));
	destination.x = GenerateCoordinate();
	destination.y = GenerateCoordinate();
	Rate loop_rate(0.5);
	sleep(1);
	pub.publish(destination);
	loop_rate.sleep();
	ROS_INFO("Rocket was launched to (%f, %f)", destination.x, destination.y);
}
