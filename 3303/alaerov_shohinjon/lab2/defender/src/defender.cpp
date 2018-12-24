#include <ros/ros.h>
#include <coordinate/Coordinate.h>
#include <cstdlib>
#include <ctime>
#include <vector> 
#include <math.h>
#include <string>
#include <iostream>
using namespace ros;
double AntirocketRadius = 200;
int MinCoordinate = 0;
int MaxCoordinate = 1000;
int AntirocketsCount = 3;
std::vector<coordinate::Coordinate> Antirockets;
double GenerateCoordinate()
{
	return (double)(MinCoordinate + rand() % (MaxCoordinate - MinCoordinate + 1));
}
double ComputeDistance(coordinate::Coordinate c1, coordinate::Coordinate c2)
{
	return sqrt(pow(c1.x-c2.x, 2)+pow(c1.y-c2.y, 2));
}
void Defende(const coordinate::Coordinate &destination)
{
	ROS_INFO("Rocket was detected. The destination is (%f; %f)", destination.x, destination.y);
	for(int i=0; i<Antirockets.size(); i++)
	{
		if(ComputeDistance(Antirockets[i], destination) <= AntirocketRadius)
		{
			if(rand()%10 <= 8)
			{
				ROS_INFO("Rocket was shot down by antirocket %i", i+1);
				return;
			}
		}
	}
	if(rand()%100 <= 5)
	{
		ROS_INFO("Rocket was shot down without use antirocket");
		return;
	}
	ROS_INFO("Rocket was not shot down");
}
void CreateAntirockets()
{
	for(int i=0; i<AntirocketsCount; i++)
	{
		coordinate::Coordinate antirocket;
		antirocket.x=GenerateCoordinate();
		antirocket.y=GenerateCoordinate();
		Antirockets.push_back(antirocket);
		ROS_INFO("Antirocket %i was placed at coordinates (%f; %f)", i+1, antirocket.x, antirocket.y);
	}
}
int main(int argc, char **argv)
{
	init(argc, argv, "defender");
	srand(time(NULL));
	CreateAntirockets();
	NodeHandle n;
	Subscriber sub=n.subscribe("Rockets", 1000, Defende);
	spin();
	return 0;
}
