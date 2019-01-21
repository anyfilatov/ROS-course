#include <ros/ros.h>
#include <coordinate/Coordinate.h>
#include <cstdlib>
#include <ctime>
#include <vector> 
#include <math.h>
#include <string>
#include <iostream>

using namespace ros;

double antirocketRadius = 200;
int antirocketsCount = 3;

std::vector<coordinate::Coordinate> Antirockets;

double generateRandCoord()
{
	return (double)(min + rand() % (max - min));
}

double computeDistance(coordinate::Coordinate c1, coordinate::Coordinate c2)
{
	return sqrt(pow(c1.x - c2.x, 2) + pow(c1.y - c2.y, 2));
}

void defend(const coordinate::Coordinate &destination)
{
	ROS_INFO("Rocket was detected. The destination is (%f; %f)", destination.x, destination.y);
	for(int i=0; i<Antirockets.size(); i++)
	{
		if(computeDistance(Antirockets[i], destination) <= antirocketRadius)
		{
			if(rand() % 10 <= 8)
			{
				ROS_INFO("Rocket shot down by AA %i", i + 1);
				return;
			}
		}
	}
	ROS_INFO("Rocket NOT destroyed");
}

void createAntirockets()
{
	for(int i = 0; i < antirocketsCount; i++)
	{
		coordinate::Coordinate antirocket;
		antirocket.x=generateRandCoord();
		antirocket.y=generateRandCoord();

		Antirockets.push_back(antirocket);
		ROS_INFO("Antirocket %i placed at coordinates (%f; %f)", i+1, antirocket.x, antirocket.y);
	}
}

int main(int argc, char **argv)
{
	init(argc, argv, "defender");
	srand(time(NULL));

	createAntirockets();

	NodeHandle n;
	Subscriber sub=n.subscribe("Rockets", 1000, defend);
	spin();
	return 0;
}
