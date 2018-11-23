#include <ros/ros.h>
#include <coordinates/Coordinates.h>
#include <cstdlib>
#include <ctime>
#include <vector> 
#include <math.h>
#include <string>
#include <iostream>
using namespace ros;
double AntimissileRadius = 200;
int MinCoordinate = 0;
int MaxCoordinate = 1000;
int AntimissilesCount = 3;
std::vector<coordinates::Coordinates> Antimissiles;
double GenerateCoordinate()
{
	return (double)(MinCoordinate + rand() % (MaxCoordinate - MinCoordinate + 1));
}
double ComputeDistance(coordinates::Coordinates c1, coordinates::Coordinates c2)
{
	return sqrt(pow(c1.x-c2.x, 2)+pow(c1.y-c2.y, 2));
}
void Defende(const coordinates::Coordinates &destination)
{
	ROS_INFO("Missile was detected. The destination is (%f; %f)", destination.x, destination.y);
	for(int i=0; i<Antimissiles.size(); i++)
	{
		if(ComputeDistance(Antimissiles[i], destination) <= AntimissileRadius)
		{
			if(rand()%10 <= 8)
			{
				ROS_INFO("Missile was shot down by antimissile %i", i+1);
				return;
			}
		}
	}
	if(rand()%100 <= 5)
	{
		ROS_INFO("Missile was shot down without use antimissile");
		return;
	}
	ROS_INFO("Missile was not shot down");
}
void PlaceAntimissiles()
{
	for(int i=0; i<AntimissilesCount; i++)
	{
		coordinates::Coordinates antimissile;
		antimissile.x=GenerateCoordinate();
		antimissile.y=GenerateCoordinate();
		Antimissiles.push_back(antimissile);
		ROS_INFO("Antimissile %i was placed at coordinates (%f; %f)", i+1, antimissile.x, antimissile.y);
	}
}
int main(int argc, char **argv)
{
	init(argc, argv, "defender");
	srand(time(NULL));
	PlaceAntimissiles();
	NodeHandle n;
	Subscriber sub=n.subscribe("Missiles", 1000, Defende);
	spin();
	return 0;
}
