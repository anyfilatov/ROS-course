#ifndef BASE_H
#define BASE_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "robot.h"
#include <tuple>
#include <vector>
#include <string>
#include <iostream>
#include <unistd.h>

class Base
{
private:
	static const int height = 4;
	static const int gridDimention = 5;
	static const int pauseMicroseconds = 200000;

	enum cellStatus
	{
		UNSET,
		INPROCESS,
		FILLED
	};

	int counter = 0;
    cellStatus grid[gridDimention][gridDimention];
	std::vector<Robot*> robots;
	ros::NodeHandle n;	
public:
    Base()
    {
	setGrid();
    }

    ~Base()
    {
    }

    void defendPlanet()
    {
		while (!isGridDone())
		{
			update();
			auto cell = getFreePosition();

			if (std::get<0>(cell) == -1)
				continue;
			auto robotName = spawnRobot(cell);
			grid[std::get<0>(cell)][std::get<1>(cell)] = cellStatus::INPROCESS;

			ros::Duration(1, 0).sleep();
			usleep(pauseMicroseconds);
		}
    }
private:
	std::string spawnRobot(std::tuple<short, short> cell)
	{
		counter++;
		Robot* robot = new Robot(n, 50, "robot" + std::to_string(counter));
		robot->spawnModel("/home/ed/.gazebo/models/quadrotor/model-1_4.sdf", 0, 0, 0);
		robot->move(cellIndexToCoordinate(std::get<0>(cell)), cellIndexToCoordinate(std::get<1>(cell)), height);
		robots.push_back(robot);
		return robot->getName();
	}

	void update()
	{
		for(auto const& robot: robots)
		{
			auto status = robot->getStatus();
			switch (status) 
			{
				case Robot::status::SET:
					grid[coordinateToCellIndex(robot->getInitialX())][coordinateToCellIndex(robot->getInitialY())] = cellStatus::FILLED;
					break;
				case Robot::status::STOPPED:
					grid[coordinateToCellIndex(robot->getInitialX())][coordinateToCellIndex(robot->getInitialY())] = cellStatus::UNSET;
					break;
			}
		}
	}

	std::tuple<short, short> getFreePosition()
	{
		for (int i = 0; i < gridDimention; i++)
		{
			for (int j = 0; j < gridDimention; j++)
			{
				if (grid[i][j] == cellStatus::UNSET)
				{
					return std::make_tuple(i, j);
				}
			}
		}
		return std::make_tuple(-1, -1);
	}

	void setGrid()
	{
		for (int i = 0; i < gridDimention; i++)
		{
			for (int j = 0; j < gridDimention; j++)
			{
				grid[i][j] = cellStatus::UNSET;
			}
		}
	}

	bool isGridDone()
	{
		for (int i = 0; i < gridDimention; i++)
		{
			for (int j = 0; j < gridDimention; j++)
			{
				if (grid[i][j] != cellStatus::FILLED)
				{
					return false;
				}
			}
		}
		return true;
	}

	int cellIndexToCoordinate(short cellIndex)
	{
		// cellIndex - 1; if gridDimention == 3
		return cellIndex - 2;
	}

	short coordinateToCellIndex(int coordinate)
	{
		// (short)(coordinate + 1); if gridDimention == 3
		return (short)(coordinate + 2);
	}
};

#endif // BASE_H
