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
#include<functional>

class Base
{
private:
	enum cellStatus
	{
		UNSET,
		INPROCESS,
		FILLED
	};

	int counter = 0;
    cellStatus grid[3][3];
	std::vector<std::reference_wrapper<Robot>> robots;
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
			std::cout << "get position to spawn: " << std::get<0>(cell) << std::get<1>(cell) << std::endl;
			if (std::get<0>(cell) == -1)
				break;
			auto robotName = spawnRobot(cell);
			std::cout << "NEW ROBOT NAME: " << robotName << std::endl;
			ros::Rate rate(50);
			rate.sleep();
		}
    }
private:
	std::string spawnRobot(std::tuple<short, short> cell)
	{
		counter++;
		Robot* robot = new Robot(n, 50, "robot" + std::to_string(counter));
		robot->spawnModel("/home/ed/.gazebo/models/quadrotor/model-1_4.sdf", 0, 0, 0);
		robot->move(-10 + 10*std::get<0>(cell), -10 + 10*std::get<1>(cell), 20);
		robots.push_back(*robot);
		return robot->getName();
	}

	void update()
	{
	/*
		HERE:
		check all flying robots - if in destination - set his and cell status to SET
		check for messages from destroyer - if he destroyed ship - set status to DEAD, or remove from vector
	*/
	}

	std::tuple<short, short> getFreePosition()
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				std::cout << "CHECK grid_" << i << j << " = " << grid[i][j] << std::endl;
				std::cout << "CHECK values" << cellStatus::UNSET << " " << cellStatus::INPROCESS << " " << cellStatus::FILLED << std::endl;
				if (grid[i][j] == cellStatus::UNSET)
				{
					std::cout << "return some position" << std::endl;
					return std::make_tuple(i, j);
				}
			}
		}
		std::cout << "return shit position" << std::endl;
		return std::make_tuple(-1, -1);
	}

	bool isGridDone()
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				std::cout << "grid_" << i << j << " = " << grid[i][j] << std::endl;
				if (grid[i][j] != cellStatus::FILLED)
				{
					return false;
				}
			}
		}
		return true;
	}

	void setGrid()
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				grid[i][j] = cellStatus::UNSET;
			}
		}
	}
};

#endif // BASE_H
