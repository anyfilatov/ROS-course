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
#include <algorithm>
#include <random>

class Base
{
private:
	static const int height = 4;
	static const int gridDimention = 3;
	static const int pauseMicroseconds = 1500000;

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
		robot->move(cellIndexToCoordinate(std::get<0>(cell)), cellIndexToCoordinate(std::get<1>(cell)), height, 1000);
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
		std::vector<std::tuple<short, short> > unsetCells;
		std::vector<std::tuple<short, short> > processCells;
		std::vector<std::tuple<short, short> > filledCells;

		for (int i = 0; i < gridDimention; i++)
		{
			for (int j = 0; j < gridDimention; j++)
			{
				switch (grid[i][j])
				{
					case cellStatus::UNSET:
						unsetCells.push_back(std::make_tuple(i, j));
						break;
					case cellStatus::INPROCESS:
						processCells.push_back(std::make_tuple(i, j));
						break;
					case cellStatus::FILLED:
						filledCells.push_back(std::make_tuple(i, j));
						break;
				}
			}
		}

		if (!unsetCells.empty())
		{
			auto rng = std::default_random_engine {};
			std::shuffle(std::begin(filledCells), std::end(filledCells), rng);
			std::shuffle(std::begin(processCells), std::end(processCells), rng);
			for(auto const& unsetCell: unsetCells)
			{
				if (!filledCells.empty())
				{
					for(auto const& filledCell: filledCells)
					{
						int distance = std::get<0>(filledCell) + std::get<1>(filledCell) - std::get<0>(unsetCell) - std::get<1>(unsetCell);
						if (std::abs(distance) == 1) 
							return std::make_tuple(std::get<0>(unsetCell), std::get<1>(unsetCell));
					}
				}
				if (!processCells.empty())
				{
					for(auto const& processCell: processCells)
					{
						int distance = std::get<0>(processCell) + std::get<1>(processCell) - std::get<0>(unsetCell) - std::get<1>(unsetCell);
						if (std::abs(distance) == 1) 
							return std::make_tuple(std::get<0>(unsetCell), std::get<1>(unsetCell));
					}
				}
			}
			std::shuffle(std::begin(unsetCells), std::end(unsetCells), rng);
			return std::make_tuple(std::get<0>(unsetCells[0]), std::get<1>(unsetCells[0]));
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
		return cellIndex - (gridDimention % 2 != 0 ? gridDimention - 1 : gridDimention)/2;
	}

	short coordinateToCellIndex(int coordinate)
	{
		return (short)((gridDimention % 2 != 0 ? gridDimention - 1 : gridDimention)/2 + coordinate);
	}
};

#endif // BASE_H
