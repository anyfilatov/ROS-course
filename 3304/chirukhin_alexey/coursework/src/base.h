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
	std::vector<Robot> robots;
	ros::NodeHandle n;
public:
    Base()
    {
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
				break;
			auto robotName = spawnRobot(cell);
// maybe no need to return robotNAme from spawn
		}
/*
        Robot robot1(n, rate, "robot1");
	    robot1.spawnModel("/home/ed/.gazebo/models/quadrotor/model-1_4.sdf", 0, 0, 0);
	    robot1.move(10, 10, 20);

	    for (int i = 0; i < 100; i++)
		rate.sleep();

	    Robot robot2(n, rate, "robot2");
	    robot2.spawnModel("/home/ed/.gazebo/models/quadrotor/model-1_4.sdf", 0, 0, 0);
	    robot2.move(-10, 10, 20);

	    for (int i = 0; i < 200; i++)
		rate.sleep();

	    robot1.deleteModel();

	    for (int i = 0; i < 200; i++)
		rate.sleep();

	    robot2.deleteModel();
*/
    }
private:
	std::string spawnRobot(std::tuple<short, short> cell)
	{
		counter++;
		Robot robot(n, 50, "robot" + std::to_string(counter));
	    robot.spawnModel("/home/ed/.gazebo/models/quadrotor/model-1_4.sdf", 0, 0, 0);
	    robot.move(-10 + 10*std::get<0>(cell), -10 + 10*std::get<1>(cell), 20);
		robots.push_back(robot);
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
				if (grid[i][j] == cellStatus::UNSET)
				{
					return std::make_tuple(i, j);
				}
			}
		}
		return std::make_tuple(-1, -1);
	}

	bool isGridDone()
	{
		for (int i = 0; i < 3; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				if (grid[i][j] != cellStatus::FILLED)
				{
					return false;
				}
			}
		}
		return true;
	}
};

#endif // BASE_H
