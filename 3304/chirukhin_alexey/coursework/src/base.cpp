#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "robot.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "base");

    ros::NodeHandle n;
    ros::Rate rate(50);

    Robot robot1(n, rate, "robot1");
    robot1.spawnModel("/home/pr3sto/.gazebo/models/quadrotor/model-1_4.sdf", 0, 0, 0);
    robot1.move(10, 10, 20);

    for (int i = 0; i < 100; i++)
        rate.sleep();

    Robot robot2(n, rate, "robot2");
    robot2.spawnModel("/home/pr3sto/.gazebo/models/quadrotor/model-1_4.sdf", 0, 0, 0);
    robot2.move(-10, 10, 20);

    for (int i = 0; i < 200; i++)
        rate.sleep();

    robot1.deleteModel();

    for (int i = 0; i < 200; i++)
        rate.sleep();

    robot2.deleteModel();

    ros::spin();
    return 0;
}
