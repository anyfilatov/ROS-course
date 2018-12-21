#include <csignal>
#include "ros/ros.h"
#include "base.h"
#include "enemy.h"

Enemy* enemy = nullptr;
Base* base = nullptr;

void signalHandler(int signum)
{
    ROS_INFO("TERMINATED");

    if (enemy != nullptr)
    {
        delete enemy;
    }

    if (base != nullptr)
    {
        delete base;
    }

    ros::shutdown();
}

int main(int argc, char **argv)
{
    signal(SIGINT, signalHandler);

    ros::init(argc, argv, "base", ros::init_options::NoSigintHandler);
    ros::NodeHandle n;

    enemy = new Enemy(n, 50, "enemy");
    enemy->spawnModel("/home/pr3sto/.gazebo/models/dumpster/model-1_4.sdf", 0, 0, 50);
    enemy->move(0, 0, 0, 10000);
    enemy->startShooting();

    base = new Base();
    base->defendPlanet();

    ros::spin();
    return 0;
}
