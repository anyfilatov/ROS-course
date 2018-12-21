#include "base.h"
#include "enemy.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "base");
    ros::NodeHandle n;

    Enemy enemy(n, 50, "enemy");
    enemy.spawnModel("/home/pr3sto/.gazebo/models/dumpster/model-1_4.sdf", 0, 0, 100);
    enemy.move(0, 0, 0, 10000);
    enemy.startShooting();

    Base base;
    base.defendPlanet();

    ros::spin();
    return 0;
}
