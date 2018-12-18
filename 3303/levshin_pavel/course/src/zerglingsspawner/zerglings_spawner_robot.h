#include "../robot.h"

#define DISPERSION 10
#define COUNT_FRAME 30

class ZerglingSpawnerRobot: public Robot{
    protected:
        int countFrame;
        int health;
        uint countUnits = 0;
        ros::Subscriber subHit;
        ros::Subscriber subDeath;
        float cooldownSpawn;
        V_string zerglings;
        string genRunString(int id_home, int id_unit, float x, float y);
        void spawnUnit(int id_home, float x, float y);
        void handlerHit(const std_msgs::Empty& msg);
        void handlerDeath(const std_msgs::String& msg);
        tf::Vector3 getNoCollisionPosition();

    public:
        ZerglingSpawnerRobot(int id, int health, float cooldownSpawn, tf::Vector3 position, tf::tfVector4 rotation, tf::Vector3 scale, tf::tfVector4 color, float speed, float min_dist);
        virtual void run();
};