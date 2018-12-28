#include "../robot.h"

#define DISPERSION 10
#define COUNT_FRAME 30

class TemplarsSpawnerRobot: public Robot{
    protected:
        bool isGoToFriend;
        float randomDirectionX;
        float randomDirectionY;
        int countFrame;
        int countWaves;
        int countUnitInWave;
        int cooldownSpawn;
        uint countUnits = 0;
        V_string templars;
        ros::Subscriber subDeath;
        string genRunString(int id_home, int id_unit, float x, float y);
        void spawnUnit(int id_home, float x, float y);
        tf::Vector3 getNoCollisionPosition();
        ros::V_string getNodesNameByPattern(string patterm);
        void handlerDeath(const std_msgs::String& msg);

    public:
        TemplarsSpawnerRobot(int id, int countWaves, int countUnitInWave, int cooldownSpawn, tf::Vector3 position, tf::tfVector4 rotation, tf::Vector3 scale, tf::tfVector4 color, float speed, float min_dist);
        virtual void run();
};