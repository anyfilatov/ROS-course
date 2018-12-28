#include "../robot.h"

#define DISPERSION 10
#define COUNT_FRAM 30

class TemplarRobot: public Robot{
    protected:

        enum Stage{
            FIND_CAMP,
            GO_TO_CAMP,
            HIT_CAMP,
            HIT_ZERG,
            DEATH
        };

        bool isGoToFriend;
        float homeX;
        float homeY;
        Stage stage;
        float cooldownHit = 0;
        ros::Publisher pubHitCamp;
        ros::Publisher pubHitZerg;
        ros::Subscriber subHit;
        ros::Publisher pubDeath;
        void handlerHit(const std_msgs::Empty& msg);

    public:
        TemplarRobot(string name, float cooldownHit, int id, float r, float g, float b, float x, float y, float speed, float min_dist);
        virtual void run();
};