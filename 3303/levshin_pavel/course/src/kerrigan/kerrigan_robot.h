#include "../robot.h"

#define DISPERSION 10
#define COUNT_FRAM 30

class KerriganRobot: public Robot{
    protected:

        enum Stage{
            FIND_TEMPLAR,
            GO_TO_TEMPLAR,
            HIT_TEMPLER,
            HIT_ZERG,
            DEATH
        };

        bool isGoToFriend;
        float homeX;
        float homeY;
        Stage stage;
        float cooldownHit = 0;
        ros::Publisher pubHitTemplar;
        ros::Subscriber subHit;
        ros::Publisher pubDeath;
        void handlerHit(const std_msgs::Empty& msg);

    public:
        KerriganRobot(string name, float cooldownHit, int id, float r, float g, float b, float x, float y, float speed, float min_dist);
        virtual void run();
};