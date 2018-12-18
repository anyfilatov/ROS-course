#include "../robot.h"

#define DISPERSION 10
#define COUNT_FRAME 30

class ZerglingRobot: public Robot{
    protected:

        enum Stage {
            FIND_TEMPLAR,
            GO_TO_TEMPLAR,
            DEATH
        };

        bool isGoToFriend;
        float randomDirectionX;
        float randomDirectionY;
        int countFrame;
        Subscriber subHit;
        Publisher pubDeath;
        Subscriber subDeath;
        Stage stage;
        string currTemplar;

        void handlerHit(const std_msgs::Empty& msg);
        void handlerDeath(const std_msgs::String& msg);
    public:
        ZerglingRobot(string name, int id, float r, float g, float b, float x, float y, float speed, float min_dist);
        virtual void run();
};