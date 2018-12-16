#include "robot.h"

#define DISPERSION 10
#define COUNT_FRAME 30


class LostRobot: public Robot{
    protected:

        enum Status {
            LOST,
            ROTATE_TO_FRIEND,
            GO_TO_FRIEND
        };

        bool isGoToFriend;
        float randomAngularFactor;
        float randomLinearFactor;
        float randomDirectionY;
        int countFrame;
        Status status;
        void updateRandomDirection();

    public:
        LostRobot(string name, string fn, int id, float x, float y, float speedLinear, float speedAngular, float min_dist):   Robot(name, fn, id, x, y, speedLinear, speedAngular, min_dist), 
                                                                                                    isGoToFriend(false), countFrame(0)
        {
            status = Status::LOST;
        };
        virtual void run();
};