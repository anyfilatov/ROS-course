#include "robot.h"

#define DISPERSION 10
#define COUNT_FRAM 30

class RescuerRobot: public Robot{
    protected:
        bool isGoToFriend;
        float homeX;
        float homeY;

    public:
        RescuerRobot(string name, string fn, int id, float r, float g, float b, float x, float y, float speed, float min_dist): Robot(name, fn, id, r,g, b, x, y, speed, min_dist), 
                                                                                                     isGoToFriend(true), homeX(x), homeY(y){};
        virtual void run();
};