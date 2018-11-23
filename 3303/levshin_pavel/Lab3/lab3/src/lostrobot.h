#include "robot.h"

#define DISPERSION 10
#define COUNT_FRAME 30

class LostRobot: public Robot{
    protected:
        bool isGoToFriend;
        float randomDirectionX;
        float randomDirectionY;
        int countFrame;

        void updateRandomDirection();

    public:
        LostRobot(string name, string fn, int id, float r, float g, float b, float x, float y, float speed, float min_dist):   Robot(name, fn, id, r,g, b, x, y, speed, min_dist), 
                                                                                                    isGoToFriend(false), countFrame(0){};
        virtual void run();
};