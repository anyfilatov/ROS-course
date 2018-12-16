#include "robot.h"

#define DISPERSION 10
#define COUNT_FRAM 30

class RescuerRobot: public Robot{
    protected:

        enum Status {
            ROTATE_TO_FRIEND,
            FIND_FRIEND,
            ROTATE_TO_HOME,
            GO_TO_HOME
        };

        bool isGoToFriend;
        tf::Transform home;
        Status status;

    public:
        RescuerRobot(string name, string fn, int id, float x, float y, float speedLinear, float speedAngular, float min_dist): Robot(name, fn, id, x, y, speedLinear, speedAngular, min_dist), 
                                                                                                     isGoToFriend(true)
        {
            tf::Transform transform;
	        transform.setOrigin(tf::Vector3(x, y, 0));
	        tf::Quaternion q;
	        q.setRPY(0, 0, 0);
	        transform.setRotation(q);
            status = Status::ROTATE_TO_FRIEND;
        };
        virtual void run();
};