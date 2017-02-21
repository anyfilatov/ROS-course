#ifndef CLEANER_CLEANER_H
#define CLEANER_CLEANER_H

#include <gazebo/msgs/laserscan_stamped.pb.h>
#include <tf/transform_listener.h>

#include "gazebo_model.h"

using std::cout;
using std::endl;
using std::mem_fun;

using tf::Quaternion;
using tf::Vector3;

using ros::Subscriber;

class Cleaner : GazeboModel {

public:
    Cleaner(string pathToModel, string name);
    void makeStep();

protected:
    static const float MOVE_STEP;
    static const float ROTATE_STEP;

    static bool inFrontObstacle;

    static void scannerHandler(ConstLaserScanStampedPtr &report);

    void moveForward(float distance);
    void rotate(float angleDegree);
    void execute(float &distance, float &angle);
};


#endif //CLEANER_CLEANER_H
