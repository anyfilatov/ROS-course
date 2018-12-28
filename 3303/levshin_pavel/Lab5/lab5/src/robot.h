
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "gazebo_msgs/SetModelState.h"

#include <string>
#include <cmath>

#define FPS 100
//#define MIN_DISTANCE 0.5f
#define BASE_WORLD "odom"

using namespace std;
using namespace ros;

class Robot
{
  protected:
    string name;
    string friendName;
    float speedLinear;
    float currSpeedLinear;
    float speedAngular;
    float currSpeedAngular;
    float min_dist;
    tf::Transform myPosition;
    tf::Transform myFriend;

    NodeHandle nh;
    Publisher pub;

    tf::TransformListener listener;

    void move(float linear_x, float angular_z);
    bool isNearThePoint(float x, float y, float eps);
    bool isFriendReached();
    tf::Transform getTransform(string name);
    float calculateRotation(tf::Transform q_to,tf::Transform q_from);
    float toEulerAngle(tf::Transform q);
    float getAlfa(tf::Transform q_to,tf::Transform q_from);
  public:
    Robot(string name, string fn, int id, float x, float y, float speedLinear, float speedAngular, float min_dist);
    virtual void run() = 0;

};