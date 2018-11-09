
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <cmath>

#define FPS 30
//#define MIN_DISTANCE 0.5f
#define BASE_WORLD "world"

using namespace std;
using namespace ros;

class Robot
{
  protected:
    string name;
    string friendName;
    float speed;
    float min_dist;
    
    visualization_msgs::Marker marker;
    tf::StampedTransform myFriend;

    NodeHandle nh;
    Publisher pub;

    tf::TransformBroadcaster br;
    tf::TransformListener listener;

    void moveTo(float x, float y);
    bool isNearThePoint(float x, float y);
    bool isFriendReached();
    void getFriendTransform();
    void saveTransform();
  public:
    Robot(string name, string fn, int id, float r, float g, float b, float x, float y, float speed, float min_dist);
    virtual void run() = 0;

};