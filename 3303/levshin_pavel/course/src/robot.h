
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"

#include <string>
#include <cmath>

#define FPS 100
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
    visualization_msgs::Marker markerText;
    tf::StampedTransform myFriend;

    NodeHandle nh;
    Publisher pub;
    Publisher pubGameOver;
    Subscriber subGameOver;
    bool isExecute = true;
    double time = 0;
    const double deltaTime = 1/(double)FPS;
    tf::TransformBroadcaster br;
    tf::TransformListener listener;

    void moveTo(float x, float y);
    bool isNearThePoint(float x, float y);
    bool isFriendReached();
    tf::Transform getTransform(string name);
    void saveTransform();
    ros::V_string getNodesNameByPattern(string pattern);
    void handlerGameOver(const std_msgs::String& msg);;
    void deleteMarker(); 

  public:
    Robot(string name, string topic, int id, int32_t markerType, tf::Vector3 position, tf::tfVector4 rotation, tf::Vector3 scale, tf::tfVector4 color, float speed, float min_dist);
    virtual void run() = 0;

};