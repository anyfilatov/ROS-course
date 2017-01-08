#include "ros/ros.h"
#include "total_msg/totalMesg.h"


void chatterCallback(const total_msg::totalMesg& msg)
{
  if((msg.x)%5==0)
    ROS_INFO("The message is: [%s]", msg.a.c_str());
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();

  return 0;
}
