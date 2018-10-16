#include "ros/ros.h"
#include "lab2_msg/msgnumberone.h"
#include "lab2_msg/msgnumbertwo.h"
#include "std_msgs/String.h"
#include "iostream"
#include "fstream"



void chatterloc(const lab2_msg::msgnumberone& locmsg)
{
  std::cout<< "Oh i see: " << locmsg.location<< std::endl;
  
}

void chattersec(const lab2_msg::msgnumbertwo& secmsg)
{
  std::cout<< "Got it: " << secmsg.meettime<< std::endl;
  if(secmsg.count %2 == 0)
    {
    std::cout<< "Good to know "<< std::endl;
    }
  else
    {
    std::cout<< "I'm already know it "<< std::endl;
    }
  
}

 

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber fsub = n.subscribe("firstchatter", 1000, chatterloc);
  ros::Subscriber ssub = n.subscribe("secondchatter", 1000, chattersec);
  ros::spin();

  return 0;
}
