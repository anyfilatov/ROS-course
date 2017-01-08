#include "ros/ros.h"
#include "std_msgs/String.h"
#include "total_msg/totalMesg.h"
#include <sstream>

int main(int argc, char **argv)
{
  std::string speech[8]= {
 "Obey to our leader -- Obey to our leader -- Obey to our leader",
  "Meeting time:",
  "12.30",
  "Street:",
  "Rasstannaya",
  "Place:",
  "Bar",
  "end of the secret message"
  };
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<total_msg::totalMesg>("chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 1;
  int i=1;
  while (ros::ok())
  {
    total_msg::totalMesg msg;
    msg.x=count;
    std::stringstream ss;
    if (count%5!=0)
        ss << count<<": "<< speech[0];
    else
    {
        ss<<msg.x<<": "<< speech[i];
        i++;
        if(i==8) i=1;
    }

    msg.a=ss.str();
    ROS_INFO("%s", msg.a.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
