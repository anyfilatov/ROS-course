#include "ros/ros.h"
#include "std_msgs/Header.h"
#include <string>

int main(int argc, char** argv){
  ros::init(argc, argv, "default_name");
  ros::NodeHanlde nh;
  ros::Publisher pub = nh.advertise<std_msgs::Header>("topic",10);
  for (int i = 0; i < 10; i++) {
    std_msgs::Header msg;
    msg.frame_id = "Message: " + std::string(i);
    msg.seq = 100-i;
    pub.publish(msg);
  }
  return 0;
}
