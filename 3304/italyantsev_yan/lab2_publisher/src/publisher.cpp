#include "ros/ros.h"
#include "std_msgs/String.h"
#include "lab2_msg/msgnumberone.h"
#include "lab2_msg/msgnumbertwo.h"
#include <sstream>
    
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher firstpub = n.advertise<lab2_msg::msgnumberone>("firstchatter", 1000);
  ros::Publisher secondpub = n.advertise<lab2_msg::msgnumbertwo>("secondchatter", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
    
  while (count<1)
  {
    lab2_msg::msgnumberone msg1;
    lab2_msg::msgnumbertwo msg2;
    std::cout << "Do you know where it is:\n" << "and what a time right now?:\n" << "*Secret number message*:\n";
    std::cin >> msg1.location >> msg2.meettime >> msg2.count;    
    std::cout<< "Message: " << msg1.location << " " << msg2.meettime << " " << msg2.count << std::endl;
    firstpub.publish(msg1);
    secondpub.publish(msg2);
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
