#include "ros/ros.h"
#include "lab2_msg/Msg.h"

uint32_t cm = 0;
uint32_t count_msgs = 10;


void handler(const lab2_msg::Msg & msg){
    cm++;
    if (*(msg.text).rbegin() % 2 == 0)
        ROS_INFO("Msg: \"%s\" is true", msg.text.c_str());
    else 
        ROS_INFO("Msg: \"%s\" is false", msg.text.c_str());
    if (cm == count_msgs) ros::shutdown();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "Accomplice");
    ROS_INFO("I get information!\n");
    ros::NodeHandle nh;
    ros::Subscriber sub_accomplice = nh.subscribe("Talk", count_msgs, handler);
    ros::spin();
	return 0;
}	