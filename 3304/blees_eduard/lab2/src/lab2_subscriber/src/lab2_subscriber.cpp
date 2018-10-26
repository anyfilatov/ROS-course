#include <string>
#include "ros/ros.h"
#include "lab2_msg/message.h"

void subscriberCallback(const lab2_msg::message &msg)
{
    static std::string fullMessage;
    static int n = 0;
    n++;
    if (msg.id == 0)
    {
        ROS_INFO_STREAM("excuse me wtf???");
    } 
    else if (msg.id % 5 == 0)
    {
        fullMessage += msg.msg + " ";
        ROS_INFO_STREAM("I heard: " << msg.msg);
        ROS_INFO_STREAM("Full message: " << fullMessage);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lab2_subscriber");
    ROS_INFO_STREAM("lab2_subscriber init");
    ros::NodeHandle node_handle;
    ros::Subscriber sub = node_handle.subscribe("/highly_secret_messages", 1000, &subscriberCallback);
    ros::spin();
    return 0;
}
