#include <string>
#include "ros/ros.h"
#include "lab2_msg/Sentence.h"

void sentence_handler(const lab2_msg::Sentence &msg);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lab2_listener_node");
    ros::NodeHandle node_handle;
    ROS_INFO_STREAM("Lab2's listener started");
    ros::Subscriber sub = node_handle.subscribe("/topic_name", 1000, &sentence_handler);
    ros::spin();
    ROS_INFO_STREAM("Lab2's listener finished");
    return 0;
}

void sentence_handler(const lab2_msg::Sentence &msg)
{
    static unsigned n = 0;
    static std::string message;
    // ROS_INFO_STREAM("I've heard:" << msg.sentence);
    n++;
    if ((msg.number != 0) && (msg.number % 5 == 0))
    {
        message += msg.sentence + " ";
        ROS_INFO_STREAM("I've heard: " << msg.sentence);
        ROS_INFO_STREAM("Message: " << message);
    }
}