#include <string>
#include <fstream>
#include "ros/ros.h"
#include "ros/package.h"
#include "lab2_msg/Sentence.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lab2_speaker_node");
    ros::NodeHandle node_handle;
    ros::Publisher pub = node_handle.advertise<lab2_msg::Sentence>("/topic_name", 1000);
    lab2_msg::Sentence msg;
    unsigned num = 0;

    std::ifstream fs;
    std::string str;
    fs.open((ros::package::getPath("lab2_speaker") + "/speech.txt").c_str(), std::fstream::in);
    if (!fs.is_open())
    {
        ROS_WARN_STREAM("Error while open file");
        return 0;
    }

    ROS_INFO_STREAM("Lab2's speaker started");
    ros::Rate rate(1);
    while (ros::ok() && std::getline(fs, str))
    {
        msg.sentence = str;
        msg.number = ++num;
        pub.publish(msg);
        ROS_INFO_STREAM("I've said: " << msg.sentence);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO_STREAM("Lab2's speaker finished");
    return 0;
}