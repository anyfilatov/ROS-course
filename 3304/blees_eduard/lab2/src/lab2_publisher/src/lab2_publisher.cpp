#include <string>
#include <fstream>
#include <ctime>
#include "ros/ros.h"
#include "ros/package.h"
#include "lab2_msg/message.h"

std::string random_string( std::size_t length )
{
    static const std::string alphabet = "abcdefghijklmnopqrstuvwxyz" ;

    std::string str;
    std::srand(unsigned(std::time(0)));
    while( str.size() < length )
    {
	int randNum = std::rand();
	str += alphabet[ randNum % alphabet.length() ];
    }
    return str ;
}

std::string get_msg(int id)
{
    if (id % 15 == 0)
    {
	return "message!";
    } else if (id % 10 == 0)
    {
	return "important";
    } else if (id % 5 == 0)
    {
	return "very";
    } else
    {
	return random_string(10);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lab2_publisher");
    ROS_INFO_STREAM("lab2_publisher init");
    ros::NodeHandle node_handle;
    ros::Publisher pub = node_handle.advertise<lab2_msg::message>("/highly_secret_messages", 100);
    lab2_msg::message msg;
    int count = 0;

    std::ifstream fs;

    ros::Rate rate(1);
    while (ros::ok())
    {
        msg.msg = get_msg(++count);
        msg.id = count;
        pub.publish(msg);
        ROS_INFO_STREAM("I said: " << msg.msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
