#include "ros/ros.h"
#include "lab2_msg/my_message.h"
#include <cstdlib>
#include <string>
#include <sstream>

std::string intToString(int num);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "publisher_node");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<lab2_msg::my_message>("topic", 100);
    ros::Rate sleep_rate(1);

    lab2_msg::my_message msg;

    while (ros::ok())
    {
        int x = rand() % 100;
        int y = rand() % 100;

        std::string x_str = intToString(x);
        std::string y_str = intToString(y);

        for (int i = 0; (i < 2 && x_str[i] != '\0'); i++)
            x_str[i] = x_str[i] + 42;

        for (int i = 0; (i < 2 && y_str[i] != '\0'); i++)
            y_str[i] = y_str[i] + 42;

        ROS_INFO_STREAM("x: " << x << "; Encrypted: " << x_str);
        ROS_INFO_STREAM("y: " << y << "; Encrypted: " << y_str);
        ROS_INFO_STREAM("");
        
        msg.msg = x_str + " " + y_str;

        publisher.publish(msg);

        sleep_rate.sleep();    
    }

    ros::spinOnce();
    return 0;
}

std::string intToString(int num)
{
    std::ostringstream ss;
    ss << num;
    return ss.str();
}
