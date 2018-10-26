#include "ros/ros.h"
#include "lab2_msg/my_message.h"
#include <string>

void callback(const lab2_msg::my_message& msg);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "reader_node");
    ros::NodeHandle n;
    ros::Subscriber subscriber = n.subscribe("topic", 100, &callback);
    ros::spin();
    return 0;
}

void callback(const lab2_msg::my_message& msg)
{
    ROS_INFO_STREAM("msg: " << msg.msg);

    std::string x_str;
    std::string y_str;

    size_t pos = msg.msg.find(" ");
    x_str = msg.msg.substr(0, pos);
    y_str = msg.msg;
    y_str.erase(0, pos + 1);

    for (int i = 0; (i < 2 && x_str[i] != '\0'); i++)
        x_str[i] = x_str[i] - 42;

    for (int i = 0; (i < 2 && y_str[i] != '\0'); i++)
        y_str[i] = y_str[i] - 42;

    int x = atoi(x_str.c_str());
    int y = atoi(y_str.c_str());

    ROS_INFO_STREAM("x: " << x);
    ROS_INFO_STREAM("y: " << y);
    ROS_INFO_STREAM("");
}
