
#include "ros/ros.h"
#include "message/my_msg.h"
#include <string>

void callback(const message::my_msg& msg);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "subscriber");
    ros::NodeHandle n;
    ros::Subscriber subscriber = n.subscribe("topic", 100, &callback);
    ros::spin();
    return 0;
}

void callback(const message::my_msg& msg)
{
    ROS_INFO_STREAM("msg: " << msg.msg);

    std::string x_str;
    std::string y_str;

    size_t pos = msg.msg.find(" ");
    x_str = msg.msg.substr(0, pos);
    y_str = msg.msg;
    y_str.erase(0, pos + 1);

    ROS_INFO_STREAM(x_str << "  " << y_str);

    int x = atoi(x_str.c_str());
    int y = atoi(y_str.c_str());

	x = x - 7;
	y = y - 8;

    ROS_INFO_STREAM("x: " << x << "y: " << y);
    ROS_INFO_STREAM("");
}
