#include "ros/ros.h"
#include "message/my_msg.h"
#include <cstdlib>
#include <string>
#include <sstream>

std::string intToString(int num);

int main(int argc, char *argv[])
{
	

    ros::init(argc, argv, "publisher_node");
    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<message::my_msg>("topic", 100);
    ros::Rate sleep_rate(1);

    message::my_msg msg;

    while (ros::ok())
    {
        int x = rand() % 100;
        int y = rand() % 100;

	ROS_INFO_STREAM("x: " << x << "y: " << y);

	std::string x_str = intToString(x+7)+"e12/v"+intToString(x+12)+"fgh5";
	std::string y_str = intToString(y+8)+"q1dgkg56";

        ROS_INFO_STREAM("Encrypted: " << x_str << " " << y_str);
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
