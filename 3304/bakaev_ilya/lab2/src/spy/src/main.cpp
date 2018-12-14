#include <string>
#include <ros/ros.h>
#include <scout_message/message.h>

void scout_callback_handler(const scout_message::message& message) {
    switch (message.valid % 2) {
    case 0:
        ROS_INFO_STREAM("lie: " << message.message);
        break;
    case 1:
        ROS_INFO_STREAM("sooth: " << message.message);
        break;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "spy");

    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber = nodeHandle.subscribe("/public_messages", 100, &scout_callback_handler);
    ros::spin();
    return 0;
}
