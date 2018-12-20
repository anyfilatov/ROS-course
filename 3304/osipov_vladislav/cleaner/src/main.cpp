#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include "cleaner.h"

int command = 0;

void callCommand(const std_msgs::Int16::ConstPtr& msg)
{
    command = msg->data;
}

int main(int argc, char** argv)
{
    gazebo::client::setup(argc, argv);
    ros::init(argc, argv, "alg");
    ros::NodeHandle nodeHandle;
    ros::Subscriber sub = nodeHandle.subscribe("scout", 1000, callCommand);

    init( "cleaner" );

    ros::Rate rate(10);

    while (ros::ok() && command != 1)
    {
        switch (command)
        {
        case 0:
            execute();
            break;
        case 1:
            removeModel("cleaner");
            break;
        default:
            ROS_INFO("Robot paused!");
            break;
        }
        gazebo::common::Time::MSleep(100);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("END cleaner");
    gazebo::shutdown();
    return 0;
}