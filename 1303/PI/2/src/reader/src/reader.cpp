#include "ros/ros.h"
#include "std_msgs/String.h"
#include "message/location.h"

void reading(const message::location::ConstPtr &loc){
    ROS_INFO("RECIEVED MESSAGE");
    ROS_INFO("coordinates recieved: < %f, %f, %f > ", loc->x, loc->y, loc->z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reader");
    ros::NodeHandle nodeHandle;

    ros::Subscriber reader = nodeHandle.subscribe("coordinates", 1000, reading);
    ros::spin();
    
    return 0;
}