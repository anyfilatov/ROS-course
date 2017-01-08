#include <ros/ros.h>
#include "room.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "room");
    ros::NodeHandle nodeHandle;

    Room room(nodeHandle);
    room.waitAllLeaveRoom();

    ros::spin();
    
    return 0;
}
