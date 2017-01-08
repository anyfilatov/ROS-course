#include "ros/ros.h"
#include "turtle.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_lab1");

    ros::NodeHandle nodeHandle;
    Turtle turtle(nodeHandle);

    while(ros::ok()) {
        turtle.move(1.0, 1.0);
        turtle.move(10.0, 1.0);
        turtle.move(1.0, 10.0);
        turtle.move(10.0, 10.0);
    }

    ros::spin();

    return 0;
}
