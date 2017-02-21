#ifndef CLEANER_CONTROLLER_H
#define CLEANER_CONTROLLER_H

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int16.h>

/**
 * Control commands.
 */
enum Command {
    NOTHING,
    PAUSE = 'p',
    CONTINUE = 'c',
    ABORT = 'a'
};

using ros::NodeHandle;
using ros::Publisher;
using ros::Rate;
using ros::init;
using ros::spinOnce;

using std_msgs::Int16;

using std::string;
using std::cin;
using std::cout;
using std::endl;

short readCommand();

#endif
