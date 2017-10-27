#include "ros/ros.h"
#include "std_msgs/Int8MultiArray.h"

#include <unistd.h>
#include <iostream>
#include <vector>


const char X = 'X'; // wall
const char O = '.'; // space
const char T = 'T'; // turtle
std::vector<std::vector<char> > maze = {
    {X, X, X, X, X, X, X, X, X, X},
    {X, T, O, O, O, O, O, O, O, X},
    {X, O, X, O, O, O, O, X, O, X},
    {X, O, X, X, X, X, X, X, O, X},
    {X, O, O, O, O, O, O, O, O, X},
    {X, X, X, X, X, X, X, X, X, X} 
};
int i = 1;
int j = 1;

void move(const std_msgs::Int8MultiArray::ConstPtr& msg) {
    int new_i = i + msg->data[0];
    int new_j = j + msg->data[1];

    if (new_i < 0 || new_j < 0
        || new_i >= maze.size() || new_j >= maze[0].size()
        || maze[new_i][new_j] == X) {
        ROS_INFO("No Way There");
        return;
    }

    ROS_INFO("Ok");
    i = new_i;
    j = new_j;
    return;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "executor");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("maze_msg_bus", 1000, move);
    ros::spin();
    return 0;
}
