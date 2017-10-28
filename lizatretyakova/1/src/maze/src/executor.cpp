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
    {X, O, X, O, O, O, O, X, O, X},
    {X, O, X, O, O, O, O, X, O, X},
    {X, O, X, O, X, O, O, X, O, X},
    {X, O, X, X, X, X, X, X, O, X},
    {X, O, O, O, O, O, O, O, O, X},
    {X, X, X, X, X, X, X, X, X, X} 
};
int cur_i = 1;
int cur_j = 1;

void clear_screen() {
    std::cout << "\033[2J\033[1;1H";
}

void draw_maze() {
    for(int i = 0; i < maze.size(); ++i) {
        for(int j = 0; j < maze[i].size(); ++j) {
            switch(maze[i][j]) {
            case X:
                std::cout << "\033[1;33m" << maze[i][j] << "\033[0m";
                break;
            case O:
                std::cout << "\033[1;37m" << maze[i][j] << "\033[0m";
                break;
            case T:
                std::cout << "\033[1;31m" << maze[i][j] << "\033[0m";
                break;
            }
        }
        std::cout << std::endl;
    }
}

void move(const std_msgs::Int8MultiArray::ConstPtr& msg) {
    int new_i = cur_i + msg->data[0];
    int new_j = cur_j + msg->data[1];

    if (new_i < 0 || new_j < 0
        || new_i >= maze.size() || new_j >= maze[0].size()
        || maze[new_i][new_j] == X) {
        ROS_INFO("No Way There");
        return;
    }

    ROS_INFO("Ok");
    
    maze[cur_i][cur_j] = O;
    maze[new_i][new_j] = T;
    clear_screen();
    draw_maze();

    cur_i = new_i;
    cur_j = new_j;
    return;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "executor");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("maze_msg_bus", 1000, move);
    ros::spin();
    return 0;
}
