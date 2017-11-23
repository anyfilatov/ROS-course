#include "ros/ros.h"
#include "std_msgs/Int8MultiArray.h"

#include <termios.h>
#include <unistd.h>
#include <iostream>

int getch()
{
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

    int c = getchar();  // read character (non-blocking)

    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return c;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "driver");
    ros::NodeHandle n;
    ros::Publisher driver_pub = n.advertise<std_msgs::Int8MultiArray>("maze_msg_bus", 1000);

    while (ros::ok()) {
        // msg.data[0] := delta by x-axis
        // msg.data[1] := delta by y-axis
        
        while(true) {
            if (getch() == '\033') { // if the first value is esc
                getch(); // skip the [
                std_msgs::Int8MultiArray msg;
                switch(getch()) { // the real value
                    case 'A':
                        // arrow up
                        ROS_INFO("[arrow up]");
                        msg.data.push_back(-1);
                        msg.data.push_back(0);
                        break;
                    case 'B':
                        // arrow down
                        ROS_INFO("[arrow down]");
                        msg.data.push_back(1);
                        msg.data.push_back(0);
                        break;
                    case 'C':
                        // arrow right
                        ROS_INFO("[arrow right]");
                        msg.data.push_back(0);
                        msg.data.push_back(1);
                        break;
                    case 'D':
                        // arrow left
                        ROS_INFO("[arrow left]");
                        msg.data.push_back(0);
                        msg.data.push_back(-1);
                        break;
                }
                driver_pub.publish(msg);
                ros::spinOnce();
            }
        }
    }

    return 0;
}
