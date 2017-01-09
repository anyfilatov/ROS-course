#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

using namespace ros;
using geometry_msgs::Twist;

int  main (int argc, char **argv) {
    init(argc, argv, "commander");
    NodeHandle handler;

    Publisher vel_publisher = handler.advertise<Twist>("/turtle1/cmd_vel", 1000);
    Rate loop_rate(5);
    
    float speed_x = 0, speed_z = 10;

    while (ok()) {
        Twist message;
        
        if (speed_x > 10) {
        	speed_x = 0;
        }
        if (speed_z == 0) {
        	speed_z = 10;
        }

        message.linear.x = ++speed_x;
        message.linear.y = 0;
        message.linear.z = --speed_z;
        message.angular.x = 1;
        message.angular.y = 2;
        message.angular.z = 2;

        vel_publisher.publish(message);
        spinOnce();
        loop_rate.sleep();
    }
}
