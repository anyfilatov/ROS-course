#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

ros::Publisher pub;

void move(float speed, float distance, bool forward = true);
void rotate(float anglular_speed, float angle, bool clockwise = true);

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lab1_node");
    ros::NodeHandle node_handle;
    pub = node_handle.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

    ROS_INFO("Lab1 execution is started");
    while (ros::ok())
    {
        move(0.1, 1);
        rotate(0.1, M_PI_2, false);
    }
    ROS_INFO("Lab1 execution is ended");

    return 0;
}

void move(float speed, float distance, bool forward)
{
    geometry_msgs::Twist msg;
    msg.linear.x = (forward ? fabs(speed) : -fabs(speed));

    double curr_distance = 0.0;
    ros::Rate rate(100);
    double t0 = ros::Time::now().toSec();
    while (curr_distance <= distance && ros::ok())
    {
        pub.publish(msg);
        double t1 = ros::Time::now().toSec();
        curr_distance = speed * (t1 - t0);
        rate.sleep();
    }
}

void rotate(float anglular_speed, float angle, bool clockwise)
{
    geometry_msgs::Twist msg;
    msg.angular.z = (clockwise ? -fabs(anglular_speed) : fabs(anglular_speed));

    double curr_angle = 0.0;
    ros::Rate rate(100);
    double t0 = ros::Time::now().toSec();
    while (curr_angle <= angle && ros::ok())
    {
        pub.publish(msg);
        double t1 = ros::Time::now().toSec();
        curr_angle = anglular_speed * (t1 - t0);
        rate.sleep();
    }
}