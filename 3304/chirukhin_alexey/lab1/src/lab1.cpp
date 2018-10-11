#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

void move(float speed, float distance, float rotation);
void rotate(float anglular_speed, float angle);

float to_radians(float degrees);

ros::Publisher publisher;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "publisher_node");

    ros::NodeHandle n;
    publisher = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

    for (int i = 0; i < 5; i++)
    {
        // move straight line
        move(1, 2, 0);
        if (i != 4)
        {
            // rotate 144 degrees with 100 deg/sec rotation speed
            rotate(to_radians(100), to_radians(144));
        }
    }
    // rotate 70 degrees with 100 deg/sec rotation speed
    rotate(to_radians(100), to_radians(70));
    while (ros::ok())
    {
        // move circular
        move(1, 2, to_radians(54));
    }

    return 0;
}

void move(float speed, float distance, float rotation)
{
    geometry_msgs::Twist msg;

    msg.linear.x = fabs(speed);
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = rotation;

    ros::Rate sleep_rate(100);
    double curr_distance = 0.0;
    double t0 = ros::Time::now().toSec();
    while (curr_distance <= distance)
    {
        publisher.publish(msg);

        double t1 = ros::Time::now().toSec();
        curr_distance = speed * (t1 - t0);

        sleep_rate.sleep();
    }
}

void rotate(float anglular_speed, float angle)
{
    geometry_msgs::Twist msg;

    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = fabs(anglular_speed);

    ros::Rate sleep_rate(100);
    double curr_angle = 0.0;
    double t0 = ros::Time::now().toSec();
    while (curr_angle <= angle)
    {
        publisher.publish(msg);

        double t1 = ros::Time::now().toSec();
        curr_angle = anglular_speed * (t1 - t0);

        sleep_rate.sleep();
    }
}

float to_radians(float degrees)
{
    return degrees * M_PI / 180.0;
}
