#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <random>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "publisher_node");

    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

    std::uniform_real_distribution<double> linear_interval(0, 1);
    std::uniform_real_distribution<double> angular_interval(0, M_PI);
    std::default_random_engine random_engine;

    ros::Rate rate(1);

    while (ros::ok()) {
        geometry_msgs::Twist message;
        message.linear.x = linear_interval(random_engine);
        message.linear.y = linear_interval(random_engine);
        message.linear.z = linear_interval(random_engine);
        
        message.angular.x = angular_interval(random_engine);
        message.angular.y = angular_interval(random_engine);
        message.angular.z = angular_interval(random_engine);

        publisher.publish(message);
        rate.sleep();
    }

    ros::spinOnce();

    return 0;
}