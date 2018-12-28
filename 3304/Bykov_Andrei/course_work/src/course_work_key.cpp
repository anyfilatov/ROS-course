#include "ros/ros.h"
#include "std_msgs/Empty.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "course_work_key");

    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<std_msgs::Empty>("shooting_signal", 1000);

    ros::Rate rate(10);

    while (ros::ok())
    {
        system("clear");

        std::cout << "Enter 's' to shoot" << std::endl;
        std::cout << "Enter any other charecter to exit" << std::endl;

        char value;
        std::cin.clear();
        std::cin >> value;
        if(value == 's' || value == 'S') {
            ROS_INFO("Send shooting signal");

            std_msgs::Empty msg;

            pub.publish(msg);
        } else {
            ros::shutdown();
        }

        rate.sleep();
    }

    return 0;
}
