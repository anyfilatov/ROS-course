#include <csignal>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "robot.h"

Robot* finder;

void signalHandler(int signum)
{
    ROS_INFO("TERMINATED");

    if (finder != NULL)
    {
        finder->deleteModel();
        delete finder;
    }

    exit(signum);
}

int main(int argc, char ** argv)
{
    signal(SIGINT, signalHandler);

    ros::init(argc, argv, "finder", ros::init_options::NoSigintHandler);

    ros::NodeHandle n;
    ros::Publisher publisher_found_topic = n.advertise<std_msgs::String>("finder_topic", 10);
    ros::Rate rate(40);

    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;
    tf::StampedTransform lost_transform;
    tf::Transform transform;

    bool found = false;

    float x = 0.0;
    float y = 0.0;

    finder = new Robot(n, 40, "finder", "/home/user/.gazebo/models/youbot/model.sdf", x, y);

    while(ros::ok())
    {
        transform.setOrigin(tf::Vector3(x, y, 0.0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "finder"));

        if (!found)
        {
            try
            {
                tf_listener.waitForTransform("finder", "lost", ros::Time(0), ros::Duration(0.5));
                tf_listener.lookupTransform("finder", "lost", ros::Time(0), lost_transform);
            }
            catch (tf::TransformException &ex)
            {
                rate.sleep();
                continue;
            }

            float lost_x = lost_transform.getOrigin().x();
            float lost_y = lost_transform.getOrigin().y();

            if (fabs(lost_x) < 3 && fabs(lost_y) < 3)
            {
                ROS_INFO("Found!");
                found = true;
                std_msgs::String msg;
                msg.data = std::string("Follow me!");
                publisher_found_topic.publish(msg);
            }
            else
            {
                // go to lost
                if (fabs(lost_x) >= fabs(lost_y))
                {
                    if (lost_x > 0)
                        x += 5;
                    else
                        x -= 5;
                }
                else
                {
                    if (lost_y > 0)
                        y += 5;
                    else
                        y -= 5;
                }
            }
        }
        else
        {
            if (x != 0 || y != 0)
            {
                // go to finish point
                if (fabsf(0.0 - x) >= fabsf(0.0 - y))
                    // x += ((0.0 - x) / fabsf(0.0 - x));
                    x -= 5;
                else
                    // y += ((0.0 - y) / fabsf(0.0 - y));
                    y -=5 ;
            }
        }

        finder->move(x, y);

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
