#include <csignal>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "robot.h"

void foundCallback(const std_msgs::String& msg);

bool found = false;

Robot* lost_robot;

void signalHandler(int signum)
{
    ROS_INFO("TERMINATED");

    if (lost_robot != NULL)
    {
        lost_robot->deleteModel();
        delete lost_robot;
    }

    exit(signum);
}

int main(int argc, char **argv)
{
    signal(SIGINT, signalHandler);

    ros::init(argc, argv, "lost_robot", ros::init_options::NoSigintHandler);

    ros::NodeHandle n;
    ros::Subscriber subscriber_found_topic = n.subscribe("found_topic", 100, foundCallback);
    ros::Rate rate(40);

    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;
    tf::StampedTransform rescuer_transform;
    tf::Transform transform;

    srand(time(NULL));

    // random position inside 50x50 grid
    float x = (-25) + rand() % 51; // from -25 to 25
    float y = (-25) + rand() % 51; // from -25 to 25

    float rescuer_x = 0.0;
    float rescuer_y = 0.0;

    int wait_cycles = 40;

    lost_robot = new Robot(n, 40, "lost_robot", "/home/pr3sto/.gazebo/models/pioneer3at/model-1_4.sdf", x, y);

    while(ros::ok())
    {
        transform.setOrigin(tf::Vector3(x, y, 0.0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "lost_robot"));

        if (!found)
        {
            // move in random direction
            int i = rand() % 2;
            int j = rand() % 2;

            if (i == 1 && j == 1)
                x += 1;
            else if (i == 1 && j == 0)
                x -= 1;
            else if (i == 0 && j == 1)
                y += 1;
            else if (i == 0 && j == 0)
                y -= 1;
        }
        else
        {
            // wait some time before following rescuer
            while (wait_cycles > 0)
            {
                rate.sleep();
                wait_cycles--;
            }

            try
            {
                tf_listener.waitForTransform("lost_robot", "rescuer_robot", ros::Time(0), ros::Duration(0.5));
                tf_listener.lookupTransform("lost_robot", "rescuer_robot", ros::Time(0), rescuer_transform);
            }
            catch (tf::TransformException &ex)
            {
                rate.sleep();
                continue;
            }

            rescuer_x = rescuer_transform.getOrigin().x();
            rescuer_y = rescuer_transform.getOrigin().y();

            if (fabs(x) <= 1 && fabs(y) <= 1)
            {
                ROS_INFO("Rescued!");
            }
            else
            {
                // follow rescuer
                if (fabs(rescuer_x) >= fabs(rescuer_y))
                {
                    if (rescuer_x > 0)
                        x += 1;
                    else
                        x -= 1;
                }
                else
                {
                    if (rescuer_y > 0)
                        y += 1;
                    else
                        y -= 1;
                }
            }
        }

        lost_robot->move(x, y);

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

void foundCallback(const std_msgs::String& msg)
{
    ROS_INFO_STREAM("Rescuer says: " << msg.data);
    found = true;
}
