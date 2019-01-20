#include <csignal>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <random>
#include "robot.h"

using namespace std;

void foundCallback(const std_msgs::String& msg);

bool found = false;

Robot* lost;

void signalHandler(int signum)
{
    ROS_INFO("TERMINATED");

    if (lost != NULL)
    {
        lost->deleteModel();
        delete lost;
    }

    exit(signum);
}

int main(int argc, char **argv)
{
    signal(SIGINT, signalHandler);

    ros::init(argc, argv, "lost", ros::init_options::NoSigintHandler);

    ros::NodeHandle n;
    ros::Subscriber subscriber_found_topic = n.subscribe("finder_topic", 100, foundCallback);
    ros::Rate rate(40);

    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;
    tf::StampedTransform tranformToFinder;
    tf::Transform transform;

    random_device rd;
    uniform_real_distribution<double> initialInterval(0, 50);
    uniform_real_distribution<double> movingInterval(-2, 2);

    srand(time(NULL));

    // random position inside 50x50 grid
    double x = initialInterval(rd); 
    double y = initialInterval(rd); 

    double finderX = 0.0;
    double finderY = 0.0;

    int wait_cycles = 40;

    lost = new Robot(n, 40, "lost", "/home/user/.gazebo/models/pioneer2dx/model.sdf", x, y);

    while(ros::ok())
    {
        transform.setOrigin(tf::Vector3(x, y, 0.0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "lost"));

        if (!found)
        {
            // move in random direction
            x += movingInterval(rd);
            y += movingInterval(rd);
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
                tf_listener.waitForTransform("lost", "finder", ros::Time(0), ros::Duration(0.5));
                tf_listener.lookupTransform("lost", "finder", ros::Time(0), tranformToFinder);
            }
            catch (tf::TransformException &ex)
            {
                rate.sleep();
                continue;
            }

            finderX = tranformToFinder.getOrigin().x();
            finderY = tranformToFinder.getOrigin().y();

            if (fabs(x) <= 1 && fabs(y) <= 1)
            {
                ROS_INFO("Rescued!");
            }
            else
            {
                // follow rescuer
                if (fabs(finderX) >= fabs(finderY))
                {
                    if (finderX > 0)
                        x += 3;
                    else
                        x -= 3;
                }
                else
                {
                    if (finderY > 0)
                        y += 3;
                    else
                        y -= 3;
                }
            }
        }

        lost->move(x, y);

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

void foundCallback(const std_msgs::String& msg)
{
    ROS_INFO_STREAM("Finder's message: " << msg.data);
    found = true;
}
