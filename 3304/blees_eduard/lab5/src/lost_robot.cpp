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
    ros::Rate rate(60);

    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;
    tf::StampedTransform rescuer_transform;
    tf::Transform transform;

    srand(time(NULL));

    float x = (-25) + rand() % 51;
    float y = (-25) + rand() % 51;

    float rescuer_x = 0.0;
    float rescuer_y = 0.0;

    int wait_cycles = 40;

    lost_robot = new Robot(n, 120, "lost_robot", "/home/ed/.gazebo/models/pioneer3at/model-1_4.sdf", x, y);

    while(ros::ok())
    {
        transform.setOrigin(tf::Vector3(x, y, 0.0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "lost_robot"));

        if (!found)
        {
		// move in random direction
		float i = ((float) rand() / (RAND_MAX)) * 2 - 1;
		float j = ((float) rand() / (RAND_MAX)) * 2 - 1;

		float len = std::sqrt(i*i + j*j);
		float dx = i / len;
		float dy = j / len;

		x += dx;
		y += dy;
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
                tf_listener.waitForTransform("lost_robot", "saver_robot", ros::Time(0), ros::Duration(0.5));
                tf_listener.lookupTransform("lost_robot", "saver_robot", ros::Time(0), rescuer_transform);
            }
            catch (tf::TransformException &ex)
            {
                rate.sleep();
                continue;
            }

            rescuer_x = rescuer_transform.getOrigin().x();
            rescuer_y = rescuer_transform.getOrigin().y();

            if (fabs(x) <= 3 && fabs(y) <= 3)
            {
                ROS_INFO("Saved!");
            }
            else
            {
		float len = std::sqrt(rescuer_x*rescuer_x + rescuer_y*rescuer_y);
		float dx = rescuer_x / len;
		float dy = rescuer_y / len;

		x += dx;
		y += dy;
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
    ROS_INFO_STREAM("Saver says: " << msg.data);
    found = true;
}

