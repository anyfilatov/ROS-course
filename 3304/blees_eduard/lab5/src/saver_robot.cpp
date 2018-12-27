#include <csignal>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "robot.h"
#include <iostream>

Robot* saver_robot;

void signalHandler(int signum)
{
    ROS_INFO("TERMINATED");

    if (saver_robot != NULL)
    {
        saver_robot->deleteModel();
        delete saver_robot;
    }

    exit(signum);
}

int main(int argc, char ** argv)
{
    signal(SIGINT, signalHandler);

    ros::init(argc, argv, "saver_robot", ros::init_options::NoSigintHandler);

    ros::NodeHandle n;
    ros::Publisher publisher_found_topic = n.advertise<std_msgs::String>("found_topic", 10);
    ros::Rate rate(60);

    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;
    tf::StampedTransform lost_transform;
    tf::Transform transform;

    bool found = false;

    float x = 0.0;
    float y = 0.0;

    saver_robot = new Robot(n, 120, "saver_robot", "/home/ed/.gazebo/models/pioneer3at/model-1_4.sdf", x, y);

    while(ros::ok())
    {
        transform.setOrigin(tf::Vector3(x, y, 0.0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "saver_robot"));

        if (!found)
        {
            try
            {
                tf_listener.waitForTransform("saver_robot", "lost_robot", ros::Time(0), ros::Duration(0.5));
                tf_listener.lookupTransform("saver_robot", "lost_robot", ros::Time(0), lost_transform);
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
		float absX = std::abs(lost_x);
		float absY = std::abs(lost_y);
		float len = std::sqrt(lost_x*lost_x + lost_y*lost_y);
		float dx = lost_x / len;
		float dy = lost_y / len;

		x += dx;
		y += dy;
            }
        }
        else
        {
            if (x != 0 || y != 0)
            {
		float len = std::sqrt(x*x + y*y);
		if (len > 1)
		{						
			float dx = x / len;
			float dy = y / len;

			x -= dx;
			y -= dy;
		}
		else 
		{
			x = 0;
			y = 0;
		}
            }
        }

        saver_robot->move(x, y);

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}

