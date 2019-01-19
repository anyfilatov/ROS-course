#include <csignal>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "robot_utilis.h"

RobotCar* rescure_robot_gazebo;


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rescure_robot_gazebo");

    ros::NodeHandle node;
    ros::Publisher pub = node.advertise<std_msgs::String>("found_topic", 10);
    ros::Rate rate(40);

    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;
    tf::StampedTransform lost_transform;
    tf::Transform transform;

    bool found = false;

    float x = 0.0;
    float y = 0.0;

    rescure_robot_gazebo = new Robot(n, 40, "rescure_robot_gazebo", "/root/.gazebo/models/pioneer3at/model-1_4.sdf", x, y);

    while (ros::ok())
    {
        transform.setOrigin(tf::Vector3(x, y, 0.0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "rescure_robot_gazebo"));
        
        if (!found)
        {
            try
            {
                tf_listener.waitForTransform("rescure_robot_gazebo", "lost_robot_gazebo", ros::Time(0), ros::Duration(0.5));
                tf_listener.lookupTransform("rescure_robot_gazebo", "lost_robot_gazebo", ros::Time(0), lost_transform);
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
                pub.publish(msg);
            }
            else
            {
                // go to lost
                if (fabs(lost_x) >= fabs(lost_y))
                {
                    if (lost_x > 0)
                        x += 1;
                    else
                        x -= 1;
                }
                else
                {
                    if (lost_y > 0)
                        y += 1;
                    else
                        y -= 1;
                }
            }

        }else
        {
            if (x != 0 || y != 0)
            {
                // go to finish point
                if (fabsf(0.0 - x) >= fabsf(0.0 - y))
                    x += ((0.0 - x) / fabsf(0.0 - x));
                else
                    y += ((0.0 - y) / fabsf(0.0 - y));
            }
            
        }
        
        rescure_robot_gazebo->move(x,y);
        rate.sleep();
        ros::spinOnce();
   
    }

    return 0;
}















