#pragma once
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

void broadcast_pose(double x, double y, std::string base_frame_id, std::string child_frame_id)
{
    static tf::TransformBroadcaster broadcaster;
    tf::Quaternion q;
    q.setRPY(0, 0, 0);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    transform.setRotation(q);

    broadcaster.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), base_frame_id, child_frame_id));
}

void listen_pose( double &x, double &y, std::string target_frame_id, std::string source_frame_id)
{
    static tf::TransformListener listener;
    tf::StampedTransform transform;

    try
    {
        listener.lookupTransform(target_frame_id, source_frame_id, ros::Time(0), transform);
    }
    catch (const tf::TransformException &e)
    {
        ROS_ERROR("%s", e.what());
        ros::Duration(1.0).sleep();
    }

    x = transform.getOrigin().getX();
    y = transform.getOrigin().getY();
}

bool is_beside(double x1, double y1, double x2, double y2, double R)
{
    return ((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) <= (R * R + 0.1));
}
