#pragma once
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

void broadcast_pose(double x, double y, std::string base_frame_id, std::string child_frame_id)
{
    static tf::TransformBroadcaster br;
    tf::Quaternion q;
    q.setRPY(0, 0, 0);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    transform.setRotation(q);

    br.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), base_frame_id, child_frame_id));
}

void take_pose(
    std::string target_frame_id,
    std::string source_frame_id,
    double &x,
    double &y
    )
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
        throw;
    }
    x = transform.getOrigin().getX();
    y = transform.getOrigin().getY();
}


double calc_distance(double x1, double y1, double x2, double y2)
{
    return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
}

double calc_angle(double x, double y)
{
    if (std::fabs(y) < 1e-6) return (x < 0 ? M_PI : 0.0);
	if (std::fabs(x) < 1e-6) return (y < 0 ? -M_PI / 2 : M_PI / 2);
    double angle = x / std::sqrt(x * x + y * y);
    return (y < 0 ? -std::acos(angle) : std::acos(angle));
}
