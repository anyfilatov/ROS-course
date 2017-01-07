#ifndef COURSE_EXPLORER_ROBOTCONTROLLER_H
#define COURSE_EXPLORER_ROBOTCONTROLLER_H


#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "GeomUtils.h"

class RobotController
{
    using Point2D = GeomUtils::Point2D;

public:
    RobotController(ros::NodeHandle& nodeHandle)
    {
        m_odometrySubscriber = nodeHandle.subscribe<nav_msgs::Odometry>("/base_pose_ground_truth", 100,
                                                                        &RobotController::updateCurrentPositions,
                                                                        this);
        m_laserScanSubsriber = nodeHandle.subscribe<sensor_msgs::LaserScan>("/base_scan", 100,
                                                                            &RobotController::updateLaserScan,
                                                                            this);
        m_cmdVelPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    }

    const Point2D& getCurrentPosition() const
    {
        return m_currentPosition;
    }

    float getCurrentAngle() const
    {
        return m_currentAngle;
    }

    const sensor_msgs::LaserScan& getLaserScan() const
    {
        return m_laserScan;
    }

    bool haveNewLaserScan() const
    {
        return m_haveNewLaserScan;
    }

    bool haveNewCurrentPosition() const
    {
        return m_haveNewCurrentPosition;
    }

    bool haveLaseScan() const
    {
        return m_haveLaseScan;
    }

    float getAngleToTarget(const Point2D& targetPosition) const
    {
        float dx = targetPosition.x - m_currentPosition.x;
        float dy = targetPosition.y - m_currentPosition.y;

        float targetAngle = (float) atan2(dy, dx);
        float currentAngle = m_currentAngle;

        float angle = (targetAngle - currentAngle);
        while (angle < -M_PI)
        { angle += 2.f * M_PI; }
        while (angle > M_PI)
        { angle -= 2.f * M_PI; }

        ROS_INFO("Ag %f %f %f", targetAngle, currentAngle, angle);
        return angle;
    }

    float getDistanceToTarget(const Point2D& targetPosition) const
    {
        return (float) sqrt(
                pow(m_currentPosition.x - targetPosition.x, 2.0) +
                pow(m_currentPosition.y - targetPosition.y, 2.0));
    }

    void moveForward(float linearSpeed)
    {
        geometry_msgs::Twist message;
        message.linear.x = linearSpeed;
        m_cmdVelPublisher.publish(message);
    }

    void rotate(float angularSpeed)
    {
        geometry_msgs::Twist message;
        message.angular.z = angularSpeed;
        m_cmdVelPublisher.publish(message);
    }

    void stop()
    {
        m_cmdVelPublisher.publish(geometry_msgs::Twist());
    }

private:
    void updateCurrentPositions(nav_msgs::Odometry odometry)
    {
        m_currentPosition.x = (float) odometry.pose.pose.position.x;
        m_currentPosition.y = (float) odometry.pose.pose.position.y;
        m_currentAngle = (float) tf::getYaw(odometry.pose.pose.orientation);
        m_haveNewCurrentPosition = true;
    }

    void updateLaserScan(sensor_msgs::LaserScan laserScan)
    {
        m_laserScan = laserScan;
        m_haveNewLaserScan = true;
        m_haveLaseScan = true;
    }

    Point2D m_currentPosition;
    float m_currentAngle;
    sensor_msgs::LaserScan m_laserScan;
    bool m_haveLaseScan = false;
    bool m_haveNewLaserScan = false;
    bool m_haveNewCurrentPosition = false;

    ros::Subscriber m_odometrySubscriber;
    ros::Subscriber m_laserScanSubsriber;
    ros::Publisher m_cmdVelPublisher;
};


#endif //COURSE_EXPLORER_ROBOTCONTROLLER_H
