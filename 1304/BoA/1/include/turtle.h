#ifndef TURTLE_H
#define TURTLE_H

#include <cmath>
#include <atomic>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

class Turtle {
public:

    Turtle(ros::NodeHandle& nodeHandle);
    void move(double x, double y);
    void move(double distance);
    void rotate(double angle);
    void setPosition(double x, double y, double theta);
    void setTheta(double theta);
    void setLinearSpeed(double speed);
    void setAngularSpeed(double speed);
    void setLinearPrecision(double value);
    void setAngularPrecision(double value);

private:

    void rotateByRadians(double angle);
    double getRotateAngle(double dx, double dy);
    double degrees2radians(double degrees);
    void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
    void waitPositionUpdate();

    ros::Publisher mVelocityPublisher;
    ros::Subscriber mPoseSubscriber;
    turtlesim::Pose mPose;
    double mAngularSpeed;
    double mLinearSpeed;
    double mAngularPrecision;
    double mLinearPrecision;
    std::atomic<bool> mPoseUpdated;

};

#endif /* TURTLE_H */
