#include "turtle.h"

Turtle::Turtle(ros::NodeHandle& nodeHandle):
    mLinearSpeed(2.0),
    mAngularSpeed(degrees2radians(45.0)),
    mAngularPrecision(degrees2radians(0.01)),
    mLinearPrecision(0.01),
    mPoseUpdated(false)
{
    mVelocityPublisher = nodeHandle.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    mPoseSubscriber = nodeHandle.subscribe("/turtle1/pose", 100, &Turtle::poseCallback, this);
}

void Turtle::move(double x, double y) {
    int updateFrequency = 5;
    double dx, dy, rotateAngle, distance;
    for (int i = updateFrequency; i > 0; --i) {
        waitPositionUpdate();
        dx = x - mPose.x;
        dy = y - mPose.y;
        rotateAngle = getRotateAngle(dx, dy);
        distance = std::sqrt(dx * dx + dy * dy) / i;
        rotateByRadians(rotateAngle - mPose.theta);
        move(distance);
    }
}

void Turtle::move(double distance) {
    geometry_msgs::Twist velocityMessage;
    if (distance < 0) {
        velocityMessage.linear.x = -mLinearSpeed;
        distance = -distance;
    }
    else {
        velocityMessage.linear.x = mLinearSpeed;
    }
    velocityMessage.linear.y = 0;
    velocityMessage.linear.z = 0;

    velocityMessage.angular.x = 0;
    velocityMessage.angular.y = 0;
    velocityMessage.angular.z = 0;

    double currentDistance = 0.0;
    double delta = 0.0;
    double timePoint1, timePoint2;
    ros::Rate loop_rate(100);
    timePoint1 = ros::Time::now().toSec();
    do {
        mVelocityPublisher.publish(velocityMessage);
        timePoint2 = ros::Time::now().toSec();
        currentDistance = mLinearSpeed * (timePoint2 - timePoint1);
        delta = distance - currentDistance;
        if (delta < 0.0) {
            velocityMessage.linear.x = -velocityMessage.linear.x;
        }
        distance = std::abs(delta);
        timePoint1 = timePoint2;
        ros::spinOnce();
        loop_rate.sleep();
    }
    while(distance > mLinearPrecision);
    velocityMessage.linear.x = 0;
    mVelocityPublisher.publish(velocityMessage);
}

void Turtle::rotate(double angle) {
    rotateByRadians(degrees2radians(angle));
}

void Turtle::setPosition(double x, double y, double theta) {
    waitPositionUpdate();
    double dx = x - mPose.x;
    double dy = y - mPose.y;
    double rotateAngle = getRotateAngle(dx, dy);
    rotateByRadians(rotateAngle - mPose.theta);
    move(std::sqrt(dx * dx + dy * dy));
    rotateByRadians(theta - mPose.theta);
}

void Turtle::setTheta(double theta) {
    waitPositionUpdate();
    rotateByRadians(degrees2radians(theta) - mPose.theta);
}

void Turtle::setLinearSpeed(double speed) {
    mLinearSpeed  = (speed < 0 ? -speed : speed);
}

void Turtle::setAngularSpeed(double speed) {
    mAngularSpeed  = degrees2radians(speed < 0 ? -speed : speed);
}

void Turtle::setLinearPrecision(double value) {
    mLinearPrecision = (value < 0 ? -value : value);
}

void Turtle::setAngularPrecision(double value) {
    mAngularPrecision = (value < 0 ? -value : value);
}

void Turtle::rotateByRadians(double angle) {
    geometry_msgs::Twist velocityMessage;

    velocityMessage.linear.x = 0;
    velocityMessage.linear.y = 0;
    velocityMessage.linear.z = 0;

    velocityMessage.angular.x = 0;
    velocityMessage.angular.y = 0;
    if (angle < 0) {
        velocityMessage.angular.z = -mAngularSpeed;
        angle = -angle;
    }
    else {
        velocityMessage.angular.z = mAngularSpeed;
    }

    double currentAngle = 0.0;
    double delta = 0.0;
    double timePoint1, timePoint2;
    ros::Rate loop_rate(100);
    timePoint1 = ros::Time::now().toSec();
    do {
        mVelocityPublisher.publish(velocityMessage);
        timePoint2 = ros::Time::now().toSec();
        currentAngle = mAngularSpeed * (timePoint2 - timePoint1);
        delta = angle - currentAngle;
        if (delta < 0.0) {
            velocityMessage.angular.z = -velocityMessage.angular.z;
        }
        angle = std::abs(delta);
        timePoint1 = timePoint2;
        ros::spinOnce();
        loop_rate.sleep();
    }
    while(angle > mAngularPrecision);
    velocityMessage.angular.z = 0;
    mVelocityPublisher.publish(velocityMessage);
}

double Turtle::getRotateAngle(double dx, double dy) {
    if (dy == 0) {
        if (dx < 0) return M_PI;
        else return 0.0;
    }
    else {
        if (dx == 0) {
            if (dy < 0) return - M_PI / 2;
            else return M_PI / 2;
        }
        double angleCos = dx / std::sqrt(dx * dx + dy * dy);
        if (dy < 0) {
            return -std::acos(angleCos);
        }
        else {
            return std::acos(angleCos);
        }
    }
    return 0.0;
}

double Turtle::degrees2radians(double degrees) {
    return degrees * M_PI / 180.0;
}

void Turtle::poseCallback(const turtlesim::Pose::ConstPtr& poseMessage) {
    mPose.x = poseMessage->x;
    mPose.y = poseMessage->y;
    mPose.theta = poseMessage->theta;
    mPoseUpdated = true;
}

void Turtle::waitPositionUpdate() {
    mPoseUpdated = false;
    ros::Rate loop_rate(100);
    while (!mPoseUpdated) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
