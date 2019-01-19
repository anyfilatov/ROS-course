#include <cmath>
#include "robot_info.h"
#include "ros/ros.h"

RobotInfo::RobotInfo(const std::string& name) : RobotInfo(name, 0.01, 1.0 * M_PI / 180, 0.0, 1.0, 0.1, 0.05)
{
}

RobotInfo::RobotInfo(const std::string& name, double distancePrecision, double angularPrecision, double currentAngle,
          double linearSpeed, double angularSpeed, double maxLinearSpeed)
    : mDistancePrecision(distancePrecision), mAngularPrecision(angularPrecision), mCurrentAngle(currentAngle),
      mLinearSpeed(linearSpeed), mAngularSpeed(angularSpeed),mMaxLinearSpeed(maxLinearSpeed),
      mX(0.0), mY(0.0), mName(name)
{
}

void RobotInfo::setPosition(double x, double y)
{
    mX = x;
    mY = y;
}

double RobotInfo::getDistancePrecision()
{
    return mDistancePrecision;
}

double RobotInfo::getAngularPrecision()
{
    return mAngularPrecision;
}

double RobotInfo::getCurrentAngle()
{
    return mCurrentAngle;
}

double RobotInfo::getLinearSpeed()
{
    return mLinearSpeed;
}

double RobotInfo::getAngularSpeed()
{
    return mAngularSpeed;
}

double RobotInfo::getMaxLinearSpeed()
{
    return mMaxLinearSpeed;
}

double RobotInfo::getX()
{
    return mX;
}

double RobotInfo::getY()
{
    return mY;
}

std::string RobotInfo::getName()
{
    return mName;
}

void RobotInfo::updatePosition(double dx, double dy, double distance)
{
    // update coords
    double currentLinearSpeed = mLinearSpeed * distance;
    if (currentLinearSpeed > mMaxLinearSpeed) currentLinearSpeed = mMaxLinearSpeed;
    double diffAngle = atan2(dy, dx);
    mX += currentLinearSpeed * std::cos(diffAngle);
    mY += currentLinearSpeed * std::sin(diffAngle);

    // update angle
    ros::Rate r(30);
    diffAngle = getRotateAngle(dx, dy);
    float da = M_PI / 20;
    float dt = (float)r.expectedCycleTime().toSec();
    if (fabs(mCurrentAngle - diffAngle) < 2 * da)
        {
            dx += mX * dt;
            dy += mY * dt;
            mCurrentAngle = diffAngle;
        }
        else
        {
            if (diffAngle > mCurrentAngle)
            {
                mCurrentAngle = (mCurrentAngle + da > M_PI ? 0 : mCurrentAngle + da);
            }
            else
            {
                mCurrentAngle = (mCurrentAngle - da < -M_PI ? 0 : mCurrentAngle - da);
            }
        }
}

double RobotInfo::getRotateAngle(double dx, double dy)
{
    if (dy == 0) return (dx < 0 ? M_PI : 0.0);
    else
    {
        if (dx == 0) return (dy < 0 ? -M_PI/2 : M_PI/2);
        double angleCos = dx / std::sqrt(dx * dx + dy * dy);
        return (dy < 0 ? -std::acos(angleCos) : std::acos(angleCos));
    }
    return 0.0;
} 
