#include <cmath>
#include "bonus_info.h"

BonusInfo::BonusInfo(const std::string& name) : BonusInfo(name, 0, 0.1)
{
}

BonusInfo::BonusInfo(const std::string& name, double currentAngle, double angularSpeed)
    : mCurrentAngle(currentAngle), mAngularSpeed(angularSpeed),
      mX(-10.0), mY(-10.0), mRoll(0.4), mPitch(0.25), mYaw(0), mName(name)
{
}

void BonusInfo::setPosition(double x, double y, double roll, double pitch, double yaw)
{
    mX = x;
    mY = y;
    mRoll = roll;
    mPitch = pitch;
    mYaw = yaw;	  
}


double BonusInfo::getCurrentAngle()
{
    return mCurrentAngle;
}

double BonusInfo::getAngularSpeed()
{
    return mAngularSpeed;
}

double BonusInfo::getX()
{
    return mX;
}

double BonusInfo::getRoll()
{
    return mRoll;
}

double BonusInfo::getPitch()
{
    return mPitch;
}

double BonusInfo::getYaw()
{
    return mYaw;
}

double BonusInfo::getY()
{
    return mY;
}

std::string BonusInfo::getName()
{
    return mName;
}


