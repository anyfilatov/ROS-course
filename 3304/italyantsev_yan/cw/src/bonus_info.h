#ifndef BONUS_INFO_H
#define BONUS_INFO_H

#include <string>

class BonusInfo
{
public:
    BonusInfo(const std::string& name);
    BonusInfo(const std::string& name, double currentAngle,
              double angularSpeed);
    void setPosition(double x, double y, double roll, double pitch, double yaw);
    double getCurrentAngle();
    double getAngularSpeed();
    double getX();
    double getY();
    double getRoll();
    double getPitch();
    double getYaw();
    std::string getName();

private:
    double mCurrentAngle;
    double mAngularSpeed;
    double mX;
    double mY;
    double mRoll;
    double mPitch;
    double mYaw;
    std::string mName;
};

#endif // BONUS_INFO_H 
