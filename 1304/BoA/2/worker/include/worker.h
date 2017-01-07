#ifndef WORKER_H
#define WORKER_H

#include <ros/ros.h>
#include <room/emotion.h>
#include <room/welcome.h>
#include <room/salary.h>
#include <room/person.h>
#include <string>

class Worker {
public:

    Worker(ros::NodeHandle& nodeHandle, const std::string& firstName, const std::string& lastName);
    void listen();

private:

    void welcomeToRoomCallback(const room::welcome::ConstPtr& message);
    void salaryCallback(const room::salary::ConstPtr& message);
    void leaveCallback(const room::person::ConstPtr& message);

    ros::Publisher mGreetingPublisher;
    ros::Publisher mEmotionPublisher;
    ros::Publisher mLeaveRoomPublisher;

    ros::Subscriber mWelcomeToRoomSubscriber;
    ros::Subscriber mSalarySubscriber;
    ros::Subscriber mMeetingFinishSubscriber;

    bool mInRoom;
    bool mCanExit;
    std::string mFirstName;
    std::string mLastName;
    long mSalary;
    long mMaxSalary;

};

#endif /* WORKER_H */
