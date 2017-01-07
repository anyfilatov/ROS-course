#ifndef BOOKKEEPER_H
#define BOOKKEEPER_H

#include <vector>
#include <ros/ros.h>
#include <room/welcome.h>
#include <room/person.h>
#include "worker_info.h"

class Bookkeeper {
public:

    Bookkeeper(ros::NodeHandle& nodeHandle, const std::string& firstName, const std::string& lastName);
    void addWorkerInfo(const std::string& firstName, const std::string& lastName, long salary);
    void reportSalaries();

private:

    void welcomeToRoomCallback(const room::welcome::ConstPtr& message);
    void workerGreetingCallback(const room::person::ConstPtr& message);

    std::string mFirstName;
    std::string mLastName;
    bool mInRoom;
    int mRestWorkers;
    std::vector<WorkerInfo> mWorkerList;
    ros::Publisher mSalaryPublisher;
    ros::Publisher mWelcomeToRoomPublisher;
    ros::Publisher mLeaveRoomPublisher;
    ros::Subscriber mWelcomeToRoomSubscriber;
    ros::Subscriber mWorkerGreetingSubscriber;

};

#endif /* BOOKKEEPER_H */
