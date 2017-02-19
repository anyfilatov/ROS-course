#include "bookkeeper.h"
#include <room/salary.h>
#include <ros/console.h>

Bookkeeper::Bookkeeper(ros::NodeHandle& nodeHandle, const std::string& firstName, const std::string& lastName):\
    mFirstName(firstName),
    mLastName(lastName),
    mInRoom(false),
    mRestWorkers(0)
{
    mSalaryPublisher = nodeHandle.advertise<room::salary>("/room/salary", 1000);
    mWelcomeToRoomPublisher = nodeHandle.advertise<room::welcome>("/room/welcome", 1000);
    mLeaveRoomPublisher = nodeHandle.advertise<room::person>("/room/leave", 1000);
    mWelcomeToRoomSubscriber = nodeHandle.subscribe("/room/welcome", 100, &Bookkeeper::welcomeToRoomCallback, this);
    mWorkerGreetingSubscriber = nodeHandle.subscribe("/room/worker_greeting", 100, &Bookkeeper::workerGreetingCallback, this);
}

void Bookkeeper::addWorkerInfo(const std::string& firstName, const std::string& lastName, long salary) {
    mWorkerList.push_back(WorkerInfo(firstName, lastName, salary));
    mRestWorkers++;
}

void Bookkeeper::reportSalaries() {
    ROS_INFO("[Bookkeeper] start reportSalaries");
    ROS_DEBUG("[Bookkeeper] wait enter to room");
    // enter to room
    ros::Rate loop_rate(100);
    do {
        ros::spinOnce();
        loop_rate.sleep();
    }
    while(!mInRoom);

    ROS_DEBUG("[Bookkeeper] wait all workers");
    // wait all workers
    room::welcome welcomeMessage;
    welcomeMessage.person_count = mWorkerList.size();
    do {
        mWelcomeToRoomPublisher.publish(welcomeMessage);
        ros::spinOnce();
        loop_rate.sleep();
    }
    while(mRestWorkers > 0);

    ROS_DEBUG("[Bookkeeper] report salaries");
    // report salaries
    room::salary salaryMessage;
    for (int i = 0; i < mWorkerList.size(); ++i) {
        salaryMessage.first_name = mWorkerList[i].firstName;
        salaryMessage.last_name = mWorkerList[i].lastName;
        salaryMessage.salary = mWorkerList[i].salary;
        mSalaryPublisher.publish(salaryMessage);
    }

    ROS_DEBUG("[Bookkeeper] leave room");
    // leave room
    room::person leaveMessage;
    leaveMessage.first_name = mFirstName;
    leaveMessage.last_name = mLastName;
    mLeaveRoomPublisher.publish(leaveMessage);

    ROS_INFO("[Bookkeeper] end");
}

void Bookkeeper::welcomeToRoomCallback(const room::welcome::ConstPtr& message) {
    ROS_DEBUG("[Bookkeeper] welcomeToRoomCallback(%d)", message->person_count);
    if (message->person_count == 0) {
        mInRoom = true;
    }
}

void Bookkeeper::workerGreetingCallback(const room::person::ConstPtr& message) {
    ROS_DEBUG("[Bookkeeper] workerGreetingCallback(%s %s)", message->first_name.c_str(), message->last_name.c_str());
    for (int i = 0; i < mWorkerList.size(); ++i) {
        if ((mWorkerList[i].firstName == message->first_name)
            && (mWorkerList[i].lastName == message->last_name)) {
            mRestWorkers--;
            break;
        }
    }
}
