#include "worker.h"
#include <ros/console.h>

Worker::Worker(ros::NodeHandle& nodeHandle, const std::string& firstName, const std::string& lastName):
    mFirstName(firstName),
    mLastName(lastName),
    mSalary(-1),
    mMaxSalary(-1),
    mInRoom(false),
    mCanExit(false)
{
    mGreetingPublisher = nodeHandle.advertise<room::person>("/room/worker_greeting", 1000);
    mEmotionPublisher = nodeHandle.advertise<room::emotion>("/room/emotions", 1000);
    mLeaveRoomPublisher = nodeHandle.advertise<room::person>("/room/leave", 1000);
    mWelcomeToRoomSubscriber = nodeHandle.subscribe("/room/welcome", 100, &Worker::welcomeToRoomCallback, this);
    mSalarySubscriber = nodeHandle.subscribe("/room/salary", 100, &Worker::salaryCallback, this);
    mMeetingFinishSubscriber = nodeHandle.subscribe("/room/leave", 100, &Worker::leaveCallback, this);
}

void Worker::listen() {
    ROS_INFO("[Worker %s %s] start listen", mFirstName.c_str(), mLastName.c_str());
    ROS_DEBUG("[Worker] wait for welcome to room");
    ros::Rate loop_rate(100);
    do {
        ros::spinOnce();
        loop_rate.sleep();
    }
    while(!mInRoom);

    ROS_DEBUG("[Worker] send greeting message");
    room::person greetingMessage;
    greetingMessage.first_name = mFirstName;
    greetingMessage.last_name = mLastName;
    mGreetingPublisher.publish(greetingMessage);

    ROS_DEBUG("[Worker] wait for meeting finish");
    while(!mCanExit) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_DEBUG("[Worker] leave room");
    room::person leaveMessage;
    leaveMessage.first_name = mFirstName;
    leaveMessage.last_name = mLastName;
    mLeaveRoomPublisher.publish(leaveMessage);

    ROS_INFO("[Worker] end");
}

void Worker::welcomeToRoomCallback(const room::welcome::ConstPtr& message) {
    ROS_DEBUG("[Worker] welcomeToRoomCallback(%d)", message->person_count);
    if (message->person_count > 0) {
        mInRoom = true;
    }
}

void Worker::salaryCallback(const room::salary::ConstPtr& message) {
    ROS_DEBUG("[Worker] salaryCallback(%s %s)", message->first_name.c_str(), message->last_name.c_str());
    room::emotion emotionMessage;
    emotionMessage.first_name = mFirstName;
    emotionMessage.last_name = mLastName;
    if (message->salary > mMaxSalary) {
        if ((mSalary > 0) && (mSalary == mMaxSalary)) {
            emotionMessage.emotion = ":-|";
            mEmotionPublisher.publish(emotionMessage);
        }
        mMaxSalary = message->salary;
        if ((message->last_name == mLastName) && (message->first_name == mFirstName)) {
            mSalary = mMaxSalary;
            emotionMessage.emotion = ":-D";
            mEmotionPublisher.publish(emotionMessage);
        }
    }
    else if ((message->last_name == mLastName) && (message->first_name == mFirstName)) {
        emotionMessage.emotion = ":-|";
        mEmotionPublisher.publish(emotionMessage);
    }
}

void Worker::leaveCallback(const room::person::ConstPtr& message) {
    ROS_DEBUG("[Worker] leaveCallback");
    if (!mCanExit) {
        mCanExit = true;
    }
}
