#include "room.h"
#include <room/welcome.h>
#include <ros/console.h>

Room::Room(ros::NodeHandle& nodeHandle): mRestPersons(0), mBookkeperArrived(false) {
    mWelcomeToRoomPublisher = nodeHandle.advertise<room::welcome>("/room/welcome", 1000);
    mLeaveRoomSubscriber = nodeHandle.subscribe("/room/leave", 100, &Room::leaveCallback, this);
    mPeopleCountSubscriber = nodeHandle.subscribe("/room/welcome", 100, &Room::peopleCountCallback, this);
    mEmotionSubscriber = nodeHandle.subscribe("/room/emotions", 100, &Room::emotionCallback, this);
}

void Room::waitAllLeaveRoom() {
    ROS_INFO("[Room] waitAllLeaveRoom start");
    ROS_DEBUG("[Room] send welcome message");
    room::welcome welcomeMessage;
    welcomeMessage.person_count = 0;

    ROS_DEBUG("[Room] wait bookkeeper");
    ros::Rate loop_rate(100);
    do {
        mWelcomeToRoomPublisher.publish(welcomeMessage);
        ros::spinOnce();
        loop_rate.sleep();
    } while(!mBookkeperArrived);

    ROS_DEBUG("[Room] wait all leave room");
    do {
        ros::spinOnce();
        loop_rate.sleep();
    } while(mRestPersons > 0);

    ROS_INFO("[Room] end");
}

void Room::leaveCallback(const room::person::ConstPtr& message) {
    ROS_DEBUG("[Room] leaveCallback(%s %s)", message->first_name.c_str(), message->last_name.c_str());
    mRestPersons--;
}

void Room::peopleCountCallback(const room::welcome::ConstPtr& message) {
    ROS_DEBUG("[Room] peopleCountCallback(%d)", message->person_count);
    if (message->person_count > 0) {
        mRestPersons = message->person_count;
        mBookkeperArrived = true;
    }
}

void Room::emotionCallback(const room::emotion::ConstPtr& message) {
    ROS_INFO("[Room] emotionCallback(%s %s) %s", message->first_name.c_str(), message->last_name.c_str(), message->emotion.c_str());
}
