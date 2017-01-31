#ifndef ROOM_H
#define ROOM_H

#include <ros/ros.h>
#include <room/person.h>
#include <room/welcome.h>
#include <room/emotion.h>

class Room {
public:

    Room(ros::NodeHandle& nodeHandle);
    void waitAllLeaveRoom();

private:

    void leaveCallback(const room::person::ConstPtr& message);
    void peopleCountCallback(const room::welcome::ConstPtr& message);
    void emotionCallback(const room::emotion::ConstPtr& message);

    int mRestPersons;
    bool mBookkeperArrived;
    ros::Publisher mWelcomeToRoomPublisher;
    ros::Subscriber mPeopleCountSubscriber;
    ros::Subscriber mEmotionSubscriber;
    ros::Subscriber mLeaveRoomSubscriber;

};

#endif /* ROOM_H */
