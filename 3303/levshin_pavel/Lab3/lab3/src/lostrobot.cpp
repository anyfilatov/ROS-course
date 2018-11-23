#include "lostrobot.h"

void LostRobot::run(){
    Rate loop(FPS);
    while(ok()){
        saveTransform();
        pub.publish(marker);
        getFriendTransform();
        if (!isFriendReached() && !isGoToFriend){
            updateRandomDirection();
            moveTo(marker.pose.position.x + randomDirectionX, marker.pose.position.y + randomDirectionY); 
            ROS_INFO("I'm lost! My position: (%3.2f, %3.2f), My directionto the point: (%3.2f, %3.2f)", marker.pose.position.x, marker.pose.position.y, randomDirectionX, randomDirectionY);   
        } else {
            isGoToFriend = true;
            moveTo(myFriend.getOrigin().x(), myFriend.getOrigin().y());
            ROS_INFO("My friend found me! My position: (%3.2f, %3.2f), My directionto the point: (%3.2f, %3.2f)", marker.pose.position.x, marker.pose.position.y, myFriend.getOrigin().x(), myFriend.getOrigin().y());
            if (isNearThePoint(0, 0)) break;
        }
        loop.sleep();
    }
}

void LostRobot::updateRandomDirection(){
    if (countFrame == 0){
        randomDirectionX = rand() % (2*DISPERSION) - DISPERSION;
        randomDirectionY = rand() % (2*DISPERSION) - DISPERSION;
    }
    countFrame++;
    countFrame %= COUNT_FRAME;
}