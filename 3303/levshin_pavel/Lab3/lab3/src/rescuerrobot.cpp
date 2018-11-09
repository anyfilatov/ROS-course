#include "rescuerrobot.h"

void RescuerRobot::run(){
    Rate loop(FPS);
    while(ok()){    
        saveTransform();
        pub.publish(marker);	
        getFriendTransform();  
        if (!isFriendReached() && isGoToFriend){
            moveTo(myFriend.getOrigin().x(), myFriend.getOrigin().y());
            ROS_INFO("I'm looking for my friend! My position: (%3.2f, %3.2f), My directionto the point: (%3.2f, %3.2f)", marker.pose.position.x, marker.pose.position.y, myFriend.getOrigin().x(), myFriend.getOrigin().y());   
        } else {
            isGoToFriend = false;
            moveTo(homeX, homeY);
            ROS_INFO("I found my friend! We are going home! My position: (%3.2f, %3.2f), My directionto the point: (%3.2f, %3.2f)", marker.pose.position.x, marker.pose.position.y, myFriend.getOrigin().x(), myFriend.getOrigin().y());
            if (isNearThePoint(homeX, homeY)) break;
        }
        loop.sleep();
    }
}