#include "lostrobot.h"

void LostRobot::run(){
    Rate loop(FPS);
    sleep(2);
    bool isExecute = true;
    while(isExecute){
        
        this->myPosition =  getTransform(name);
        this->myFriend = getTransform(friendName);
        
        switch(status){
            case Status::LOST: {
                updateRandomDirection();
                move(speedLinear*randomLinearFactor, speedAngular*randomAngularFactor);
                if (isFriendReached()){
                    move(0, 0);
                    status = Status::ROTATE_TO_FRIEND;
                }
                break;
            }
            case Status::ROTATE_TO_FRIEND:{
                float alpha = getAlfa(myFriend,myPosition);
                if (fabs(alpha) > 0.01)
                    move(0, -alpha);
                else {
                    move(0, 0);
                    status = Status::GO_TO_FRIEND;
                }
                break;
            }
            case Status::GO_TO_FRIEND: {
                float alpha = getAlfa(myFriend,myPosition);
                ROS_INFO("%3.2f", alpha);
                if (!isFriendReached()){
                     move(speedLinear, -alpha);
                    if (fabs(alpha) > 0.3)
                    {
                       move(0, 0); 
                       status = Status::ROTATE_TO_FRIEND;
                    } 
                }
                else move(0,0);
                if (isNearThePoint(0,0, min_dist*1.5)){
                    move(0, 0);
                     isExecute = false;
                 }
                break;
            }
            default: break;
        }
        loop.sleep();
    }
}

void LostRobot::updateRandomDirection(){
    if (countFrame == 0){
        randomAngularFactor = -1.0 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(2)));
        randomLinearFactor = static_cast <float> (rand()) /( static_cast <float> (RAND_MAX));
    }
    countFrame++;
    countFrame %= COUNT_FRAME;
}