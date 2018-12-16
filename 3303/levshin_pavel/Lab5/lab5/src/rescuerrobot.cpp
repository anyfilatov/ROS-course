#include "rescuerrobot.h"

void RescuerRobot::run(){
    Rate loop(FPS);
    sleep(2);
    bool isExecute = true;
    while(isExecute){    
        this->myPosition =  getTransform(name);
        this->myFriend = getTransform(friendName);

        switch (status){
            case Status::ROTATE_TO_FRIEND: {

                float alpha = getAlfa(myFriend,myPosition);
                if (fabs(alpha) > 0.1)
                    move(0, -alpha);
                else {
                    move(0, 0);
                    status = Status::FIND_FRIEND;
                }
                break;
            }
            case Status::FIND_FRIEND: {
                float alpha = getAlfa(myFriend,myPosition);
                ROS_INFO("%3.2f", alpha);
            
                move(speedLinear*(1- fabs(alpha)/(2*M_PI)), -alpha);

                if (fabs(alpha) > 0.3) {
                    move(0, 0);
                    status = Status::ROTATE_TO_FRIEND;
                }
                if (isFriendReached()){
                    move(0, 0);
                    status = Status::ROTATE_TO_HOME;
                }
                break;
            }
            case Status::ROTATE_TO_HOME: {

                float alpha = getAlfa(home,myPosition);
                if (fabs(alpha) > 0.1)
                    move(0, -alpha);
                else {
                    move(0, 0);
                    status = Status::GO_TO_HOME;
                }
                break;
            }
            case Status::GO_TO_HOME: {
                float alpha = getAlfa(home,myPosition);
                    move(speedLinear, -alpha);
                    if ((fabs(alpha) > 0.3)){
                        move(0, 0);
                        status = Status::ROTATE_TO_HOME;
                    }
                //else move(speedLinear, 0);  
                if (isNearThePoint(home.getOrigin().x(), home.getOrigin().y(), min_dist*0.5)) {
                    move(0,0);
                    isExecute = false;
                }
                break;
            }
            default: break;
        }

        loop.sleep();
    }
}