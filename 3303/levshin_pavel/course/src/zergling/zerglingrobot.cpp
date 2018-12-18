#include "zerglingrobot.h"

ZerglingRobot::ZerglingRobot(string name, int id, float r, float g, float b, float x, float y, float speed, float min_dist): 
                            Robot(  name,
                                    "zerglings", 
                                    id, 
                                    visualization_msgs::Marker::SPHERE, 
                                    tf::Vector3(x, y, 0),
                                    tf::tfVector4(0, 0, 0, 1),
                                    tf::Vector3(0.2, 0.2, 0.2),
                                    tf::tfVector4(r, g, b, 1), speed, min_dist), stage(Stage::FIND_TEMPLAR){};

void ZerglingRobot::handlerHit(const std_msgs::Empty& msg){
    stage = Stage::DEATH;
}

void ZerglingRobot::handlerDeath(const std_msgs::String& msg){
    if (currTemplar.compare("/"+msg.data)){
        stage = Stage::FIND_TEMPLAR;
    }
}

void ZerglingRobot::run(){
    Rate loop(FPS);
    subHit = nh.subscribe(name + "/hit", 100, &ZerglingRobot::handlerHit, this);
    subDeath = nh.subscribe("/death", 100, &ZerglingRobot::handlerDeath, this);
    pubDeath = nh.advertise<std_msgs::String>("/death", 100, true);
    while(isExecute){
        saveTransform();
        pub.publish(marker);
        switch(stage){
            case Stage::FIND_TEMPLAR: {
                V_string templars = getNodesNameByPattern("templar_");
                if (!templars.empty()){
                    string minDistenceName = templars[0];
                    float minDistance = tf::tfDistance2(tf::Vector3(marker.pose.position.x, marker.pose.position.y, 0), getTransform(minDistenceName).getOrigin());
                    for (int it = 1; it < templars.size(); it++){
                        float dist = tf::tfDistance2(tf::Vector3(marker.pose.position.x, marker.pose.position.y, 0), getTransform(templars[it]).getOrigin());
                        if (minDistance > dist){
                                minDistance = dist;
                                minDistenceName = templars[it];
                        }
                    }
                    currTemplar = minDistenceName;
                    stage = Stage::GO_TO_TEMPLAR;
                }
                break;
            }
            case Stage::GO_TO_TEMPLAR: {
                tf::Transform templarTransform = getTransform(currTemplar);
                if (tf::tfDistance2(tf::Vector3(marker.pose.position.x, marker.pose.position.y, 0), templarTransform.getOrigin()) > 0.0625){
                    moveTo(templarTransform.getOrigin().x(), templarTransform.getOrigin().y()); 
                } 
                break;
            }
            case Stage::DEATH:{
                deleteMarker();
                std_msgs::String msg;
                msg.data = name;
                pubDeath.publish(msg);
                isExecute = false;
                break;
            }
            default: break;
        }
        ros::spinOnce();
        loop.sleep();
    }
}