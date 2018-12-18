#include "kerrigan_robot.h"

KerriganRobot::KerriganRobot(string name, float cooldownHit, int id, float r, float g, float b, float x, float y, float speed, float min_dist): 
                            Robot(  name,
                                    "kerrigan", 
                                    id, 
                                    visualization_msgs::Marker::SPHERE, 
                                    tf::Vector3(x, y, 0.5),
                                    tf::tfVector4(0, 0, 0, 1),
                                    tf::Vector3(0.5, 0.5, 0.5),
                                    tf::tfVector4(r, g, b, 1), speed, min_dist), cooldownHit(cooldownHit), stage(Stage::FIND_TEMPLAR){};

void KerriganRobot::handlerHit(const std_msgs::Empty& msg){
    stage = Stage::DEATH;
}

void KerriganRobot::run(){
    Rate loop(FPS);
    string currTemplar;
    subHit = nh.subscribe(name + "/hit", 100, &KerriganRobot::handlerHit, this);
    pubDeath = nh.advertise<std_msgs::String>("/death", 100, true);
    bool isHit = false;
    while(isExecute){    
        saveTransform();
        pub.publish(marker);
        switch(stage){
            case Stage::FIND_TEMPLAR: {
                V_string templars = getNodesNameByPattern("templar_");
                if (!templars.empty()){
                    string minDistenceName = templars[0];
                    tfScalar minDistance = tf::tfDistance2(tf::Vector3(marker.pose.position.x, marker.pose.position.y, 0), getTransform(minDistenceName).getOrigin());
                    for (int it = 1; it < templars.size(); it++){
                        tfScalar dist = tf::tfDistance2(tf::Vector3(marker.pose.position.x, marker.pose.position.y, 0), getTransform(templars[it]).getOrigin());
                        if (minDistance > dist){
                                minDistance = dist;
                                minDistenceName = templars[it];
                        }
                    }
                    currTemplar = minDistenceName;
                    pubHitTemplar = nh.advertise<std_msgs::Empty>(minDistenceName + "/hit", 100, true);
                    stage = Stage::GO_TO_TEMPLAR;
                }
                break;
            }
            case Stage::GO_TO_TEMPLAR: { 
                tf::Transform templarTransform = getTransform(currTemplar);
                moveTo(templarTransform.getOrigin().x(), templarTransform.getOrigin().y());  
                if (tf::tfDistance2(tf::Vector3(marker.pose.position.x, marker.pose.position.y, 0), templarTransform.getOrigin()) < 0.25){
                    stage = Stage::HIT_TEMPLER;
                } 
                break;
            }
            case Stage::HIT_TEMPLER:{
                if ( (fabs(time - cooldownHit) < deltaTime)){
                    time = 0;
                    pubHitTemplar.publish(std_msgs::Empty());
                    stage = Stage::FIND_TEMPLAR;
                }
                time += deltaTime;
                break;
            }
            case Stage::DEATH:{
                deleteMarker();
                isExecute = false;
                break;
            }
            default: break;
        }
        ros::spinOnce();
        loop.sleep();
    }
}