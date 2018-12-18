#include "templarrobot.h"

TemplarRobot::TemplarRobot(string name, float cooldownHit, int id, float r, float g, float b, float x, float y, float speed, float min_dist): 
                            Robot(  name,
                                    "templars", 
                                    id, 
                                    visualization_msgs::Marker::CYLINDER, 
                                    tf::Vector3(x, y, 0.5),
                                    tf::tfVector4(0, 0, 0, 1),
                                    tf::Vector3(0.5, 0.5, 0.5),
                                    tf::tfVector4(r, g, b, 1), speed, min_dist), cooldownHit(cooldownHit), stage(Stage::FIND_CAMP){};

void TemplarRobot::handlerHit(const std_msgs::Empty& msg){
    stage = Stage::DEATH;
}

void TemplarRobot::run(){
    Rate loop(FPS);
    V_string camps = getNodesNameByPattern("zerglings_spawner_");
    string currCamp;
    Stage oldStage;
    subHit = nh.subscribe(name + "/hit", 100, &TemplarRobot::handlerHit, this);
    pubDeath = nh.advertise<std_msgs::String>("/death", 100, true);
    bool isHit = false;
    while(isExecute){    
        saveTransform();
        pub.publish(marker);
        switch(stage){
            case Stage::FIND_CAMP: {

	            srand(std::time(0));
                currCamp = camps[random() % (camps.size())];
                pubHitCamp = nh.advertise<std_msgs::Empty>(currCamp + "/hit", 100, true);
                stage = Stage::GO_TO_CAMP;
                break;
            }
            case Stage::GO_TO_CAMP:{ 
                tf::Transform campTransform = getTransform(currCamp);
                moveTo(campTransform.getOrigin().x(), campTransform.getOrigin().y());  
                if (tf::tfDistance2(tf::Vector3(marker.pose.position.x, marker.pose.position.y, 0), campTransform.getOrigin()) < 1.0){
                    stage = Stage::HIT_CAMP;
                } 
                break;
            }
            case Stage::HIT_CAMP:{
                if ( (fabs(time - cooldownHit) < deltaTime)){
                    time = 0;
                    pubHitCamp.publish(std_msgs::Empty());
                }
                time += deltaTime;
                break;
            }
            case Stage::HIT_ZERG:{
                if ( (fabs(time - cooldownHit) < deltaTime)){
                    time = 0;
                    isHit = false;
                    stage = oldStage;
                }
                time += deltaTime;
                break;
            }
            case Stage::DEATH:{
                deleteMarker();
                std_msgs::String msg;
                msg.data = name;
                isHit = true;
                pubDeath.publish(msg);
                isExecute = false;
                break;
            }
            default: break;
        }
        if (!isHit){
            V_string zerglings = getNodesNameByPattern("zergling_");	
            for (auto it = zerglings.begin(); it != zerglings.end(); it++){
                tf::Transform transform = getTransform(*it);
                if (tf::tfDistance2(transform.getOrigin(), tf::Vector3(marker.pose.position.x, marker.pose.position.y, 0)) < 0.25){
                    time = 0;
                    isHit = true;
                    pubHitZerg = nh.advertise<std_msgs::Empty>( (*it) + "/hit", 100);
                    pubHitZerg.publish(std_msgs::Empty());
                    oldStage = stage;
                    stage = Stage::HIT_ZERG;
                    break;
                }
            }
        }
        ros::spinOnce();
        loop.sleep();
    }
    sleep(1);
}