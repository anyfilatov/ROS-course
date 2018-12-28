#include "templars_spawner_robot.h"

TemplarsSpawnerRobot::TemplarsSpawnerRobot(int id, int countWaves, int countUnitInWave, int cooldownSpawn, tf::Vector3 position, tf::tfVector4 rotation, tf::Vector3 scale, tf::tfVector4 color, float speed, float min_dist):
                            Robot("templars_spawner_" + std::to_string(id),
                                "templars_spawns", 
                                id, 
                                visualization_msgs::Marker::CUBE, 
                                position, 
                                rotation,
                                scale,
                                color, speed, min_dist), 
                            countWaves(countWaves), countUnitInWave(countUnitInWave),cooldownSpawn(cooldownSpawn){};

string TemplarsSpawnerRobot::genRunString(int id_home, int id_unit, float x, float y){
    return ("rosrun course templar " + std::to_string(id_home) + " " + std::to_string(id_unit)+ " " + std::to_string(x) + " " + std::to_string(y));
}

void TemplarsSpawnerRobot::spawnUnit(int id_home, float x, float y){
    countUnits++;
    templars.push_back("templar_" + std::to_string(id_home) + "_" + std::to_string(countUnits));
    int pid = fork();
    if (pid == 0) {
        system(genRunString(id_home, countUnits, x, y).c_str());
        exit(0);
    }
}

tf::Vector3 TemplarsSpawnerRobot::getNoCollisionPosition(){
    double x,y;
    bool isAccept = false;
    while (!isAccept){
        x =  marker.pose.position.x - marker.scale.x/2 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(marker.scale.x)));
        y =  marker.pose.position.y - marker.scale.y/2 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(marker.scale.y)));
        V_string::iterator it;
        for (it = templars.begin(); it != templars.end(); it++){
            tf::Transform transform = getTransform(*it);
            if (tf::tfDistance(transform.getOrigin(), tf::Vector3(x, y, 0)) < 0.5) break;
        }
        if (it == templars.end()) isAccept = true;
    }
    return tf::Vector3(x, y, 0);
}

void TemplarsSpawnerRobot::handlerDeath(const std_msgs::String& msg){
    templars.erase( remove( templars.begin(), templars.end(), msg.data ), templars.end());
}

void TemplarsSpawnerRobot::run(){
    Rate loop(FPS);
    
    subDeath = nh.subscribe("/death", 100, &TemplarsSpawnerRobot::handlerDeath, this);
    while(isExecute){
        if ( (fabs(time - cooldownSpawn) < deltaTime) && (countWaves != 0)){
            time = 0;
            for (int j = 0; j < countUnitInWave; j++){
                tf::Vector3 noCollisionPosition = getNoCollisionPosition();
                spawnUnit(marker.id, noCollisionPosition.x(), noCollisionPosition.y());
            };
            countWaves--;
        }
        
        if ( (countWaves == 0) && templars.empty() ){
            std_msgs::String msg;
            msg.data = "Zergs wins!";
            pubGameOver.publish(msg);
            isExecute = false;
        }
        saveTransform();
        pub.publish(marker);
        ros::spinOnce();
        loop.sleep();
        time += deltaTime;
    }
}