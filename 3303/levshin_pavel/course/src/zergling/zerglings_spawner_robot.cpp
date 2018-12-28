#include "zerglings_spawner_robot.h"

#include "std_msgs/Empty.h"

ZerglingSpawnerRobot::ZerglingSpawnerRobot(int id, int health, float cooldownSpawn, tf::Vector3 position, tf::tfVector4 rotation, tf::Vector3 scale, tf::tfVector4 color, float speed, float min_dist):
                          Robot("zerglings_spawner_" + std::to_string(id),
                                "zerglings_spawns", 
                                id, 
                                visualization_msgs::Marker::CYLINDER, 
                                position, 
                                rotation,
                                scale,
                                color, speed, min_dist), health(health), cooldownSpawn(cooldownSpawn){};

string ZerglingSpawnerRobot::genRunString(int id_home, int id_unit, float x, float y){
    return ("rosrun course zergling " + std::to_string(id_home) + " " + std::to_string(id_unit)+ " " + std::to_string(x) + " " + std::to_string(y));
}

void ZerglingSpawnerRobot::spawnUnit(int id_home, float x, float y){
    countUnits++;
    zerglings.push_back("zergling_" + std::to_string(id_home) + "_" + std::to_string(countUnits));
    int pid = fork();
    if (pid == 0) {
        system(genRunString(id_home, countUnits, x, y).c_str());
        exit(0);
    }
}
void ZerglingSpawnerRobot::handlerHit(const std_msgs::Empty& msg){
            health--;
}

void ZerglingSpawnerRobot::handlerDeath(const std_msgs::String& msg){
    zerglings.erase( remove( zerglings.begin(), zerglings.end(), msg.data ), zerglings.end());
}

tf::Vector3 ZerglingSpawnerRobot::getNoCollisionPosition(){
    double x,y;
    bool isAccept = false;
    while (!isAccept){
        double x_dir = -1.0 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(2.0)));
        double y_dir = 2 * ( (rand() % 2) - 0.5f) * sqrt(1 - x_dir*x_dir);
        x =  marker.pose.position.x + marker.scale.x/2 * x_dir;
        y =  marker.pose.position.y + marker.scale.y/2 * y_dir;
        V_string::iterator it;
        for (it = zerglings.begin(); it != zerglings.end(); it++){
            tf::Transform transform = getTransform(*it);
            if (tf::tfDistance(transform.getOrigin(), tf::Vector3(x, y, 0)) < 0.5) break;
        }
        if (it == zerglings.end()) isAccept = true;
    }
    return tf::Vector3(x, y, 0);
}

void ZerglingSpawnerRobot::run(){
    Rate loop(FPS);
    sleep((float)marker.id/10.0f);
	srand(std::time(0));
    time = cooldownSpawn;
    subHit = nh.subscribe(name + "/hit", 100, &ZerglingSpawnerRobot::handlerHit, this);
    subDeath = nh.subscribe("/death", 100, &ZerglingSpawnerRobot::handlerDeath, this);
    while(isExecute){
        if ( (fabs(time - cooldownSpawn) < deltaTime)){
            time = 0;
            tf::Vector3 noCollisionPosition = getNoCollisionPosition();
            spawnUnit(marker.id, noCollisionPosition.x(), noCollisionPosition.y());
        }
        if (health <= 0){
            std_msgs::String msg;
            msg.data = "Templar wins!";
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