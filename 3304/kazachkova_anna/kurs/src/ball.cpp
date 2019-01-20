#include "ros/ros.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include "ball.h"

int main(int argc, char** argv) {
 
    ros::init(argc, argv, "ball");
    GazeboService::getInstance();
    ros::service::waitForService("gazebo/spawn_sdf_model"); 

    ros::Rate rate(20);
    Ball *ball = new Ball();

    while(ros::ok()){
        //be a ball
        ball->beABall();
        rate.sleep();
    }
    
    return 0;
}
