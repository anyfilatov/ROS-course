#ifndef ROS_COURCE_BALL_H
#define ROS_COURCE_BALL_H

#include <ros/ros.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "GazeboService.h"
#include <cmath>

using namespace std;

class Ball{
protected:
    ros::Publisher gazeboModelStatePublisher;
    ros::ServiceClient ballPoseClient;
    
    string pathToModel;
    string modelDescription;
    string modelName;
    geometry_msgs::Pose pose;

protected:


    void spawnModel(){
        gazebo_msgs::SpawnModel srv;
        srv.request.model_xml = modelDescription;
        srv.request.model_name = modelName;
        srv.request.initial_pose = pose;
        GazeboService::getInstance().submitSpawnMessage(srv);
        std::cout<<"Spawned robot "<<modelName<<std::endl;
    }

public:
    Ball() 
        : gazeboModelStatePublisher(GazeboService::getInstance().getModelStatePublisher())
        , ballPoseClient(GazeboService::getInstance().getGazeboModelPoseClient())
        , pathToModel("/home/user/Projects/ros/catkin_ws/src/ros-course-work/kurs/models/ball/model.sdf")
        , modelName("ball")
    {
        // double x, double y, const string& pathToModel, const string& modelName
        pose.position.x = 1.0;
        pose.position.y = -2.0;
        pose.position.z = 0.0;

        ifstream fin(pathToModel);
        string lineBuffer;
        while(getline(fin, lineBuffer)) {
            modelDescription += lineBuffer + '\n';
        }

        fin.close();

        spawnModel();
    }

    void beABall(){
        static ros::Publisher ballPositionPublisher = GazeboService::getInstance().getBallPosePublisher();
       
        gazebo_msgs::GetModelState modelState;
        modelState.request.model_name = modelName;
        
        if(ballPoseClient.call(modelState)) {
            geometry_msgs::Pose ballPose(modelState.response.pose);
            if(ballPose.position.x > 10 || ballPose.position.x < -10 ||
               ballPose.position.y > 10 || ballPose.position.y < -10
            ) {
                gazebo_msgs::ModelState modelState;
                modelState.model_name = this->modelName;
                ballPose.position.x = 0.5;
                ballPose.position.y = 0.5;
                modelState.pose = ballPose;
                gazeboModelStatePublisher.publish(modelState);
                spawnModel();
            }
        }
    }
};

#endif