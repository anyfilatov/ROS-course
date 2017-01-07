//
//  robot.cpp
//  RobotSim
//
//  Created by Imhoisili Otokhagua on 06/12/2016.
//  Copyright © 2016 Imhoisili Otokhagua. All rights reserved.
//

#include "robot.h"
#include <fstream>
#include <stdlib.h>

Robot::Robot(BattleField* bf, glm::vec3 position):RobotBase(bf, position)
{   
	_stateMachine = new StateMachine<Robot>(this);
	_battleField = bf;
	spawn();
}

void Robot::updateRos()
{
	//m_modelStateMsg.model_name = std::to_string(_id);
	//m_modelStateMsg.pose.position.x = m_position.x;
	//m_publisher.publish(m_modelStateMsg);

	ROS_INFO_STREAM(std::setprecision(2)<<std::fixed << "found -- px: "<< m_position.x <<" py: "<< m_position.y);
}

StateMachine<Robot>* Robot::getFSM() const
{
    return _stateMachine;
}

void Robot::spawn(std::string modelPath)
{
	ros::service::waitForService("gazebo/spawn_sdf_model");
	ros::ServiceClient add_robot =  m_nodeHandle.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;
 
    std::ifstream fin(modelPath.c_str());
 
    std::string model;
    std::string buf;
    while(!fin.eof()){
        getline(fin, buf);
        model += buf + "\n";
    }
    srv.request.model_xml = model;
    srv.request.model_name = std::to_string(_id);
    geometry_msgs::Pose pose;
	pose.position.x= m_position.x;
    pose.position.y= m_position.y;
	pose.position.z= m_position.z;
	pose.orientation.x= _rx;
	pose.orientation.y= _ry;
	pose.orientation.z= _rz;
	pose.orientation.w= 1;

	srv.request.initial_pose = pose;
    add_robot.call(srv);

	m_publisher = m_nodeHandle.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);

	
}

bool Robot::handleMessage(const Telegram & msg)
{
    return _stateMachine->handleMessage(msg);
}

void Robot::shoot()
{
    //shoot
}

void Robot::advance()
{
    //tells the robot to move around in the battlefield
}

void Robot::update()
{
    _stateMachine->update();
	updateRos();
}
