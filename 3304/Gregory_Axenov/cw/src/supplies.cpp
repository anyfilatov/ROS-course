#include <random>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/ModelState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cw/status.h>
#include "robot_info.h"

enum class State { Chase, Final };

bool isHooked = false;
bool Checkpoint1 = false;
bool Checkpoint2 = false;
bool Checkpoint3 = false;
bool Checkpoint4 = false;
bool Checkpoint5 = false;
bool isExit = false;

void statusCallback(const cw::status::ConstPtr& message)
{
    if (message->is_hooked)
    {
        isHooked = true;
    }
    if (message->check1)
    {
        Checkpoint1 = true;
    }
    if (message->check2)
    {
        Checkpoint2 = true;
    }
    if (message->check3)
    {
        Checkpoint3 = true;
    }
    if (message->check4)
    {
       Checkpoint4 = true;
    }
    if (message->check5)
    {
       Checkpoint5 = true;
    }
    if (message->exit)
    {
       isExit = true;
    }
}


void spawnRobot(ros::NodeHandle& node, const std::string& name, const std::string& model_path, double x, double y, double z)
{
    ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::ServiceClient add_robot = node.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;
    std::ifstream fin(model_path.c_str());
    std::string model_xml;
    std::string buf;
    while(!fin.eof())
    {
        getline(fin, buf);
        model_xml += buf + "\n";
    }
    srv.request.model_xml = model_xml;
    srv.request.model_name = name;
    geometry_msgs::Pose pose;
    geometry_msgs::Twist twist;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;
    srv.request.initial_pose = pose;
    add_robot.call(srv);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bonus");
    ros::NodeHandle node;
    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    tf::Transform strayRobotTransform;
    tf::StampedTransform finderRobotTransform;
    double dx, dy, distance;
    double goalX, goalY, goalX2, goalY2, goalX3, goalY3, goalX4, goalY4, goalX5, goalY5;
    double hookRange = 0.5;
    double exitX, exitY;
    State state = State::Chase;
    std::random_device rd;
    std::default_random_engine engine(rd());
    //area where robot travel
    std::uniform_real_distribution<> uniform_dist_x1(-10.0, -6.0);
    std::uniform_real_distribution<> uniform_dist_y1(-10.0, -6.0);
    std::uniform_real_distribution<> uniform_dist_x2(-6.0, -2.0);
    std::uniform_real_distribution<> uniform_dist_y2(-6.0, -2.0);
    std::uniform_real_distribution<> uniform_dist_x3(-2.0, 2.0);
    std::uniform_real_distribution<> uniform_dist_y3(-2.0, 2.0);
    std::uniform_real_distribution<> uniform_dist_x4(2.0, 6.0);
    std::uniform_real_distribution<> uniform_dist_y4(2.0, 6.0);
    std::uniform_real_distribution<> uniform_dist_x5(6.0, 8.0);
    std::uniform_real_distribution<> uniform_dist_y5(9.5, 10.0);

    // generate robot
    RobotInfo robot("bonus");
    RobotInfo robot2("bonus2");
    RobotInfo robot3("bonus3");
    RobotInfo robot4("bonus4");
    RobotInfo robot5("bonus5");
    goalX = uniform_dist_x1(engine);
    goalY = uniform_dist_y1(engine);
    goalX2 = uniform_dist_x2(engine);
    goalY2 = uniform_dist_y2(engine);
    goalX3 = uniform_dist_x3(engine);
    goalY3 = uniform_dist_y3(engine);
    goalX4 = uniform_dist_x4(engine);
    goalY4 = uniform_dist_y4(engine);
    goalX5 = uniform_dist_x5(engine);
    goalY5 = uniform_dist_y5(engine);

    spawnRobot(node, robot.getName(), "/home/gregory/.gazebo/models/beer/model.sdf", goalX,  goalY, 0.0);
    spawnRobot(node, robot2.getName(), "/home/gregory/.gazebo/models/beer/model.sdf", goalX2,  goalY2, 0.0);
    spawnRobot(node, robot3.getName(), "/home/gregory/.gazebo/models/beer/model.sdf", goalX3,  goalY3, 0.0);
    spawnRobot(node, robot4.getName(), "/home/gregory/.gazebo/models/beer/model.sdf", goalX4,  goalY4, 0.0);
    spawnRobot(node, robot5.getName(), "/home/gregory/.gazebo/models/beer/model.sdf", goalX5,  goalY5, 0.0);
    
    exitX = 10.0;
    exitY = 10.0;

    spawnRobot(node, "exit_point", "/home/gregory/.gazebo/models/stop_sign/model.sdf", exitX, exitY, 0.0);

    ros::Publisher gazeboPublisher = node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    ros::Publisher statusPublisher = node.advertise<cw::status>("/status", 1000);
    ros::Subscriber statusSupscriber = node.subscribe("/status", 10, &statusCallback);
    sleep(1.0);

    gazebo_msgs::ModelState robotState;
    robotState.model_name = robot.getName();
    robotState.pose.position.x = goalX;
    robotState.pose.position.y = goalY;
    robotState.pose.orientation.x = 0.0;
    robotState.pose.orientation.y = 0.0;
    robotState.pose.orientation.z = 0.0;
    robotState.pose.orientation.w = 1.0;

    strayRobotTransform.setOrigin(tf::Vector3(goalX, goalY, 0.0));
    strayRobotTransform.setRotation(tf::Quaternion(0,0,0,1));
    broadcaster.sendTransform(tf::StampedTransform(strayRobotTransform, ros::Time::now(), "world", "bonus"));

    ros::Rate rate(30);
    ROS_INFO("Start");
    gazeboPublisher.publish(robotState);
    while ((state != State::Final) && ros::ok())
    {	
    	
        if (state == State::Chase)
        {
            ros::spinOnce();
            // randomMove
            if (Checkpoint1)
            {
                ROS_INFO("Checkpoint 1");
				Checkpoint1 = false; 
				goalX = goalX2;
				goalY = goalY2;
				robotState.model_name = "bonus";
				robotState.pose.position.z = 1000;				
				gazeboPublisher.publish(robotState);
            }
            if (Checkpoint2)
            {
                ROS_INFO("Checkpoint 2");
                Checkpoint2 = false;
				goalX = goalX3;
				goalY = goalY3;
				robotState.model_name = "bonus2";
				robotState.pose.position.z = 1000;
				gazeboPublisher.publish(robotState);
            } 
	    if (Checkpoint3)
            {
                ROS_INFO("Checkpoint 3");
                Checkpoint3 = false;
				goalX = goalX4;
				goalY = goalY4;
				robotState.model_name = "bonus3";
				robotState.pose.position.z = 1000;
				gazeboPublisher.publish(robotState);
            }
            if (Checkpoint4)
            {
                ROS_INFO("Checkpoint 4");
                Checkpoint4 = false;
                goalX = goalX5;
				goalY = goalY5;			
				robotState.model_name = "bonus4";
				robotState.pose.position.z = 1000;
				gazeboPublisher.publish(robotState);
            }
	    if (Checkpoint5)
            {
                ROS_INFO("Checkpoint 5");
                Checkpoint5 = false;
                goalX = exitX;
				goalY = exitY;
				robotState.model_name = "bonus5";
				robotState.pose.position.z = 1000;
				gazeboPublisher.publish(robotState);
            }
        if (isExit)
        {
            ROS_INFO("Escape point");
            Checkpoint5 = false;
            ROS_INFO("You escaped congratulations!");
            ros::Duration(1.0).sleep();
			state = State::Final;
        }
	     
        }


        strayRobotTransform.setOrigin(tf::Vector3(goalX, goalY, 0.0));
        strayRobotTransform.setRotation(tf::Quaternion(0,0,0,1));
        broadcaster.sendTransform(tf::StampedTransform(strayRobotTransform, ros::Time::now(), "world", "bonus"));

 
        rate.sleep();
    }
    return 0;
} 
