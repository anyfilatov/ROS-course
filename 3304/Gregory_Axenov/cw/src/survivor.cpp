#include <random>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/ModelState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cw/status.h>
#include <cw/picked.h>
#include "robot_info.h"

enum class State { Chase, Waiting, Final };

bool missionComplete = false;
bool youDied = false;

void statusCallback(const cw::status::ConstPtr& message)
{
    if (!message->exit)
    {
        missionComplete = true;
    }
    if (!message->is_hooked)
    {
		youDied = true;
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
    ros::init(argc, argv, "finder_robot");
    ros::NodeHandle node;

    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    tf::Transform finderTransform;
    tf::StampedTransform lostRobotTransform;
    tf::StampedTransform zombieTransform;
    
    double dx, dy, distance;
    double hookRange = 0.6;
    double pickupRange = 0.5;
    State state = State::Chase;
    std::random_device rd;
    std::default_random_engine engine(rd());
    std::uniform_real_distribution<> uniform_dist(-6.0, 6.0);

    RobotInfo robot("finder_robot");
    robot.setPosition(-10.0, -10.0);



    spawnRobot(node, robot.getName(), "/home/gregory/.gazebo/models/person_walking/model.sdf", robot.getX(), robot.getY(), 0.0);


    ros::Publisher gazeboPublisher = node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    ros::Publisher statusPublisher = node.advertise<cw::status>("/status", 1000);
    ros::Subscriber statusSubscriber = node.subscribe("/status", 10, &statusCallback);
    sleep(1.0);

    gazebo_msgs::ModelState robotState;
    robotState.model_name = robot.getName();
    robotState.pose.position.x = robot.getX();
    robotState.pose.position.y = robot.getY();
    robotState.pose.orientation.x = 0.0;
    robotState.pose.orientation.y = 0.0;
    robotState.pose.orientation.z = -0.7;
    robotState.pose.orientation.w = 1.0;

    finderTransform.setOrigin(tf::Vector3(robot.getX(), robot.getY(), 0.0));
    finderTransform.setRotation(tf::Quaternion(0,0,-0.7,1));
    broadcaster.sendTransform(tf::StampedTransform(finderTransform, ros::Time::now(), "world", "finder_robot"));

    ros::Rate r(30);
    ROS_INFO("Start");
    while ((state != State::Final) && ros::ok() && !youDied)
    {
        if (state == State::Chase)
        {

            try {
                ros::Time commonTime;
                std::string error;
                listener.waitForTransform("finder_robot", "bonus", ros::Time(0), ros::Duration(1.0));
                listener.getLatestCommonTime("finder_robot", "bonus",commonTime, &error);
                listener.lookupTransform("finder_robot", "bonus", commonTime, lostRobotTransform);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                gazeboPublisher.publish(robotState);
                ros::Duration(1.0).sleep();
                continue;
            }

            dx = lostRobotTransform.getOrigin().getX();// - robotX;
            dy = lostRobotTransform.getOrigin().getY();// - robotY;
            distance = std::sqrt(dx * dx + dy * dy);
	    cw::status statusMessage;
            if (robot.getX() >= -10.0 && robot.getY() >= -10.0 && distance <= pickupRange)
            {
                ROS_INFO("Pass Check1");
                statusMessage.check1 = true;
                statusPublisher.publish(statusMessage);
                
            }
	    if (robot.getX() >= -6.0 && robot.getY() >= -6.0 && distance <= pickupRange)
            {
                ROS_INFO("Pass Check2");
                statusMessage.check2 = true;
                statusPublisher.publish(statusMessage);
                
            }
	   if (robot.getX() >= -2.0 && robot.getY() >= -2.0 && distance <= pickupRange)
            {
                ROS_INFO("Pass Check3");
                statusMessage.check3 = true;
                statusPublisher.publish(statusMessage);
                
            }
	    if (robot.getX() >= 2.0 && robot.getY() >= 2.0 && distance <= pickupRange)
            {
                ROS_INFO("Pass Check4");
                statusMessage.check4 = true;
                statusPublisher.publish(statusMessage);
                
            }
        if (robot.getX() >= 6.0 && robot.getY() >= 6.0 && distance <= pickupRange)
            {
                ROS_INFO("Pass Check5");
                statusMessage.check5 = true;
                statusPublisher.publish(statusMessage);
                
            }
	    if (robot.getX() >= 9.0 && robot.getY() >= 9.0 && distance <= pickupRange)
            {
                ROS_INFO("Final point");
                statusMessage.exit = true;
                statusPublisher.publish(statusMessage);
				state = State::Final;
				if (missionComplete)
				        {
				        ROS_INFO("Exit");
				        state = State::Final;
				        }
                
            }
	    	
			
			double dx1, dy1;
	    	for (int i = 1; i < 5; i++) {

	    	std::string zombieName = "zombies" + std::to_string(i);
	    	
				try {
		            ros::Time commonTime;
		            std::string error;
		            listener.waitForTransform("finder_robot", zombieName, ros::Time(0), ros::Duration(1.0));
		            listener.getLatestCommonTime("finder_robot", zombieName, commonTime, &error);
		            listener.lookupTransform("finder_robot", zombieName, ros::Time(0), zombieTransform);
		        }
		        catch (tf::TransformException &ex)
		        {
		            ROS_ERROR("%s",ex.what());
		            gazeboPublisher.publish(robotState);
		            ros::Duration(1.0).sleep();
		            continue;
		        }
		        
		        dx1 = zombieTransform.getOrigin().getX();
            	dy1 = zombieTransform.getOrigin().getY();
            	
	    		cw::status statusMessage; 
            
		        if (std::abs(dx1) < hookRange && std::abs(dy1) < hookRange)
		        {
		            ROS_INFO("Lumpen has hooked you");
		            statusMessage.is_hooked = true;
		            
		            statusPublisher.publish(statusMessage);
		            ros::Duration(1.0).sleep();
		            youDied =true;
		            
		        }
            }
	   
            robot.updatePosition(dx, dy, distance);
            robotState.pose.position.x = robot.getX();
            robotState.pose.position.y = robot.getY();
            robotState.pose.orientation.z = sin(robot.getCurrentAngle() / 2);
            robotState.pose.orientation.w = cos(robot.getCurrentAngle() / 2);
        }

        finderTransform.setOrigin(tf::Vector3(robot.getX(), robot.getY(), 0.0));
        finderTransform.setRotation(tf::Quaternion(0,0,-0.7,1));
        broadcaster.sendTransform(tf::StampedTransform(finderTransform, ros::Time::now(), "world", "finder_robot"));

        gazeboPublisher.publish(robotState);
        r.sleep();
    }
    return 0;
} 
