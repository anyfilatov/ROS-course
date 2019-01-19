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


enum class State { Chase, Convoy, Final };

bool isHooked = false;

void statusCallback(const cw::status::ConstPtr& message)
{
    if (message->is_hooked)
    {
        isHooked = true;
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
    ros::init(argc, argv, "zombies");
    ros::NodeHandle node;
    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    tf::Transform strayRobotTransform;
    tf::StampedTransform finderRobotTransform;
    double dx1, dy1, dx2, dy2, dx3, dy3, dx4, dy4, distance1, distance2, distance3, distance4;
    double goalX1, goalY1, goalX2, goalY2, goalX3, goalY3, goalX4, goalY4, goalX5, goalY5;
    double hookRange = 0.5;
    State state = State::Chase;
    std::random_device rd;
    std::default_random_engine engine(rd());
    //area where robot travel
    std::uniform_real_distribution<> uniform_dist_x1(0.0, -10.0);
    std::uniform_real_distribution<> uniform_dist_y1(0.0, 10.0);
    std::uniform_real_distribution<> uniform_dist_x2(0.0, 10.0);
    std::uniform_real_distribution<> uniform_dist_y2(0.0, 10.0);
    std::uniform_real_distribution<> uniform_dist_x3(-10.0, 0.0);
    std::uniform_real_distribution<> uniform_dist_y3(-10.0, 0.0);
    std::uniform_real_distribution<> uniform_dist_x4(0.0, 10.0);
    std::uniform_real_distribution<> uniform_dist_y4(0.0, -10.0);


    // generate robot
    RobotInfo robot1("zombies1");
    RobotInfo robot2("zombies2");
    RobotInfo robot3("zombies3");
    RobotInfo robot4("zombies4");
    robot1.setPosition(-9.0, 9.0);
    robot2.setPosition(0.0, 9.0);
    robot3.setPosition(0.0, -9.0);
    robot4.setPosition(9.0, 0.0);
    goalX1 = uniform_dist_x1(engine);
    goalY1 = uniform_dist_y1(engine);
    goalX2 = uniform_dist_x2(engine);
    goalY2 = uniform_dist_y2(engine);
    goalX3 = uniform_dist_x3(engine);
    goalY3 = uniform_dist_y3(engine);
    goalX4 = uniform_dist_x4(engine);
    goalY4 = uniform_dist_y4(engine);


    	
	spawnRobot(node, "necromancer1", "/home/gregory/.gazebo/models/dumpster/model.sdf", -11.0, 11.0, 0.0);
	spawnRobot(node, "necromancer2", "/home/gregory/.gazebo/models/dumpster/model.sdf", 0.0, 11.0, 0.0);
	spawnRobot(node, "necromancer3", "/home/gregory/.gazebo/models/dumpster/model.sdf", 0.0, -11.0, 0.0);
	spawnRobot(node, "necromancer4", "/home/gregory/.gazebo/models/dumpster/model.sdf", 11.0, 0.0, 0.0);
	spawnRobot(node, "zombies1", "/home/gregory/.gazebo/models/person_standing/model.sdf", -9.0, 9.0, 0.0);
	spawnRobot(node, "zombies2", "/home/gregory/.gazebo/models/person_standing/model.sdf", 0.0, 9.0, 0.0);
	spawnRobot(node, "zombies3", "/home/gregory/.gazebo/models/person_standing/model.sdf", 0.0, -9.0, 0.0);
	spawnRobot(node, "zombies4", "/home/gregory/.gazebo/models/person_standing/model.sdf", 9.0, 0.0, 0.0);

    ros::Publisher gazeboPublisher = node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    ros::Publisher statusPublisher = node.advertise<cw::status>("/status", 1000);
    ros::Subscriber statusSupscriber = node.subscribe("/status", 10, &statusCallback);
    sleep(1.0);

    gazebo_msgs::ModelState robotState4;
    gazebo_msgs::ModelState robotState2;
    gazebo_msgs::ModelState robotState3;
    gazebo_msgs::ModelState robotState1;
//first
    robotState1.model_name = robot1.getName();
    robotState1.pose.position.x = robot1.getX();
    robotState1.pose.position.y = robot1.getY();
//second
    robotState2.model_name = robot2.getName();
    robotState2.pose.position.x = robot2.getX();
    robotState2.pose.position.y = robot2.getY();
//third
    robotState3.model_name = robot3.getName();
    robotState3.pose.position.x = robot3.getX();
    robotState3.pose.position.y = robot3.getY();
//fourth	     
    robotState4.model_name = robot4.getName();
    robotState4.pose.position.x = robot4.getX();
    robotState4.pose.position.y = robot4.getY();

    robotState1.pose.orientation.x = 0.0;
    robotState1.pose.orientation.y = 0.0;
    robotState1.pose.orientation.z = 0.0;
    robotState1.pose.orientation.w = 1.0;

    strayRobotTransform.setOrigin(tf::Vector3(robot1.getX(), robot1.getY(), 0.0));
    strayRobotTransform.setRotation(tf::Quaternion(0,0,0,1));
    broadcaster.sendTransform(tf::StampedTransform(strayRobotTransform, ros::Time::now(), "world", "zombies1"));
    
    strayRobotTransform.setOrigin(tf::Vector3(robot2.getX(), robot2.getY(), 0.0));
    strayRobotTransform.setRotation(tf::Quaternion(0,0,0,1));
    broadcaster.sendTransform(tf::StampedTransform(strayRobotTransform, ros::Time::now(), "world", "zombies2"));
    
    strayRobotTransform.setOrigin(tf::Vector3(robot3.getX(), robot3.getY(), 0.0));
    strayRobotTransform.setRotation(tf::Quaternion(0,0,0,1));
    broadcaster.sendTransform(tf::StampedTransform(strayRobotTransform, ros::Time::now(), "world", "zombies3"));
    
    strayRobotTransform.setOrigin(tf::Vector3(robot4.getX(), robot4.getY(), 0.0));
    strayRobotTransform.setRotation(tf::Quaternion(0,0,0,1));
    broadcaster.sendTransform(tf::StampedTransform(strayRobotTransform, ros::Time::now(), "world", "zombies4"));

    ros::Rate rate(30);
    ROS_INFO("Start");
    while ((state != State::Final) && ros::ok())
    {
        if (state == State::Chase)
        {
         	ros::spinOnce();   
            // randomMove
            if (isHooked)
            {
                ROS_INFO("Is hooked");
                isHooked = false;
                return(1);
            }
            else
            {
                dx1 = goalX1 - robot1.getX();
                dy1 = goalY1 - robot1.getY();
                distance1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
				dx2 = goalX2 - robot2.getX();
                dy2 = goalY2 - robot2.getY();
                distance2 = std::sqrt(dx2 * dx2 + dy2 * dy2);
				dx3 = goalX3 - robot3.getX();
                dy3 = goalY3 - robot3.getY();
                distance3 = std::sqrt(dx3 * dx3 + dy3 * dy3);
				dx4 = goalX4 - robot4.getX();
                dy4 = goalY4 - robot4.getY();
                distance4 = std::sqrt(dx4 * dx4 + dy4 * dy4);
                
                if (distance1 <= robot1.getDistancePrecision())
                {
                    goalX1 = uniform_dist_x1(engine);
                    goalY1 = uniform_dist_y1(engine);
                    
                }
                else
                {	
                    robot1.updatePosition(dx1, dy1, distance1);
                    robotState1.pose.position.x = robot1.getX();
                    robotState1.pose.position.y = robot1.getY();
                    robotState1.pose.orientation.z = sin(robot1.getCurrentAngle() / 2);
                    robotState1.pose.orientation.w = cos(robot1.getCurrentAngle() / 2);
		    		rate.sleep();	
		        }	
				
				if (distance2 <= robot2.getDistancePrecision())
				        {
				            goalX2 = uniform_dist_x2(engine);
				            goalY2 = uniform_dist_y2(engine);
				            
				        }
				else
				{
					robot2.updatePosition(dx2, dy2, distance2);
				            robotState2.pose.position.x = robot2.getX();
				            robotState2.pose.position.y = robot2.getY();
				            robotState2.pose.orientation.z = sin(robot2.getCurrentAngle() / 2);
				            robotState2.pose.orientation.w = cos(robot2.getCurrentAngle() / 2);
				}
				if (distance3 <= robot3.getDistancePrecision())
				        {
				            goalX3 = uniform_dist_x3(engine);
				            goalY3 = uniform_dist_y3(engine);
				            
				        }
				else
				{
					robot3.updatePosition(dx3, dy3, distance3);
				            robotState3.pose.position.x = robot3.getX();
				            robotState3.pose.position.y = robot3.getY();
				            robotState3.pose.orientation.z = sin(robot3.getCurrentAngle() / 2);
				            robotState3.pose.orientation.w = cos(robot3.getCurrentAngle() / 2);
				}
				if (distance4 <= robot4.getDistancePrecision())
				        {
				            goalX4 = uniform_dist_x4(engine);
				            goalY4 = uniform_dist_y4(engine);
				            
				        }
				else
				{
					robot4.updatePosition(dx4, dy4, distance4);
				            robotState4.pose.position.x = robot4.getX();
				            robotState4.pose.position.y = robot4.getY();
				            robotState4.pose.orientation.z = sin(robot4.getCurrentAngle() / 2);
				            robotState4.pose.orientation.w = cos(robot4.getCurrentAngle() / 2);
				}
				       
				    }

				}


        strayRobotTransform.setOrigin(tf::Vector3(robot1.getX(), robot1.getY(), 0.0));
    	strayRobotTransform.setRotation(tf::Quaternion(0,0,0,1));
    	broadcaster.sendTransform(tf::StampedTransform(strayRobotTransform, ros::Time::now(), "world", "zombies1"));
    	
    	
    	strayRobotTransform.setOrigin(tf::Vector3(robot2.getX(), robot2.getY(), 0.0));
    	strayRobotTransform.setRotation(tf::Quaternion(0,0,0,1));
    	broadcaster.sendTransform(tf::StampedTransform(strayRobotTransform, ros::Time::now(), "world", "zombies2"));
    	
    	strayRobotTransform.setOrigin(tf::Vector3(robot3.getX(), robot3.getY(), 0.0));
    	strayRobotTransform.setRotation(tf::Quaternion(0,0,0,1));
    	broadcaster.sendTransform(tf::StampedTransform(strayRobotTransform, ros::Time::now(), "world", "zombies3"));
    	
    	strayRobotTransform.setOrigin(tf::Vector3(robot4.getX(), robot4.getY(), 0.0));
    	strayRobotTransform.setRotation(tf::Quaternion(0,0,0,1));
        broadcaster.sendTransform(tf::StampedTransform(strayRobotTransform, ros::Time::now(), "world", "zombies4"));

        gazeboPublisher.publish(robotState1);

		gazeboPublisher.publish(robotState2);
		gazeboPublisher.publish(robotState3);
		gazeboPublisher.publish(robotState4);
		
		rate.sleep();
		
        
    }
    return 0;
} 
