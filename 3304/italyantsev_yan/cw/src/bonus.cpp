#include <random>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/ModelState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cw/picked.h>
#include <tf2/LinearMath/Quaternion.h>
#include "bonus_info.h"

bool bonusPicked = false;

void pickedCallback(const cw::picked::ConstPtr& message)
{
    if (!message->is_picked)
    {
        bonusPicked = true;
    }
}

void spawnBonus(ros::NodeHandle& node, const std::string& name, const std::string& model_path, double x, double y, double z)
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
    ros::init(argc, argv, "supplies");
    ros::NodeHandle node;
    //tf::Transform strayRobotTransform;
    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    double hookRange = 0.5;
    double dx, dy, distance;
    double goalX1, goalY1, goalX2, goalY2, goalX3, goalY3, goalX4, goalY4, goalX5, goalY5;
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
    std::uniform_real_distribution<> uniform_dist_x5(6.0, 10.0);
    std::uniform_real_distribution<> uniform_dist_y5(6.0, 10.0);

    BonusInfo bonus("supplies");
    goalX1 = uniform_dist_x1(engine);
    goalY1 = uniform_dist_y1(engine);
    goalX2 = uniform_dist_x2(engine);
    goalY2 = uniform_dist_y2(engine);
    goalX3 = uniform_dist_x3(engine);
    goalY3 = uniform_dist_y3(engine);
    goalX4 = uniform_dist_x4(engine);
    goalY4 = uniform_dist_y4(engine);
    goalX5 = uniform_dist_x5(engine);
    goalY5 = uniform_dist_y5(engine);

    spawnBonus(node, "supplies1z", "/home/osboxes/.gazebo/models/beer/model.sdf", goalX1,  goalY1, 1.0);
    spawnBonus(node, "supplies2", "/home/osboxes/.gazebo/models/beer/model.sdf", 	goalX2, goalY2, 1.0);
    spawnBonus(node, "supplies3", "/home/osboxes/.gazebo/models/beer/model.sdf", 	goalX3, goalY3, 1.0);
    spawnBonus(node, "supplies4", "/home/osboxes/.gazebo/models/beer/model.sdf", 	goalX4, goalY4, 1.0);
    spawnBonus(node, "supplies5", "/home/osboxes/.gazebo/models/beer/model.sdf", 	goalX5, goalY5, 1.0);


    ros::Publisher gazeboPublisher = node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    ros::Publisher pickedPublisher = node.advertise<cw::picked>("/picked", 1000);
    ros::Subscriber pickedSubscriber = node.subscribe("/picked", 10, &pickedCallback);
    sleep(1.0);

    gazebo_msgs::ModelState robotState;
    robotState.pose.position.x = 0.0;
    robotState.pose.position.y = 0.0;
    robotState.pose.position.z = 1.0;	
    robotState.pose.orientation.x = 0.0;
    robotState.pose.orientation.y = 0.0;
    robotState.pose.orientation.z = 0.0;
    robotState.pose.orientation.w = 1.0;

    
    /*strayRobotTransform.setOrigin(tf::Vector3(goalX, goalY, 1.0));
    strayRobotTransform.setRotation(tf::Quaternion(0.4,0.25,0,1));
    broadcaster.sendTransform(tf::StampedTransform(strayRobotTransform, ros::Time::now(), "world", "supplies"));*/
    
	
	ros::Rate rate(30);
    	ROS_INFO("Bonus Appear");
   	/*while (ros::ok())
	{
        ros::spinOnce();
	dx = goalX;
        dy = goalY;
        distance = std::sqrt(dx * dx + dy * dy);
		if(distance <= hookRange)
		{
		goalX = uniform_dist_x(engine);
                goalY = uniform_dist_y(engine);
		}
		if(bonusPicked)
		{
		ROS_INFO("Bonus have been Picked");
		bonusPicked = false;
		return(1);
		}
	}*/
   /* strayRobotTransform.setOrigin(tf::Vector3(goalX, goalY, 1.0));
    strayRobotTransform.setRotation(tf::Quaternion(0.4,0.1,0,1));
    broadcaster.sendTransform(tf::StampedTransform(strayRobotTransform, ros::Time::now(), "world", "supplies"));*/

	gazeboPublisher.publish(robotState);

        rate.sleep();
    
    return 0;
} 
