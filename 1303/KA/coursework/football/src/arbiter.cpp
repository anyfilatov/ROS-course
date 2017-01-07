#include <ros/ros.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"


#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include <cmath>  
#include <cstdlib>
#include <ctime>

using namespace std;

class Arbiter {
private:
	double homeGoalX, homeGoalY, guestGoalX, guestGoalY;
	double homeGoalLineX, homeGoalLineY, guestGoalLineX, guestGoalLineY;
	string homeName, guestName;
	string homeNameGoalLine, guestNameGoalLine;

	gazebo_msgs::ModelState robotStateGoal1, robotStateGoal2, robotStateField;

	ros::NodeHandle &nh;
	ros::Publisher pub;

	void broadcastPosition(double x, double y, string name) {
		static tf::TransformBroadcaster br;
		tf::Transform transform;

		transform.setOrigin(tf::Vector3(x, y, 0.0));
		tf::Quaternion q;
		q.setRPY(0, 0, 0);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
	}

	void visualisationField() {
		string fieldName = "field";
		ros::service::waitForService("gazebo/spawn_sdf_model");
	    ros::ServiceClient addRobot = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
	    gazebo_msgs::SpawnModel srv;
	 
	    ifstream fin("/home/user/.gazebo/models/robocup_3Dsim_field/model.sdf");
 
		string model;
	    string buf;
	    while(!fin.eof()){
	        getline(fin, buf);
	        model += buf + "\n";
	    }

	    srv.request.model_xml = model;
	    srv.request.model_name = fieldName;
	    geometry_msgs::Pose pose;
	    srv.request.initial_pose = pose;
	    addRobot.call(srv);

	    //state...
	    robotStateField.model_name = fieldName;
	    robotStateField.pose.position.x = 0;
		robotStateField.pose.position.y = 0;
		robotStateField.pose.position.z = 0;
		robotStateField.pose.orientation.z = 0.0;
		robotStateField.pose.orientation.w = 0.0;
		// pub.publish(robotStateField);
		// ros::spinOnce();
	}

	void visualisationGoal1() {
		string goal1Name = "goal1";
		ros::service::waitForService("gazebo/spawn_sdf_model");
	    ros::ServiceClient addRobot = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
	    gazebo_msgs::SpawnModel srv;
	 
	    ifstream fin("/home/user/.gazebo/models/robocup_3Dsim_goal/model.sdf");
 
		string model;
	    string buf;
	    while(!fin.eof()){
	        getline(fin, buf);
	        model += buf + "\n";
	    }

	    srv.request.model_xml = model;
	    srv.request.model_name = goal1Name;
	    geometry_msgs::Pose pose;
	    srv.request.initial_pose = pose;
	    addRobot.call(srv);

	    //state...
	    robotStateGoal1.model_name = goal1Name;
	    robotStateGoal1.pose.position.x = 15;
		robotStateGoal1.pose.position.y = 0;
		robotStateGoal1.pose.position.z = 0;
		robotStateGoal1.pose.orientation.z = 0.0;
		robotStateGoal1.pose.orientation.w = 0.0;
		// pub.publish(robotStateGoal1);
		// ros::spinOnce();
	}

	void visualisationGoal2() {
		string goal1Name = "goal2";
		ros::service::waitForService("gazebo/spawn_sdf_model");
	    ros::ServiceClient addRobot = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
	    gazebo_msgs::SpawnModel srv;
	 
	    ifstream fin("/home/user/.gazebo/models/robocup_3Dsim_goal/model.sdf");
 
		string model;
	    string buf;
	    while(!fin.eof()){
	        getline(fin, buf);
	        model += buf + "\n";
	    }

	    srv.request.model_xml = model;
	    srv.request.model_name = goal1Name;
	    geometry_msgs::Pose pose;
	    srv.request.initial_pose = pose;
	    addRobot.call(srv);

	    //state...
	    robotStateGoal2.model_name = goal1Name;
	    robotStateGoal2.pose.position.x = -15;
		robotStateGoal2.pose.position.y = 0;
		robotStateGoal2.pose.position.z = 0;
		robotStateGoal2.pose.orientation.z = 3.14 / 2;
		robotStateGoal2.pose.orientation.w = 0;
		// pub.publish(robotStateGoal2);
		// ros::spinOnce();
	}

public:
	Arbiter(ros::NodeHandle &nh, string h, string g) : nh(nh) {
		homeGoalX = -14.0;
		homeGoalY = 0.0;
		guestGoalX = 14.0;
		guestGoalY = 0.0;
		homeName = h;
		guestName = g;

		homeGoalLineX = -15.0;
		homeGoalLineY = 0;
		guestGoalLineX = 15.0;
		guestGoalLineY = 0;

		string postfix = "_goal-line";
		homeNameGoalLine = homeName + postfix;
		guestNameGoalLine = guestName + postfix;

		pub = nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
	}

	~Arbiter(){}

	void broadcastGoal() {
		ros::Rate rate(100);
		while (nh.ok()) {

			broadcastPosition(homeGoalX, homeGoalY, homeName);
			broadcastPosition(guestGoalX, guestGoalY, guestName);

			broadcastPosition(homeGoalLineX, homeGoalLineY, homeNameGoalLine);
			broadcastPosition(guestGoalLineX, guestGoalLineY, guestNameGoalLine);

			pub.publish(robotStateField);
			pub.publish(robotStateGoal1);
			pub.publish(robotStateGoal2);

			ros::spinOnce();
			rate.sleep();
		}
	}

	void visualisation() {
		visualisationField();
		visualisationGoal1();
		visualisationGoal2();
		ros::spinOnce();
	}
	
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "arbiter");
	if (argc < 3) {
		ROS_ERROR("Not specified arbiter's arguments!");
		return -1;
	}

	ros::NodeHandle nh;
	Arbiter arbiter(nh, argv[1], argv[2]);
	arbiter.visualisation();
	arbiter.broadcastGoal();
	
	ros::spin();
	return 0;
}