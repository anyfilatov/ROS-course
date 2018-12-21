#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"


using namespace std;
bool isFound = false;
bool isFineshed = false;


void getStatus(const std_msgs::String::ConstPtr& msg) {

	

	if (msg->data == "200")	
		{
			
			isFound = true;

		}
	else if (msg->data == "205")	
		{
			
			isFineshed = true;
			isFound = true;
		}
	else {
		
		isFound = false;
		isFineshed = false;
	}
}


int main(int argc, char ** argv)
{
	srand (time(NULL));
	ros::init(argc, argv, "lostRobot");
	ros::NodeHandle n;
	ros::Subscriber getStatusInfo = n.subscribe("getStatusInfo_t", 10, getStatus);

	string model;
	string buf;
	ros::Rate loop_rate(40);
	tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::TransformListener listener;
	tf::StampedTransform findTr;
	ros::service::waitForService("gazebo/spawn_sdf_model");
	ros::ServiceClient add_robot =
	    n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
	gazebo_msgs::SpawnModel srv;

	//download robot's model
	ifstream fin("/home/valerosha/.gazebo/models/turtlebot3_burger/model.sdf");
	while (!fin.eof() && fin) {
		getline(fin, buf);
		model += buf + "\n";
	}
	srv.request.model_xml = model;
	srv.request.model_name = "lostRobot";
	geometry_msgs::Pose pose;
	int min = 0;
	int max = 2;
	 // random position 
	float start_x = min + rand() % (max - min);
	float start_y = min + rand() % (max - min);
	pose.position.x = start_x;
	pose.position.y = start_y;
	srv.request.initial_pose = pose;
	add_robot.call(srv);
	ros::Publisher pubgaz =
	    n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
	

	gazebo_msgs::ModelState msg;
	msg.model_name = "lostRobot";
	msg.pose.position.x = start_x;
	msg.pose.position.y = start_y;
	pubgaz.publish(msg);

	int check = 0;
	while (ros::ok())
	{
		transform.setOrigin(tf::Vector3(msg.pose.position.y, msg.pose.position.y, 0.0));
		transform.setRotation(tf::Quaternion(0, 0, 0, 1));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "lostRobot"));
		float old_tr_x = fabs(findTr.getOrigin().x());
		float old_tr_y = fabs(findTr.getOrigin().y());
		if (isFound)
		{

			try{
				listener.waitForTransform("lostRobot", "findRobot", ros::Time(0), ros::Duration(0.5));
				listener.lookupTransform("lostRobot", "findRobot", ros::Time(0), findTr);

			}
			catch (tf::TransformException &ex)
			{
				ros::Duration(1.0).sleep();
				continue;
		        }
	
			float dx = fabs(findTr.getOrigin().x()) / 200;
			float dy = fabs(findTr.getOrigin().y()) / 200;
			if(isFineshed){

				try{
				listener.waitForTransform("lostRobot", "findRobot", ros::Time(0), ros::Duration(0.5));
				listener.lookupTransform("lostRobot", "findRobot", ros::Time(0), findTr);

			}
			catch (tf::TransformException &ex)
			{
				ros::Duration(1.0).sleep();
				continue;
		        }
	
			float dx = fabs(findTr.getOrigin().x()) / 200;
			float dy = fabs(findTr.getOrigin().y()) / 200;
			if(findTr.getOrigin().x() > 0)
			{
				dx*= 1;
			}
			else 
			{
				dx*= -1;
			}
			if(findTr.getOrigin().y() > 0)
			{
				dy*= 1;
				
			}
			else 
			{
				dy*= -1;
			}

			for (int i = 0; i < 175; i++ ) {
				msg.pose.position.x += dx;
				msg.pose.position.y += dy;
				pubgaz.publish(msg);
				loop_rate.sleep();

				}

				return 0;
			}
			if(findTr.getOrigin().x() > 0)
			{
				dx*= 1;
			}
			else 
			{
				dx*= -1;
			}
			if(findTr.getOrigin().y() > 0)
			{
				dy*= 1;
				
			}
			else 
			{
				dy*= -1;
			}

		
			msg.pose.position.x += dx;
			msg.pose.position.y += dy;

			pubgaz.publish(msg);
			loop_rate.sleep();
		}

		else
		{
			loop_rate.sleep();

		}

		ros::spinOnce();
	}

	return 0;
}
