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


int main(int argc, char ** argv)
{
	srand (time(NULL));
	ros::init(argc, argv, "findRobot");
        ros::NodeHandle n;


	ros::Publisher pub = n.advertise<std_msgs::String>("getStatusInfo_t", 10);
	ros::Rate loop_rate(40);
	tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::TransformListener listener;
	tf::StampedTransform lostTr;
        ros::service::waitForService("gazebo/spawn_sdf_model"); 
     	ros::ServiceClient add_robot = 
             n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
        gazebo_msgs::SpawnModel srv;
 	ifstream fin("/home/valerosha/.gazebo/models/turtlebot3_burger/model.sdf");

        string model;
        string buf;
        while(!fin.eof()){
            getline(fin, buf);
            model += buf + "\n";
        }
        srv.request.model_xml = model;
        srv.request.model_name = "findRobot";
	int min = 4;
	int max = 9;
	 // random position 
	float start_x = min + rand() % (max - min);
	float start_y = min + rand() % (max - min);
        geometry_msgs::Pose pose;
	pose.position.x = start_x;
	pose.position.y = start_y;
        srv.request.initial_pose = pose;
        add_robot.call(srv);


        ros::Publisher pubgaz = 
            n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
        

        gazebo_msgs::ModelState msg;
        msg.model_name = "findRobot";
       

	msg.pose.position.x = start_x;
	msg.pose.position.y = start_y;
        pubgaz.publish(msg);


	while (ros::ok())
	{

		if (pub.getNumSubscribers() == 0)
		{			
			continue;
		}
		transform.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0));
		transform.setRotation(tf::Quaternion(0, 0, 0, 1));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "findRobot"));

		if (!isFound)
		{
		
			try{
				listener.waitForTransform("findRobot", "lostRobot", ros::Time::now(), ros::Duration(0.5));
				listener.lookupTransform("findRobot", "lostRobot", ros::Time(0), lostTr);
			}
			catch (tf::TransformException &ex)
			{
				loop_rate.sleep();
				continue;
		        }
			//if lost robot was found
		
			if (fabs(lostTr.getOrigin().x()) < 2 && fabs(lostTr.getOrigin().y()) < 2)
			{
				std_msgs::String msg;
				msg.data = "200";
				isFound = true;

				pub.publish(msg);
				continue;
			}


			float dx = fabs(lostTr.getOrigin().x()) / 500;
			float dy = fabs(lostTr.getOrigin().y()) / 500;
			if(lostTr.getOrigin().x() > 0)
			{
				dx*= 1;
			}
			else 
			{
				dx*= -1;
			}
			if(lostTr.getOrigin().y() > 0)
			{
				dy*= 1;
				
			}
			else 
			{
				dy*= -1;
			}

			for (int i = 0; i < 500; i++ ) {
				msg.pose.position.x += dx;
				msg.pose.position.y += dy;
				pubgaz.publish(msg);
				loop_rate.sleep();

			}
		}
		else
		{
			float dx = fabs(msg.pose.position.x - start_x) / 500;
			float dy = fabs(msg.pose.position.y - start_y) / 500;
			if(msg.pose.position.x < start_x)
			{
				dx*= 1;
			}
			else 
			{
				dx*= -1;
			}
			if(msg.pose.position.y < start_y)
			{
				dy*= 1;
				
			}
			else 
			{
				dy*= -1;
			}
			while (msg.pose.position.x != start_x && msg.pose.position.y != start_y)
			{
				
				msg.pose.position.x += dx;
				msg.pose.position.y += dy;
				pubgaz.publish(msg);


				transform.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0));
				transform.setRotation(tf::Quaternion(0, 0, 0, 1));
				br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "findRobot"));
				loop_rate.sleep();

			}			cout<< " vseee"<< " \n";
				std_msgs::String msgn;
				msgn.data = "205";
				pub.publish(msgn);
			return 0;
		}

		ros::spinOnce();
	}

	return 0;
}

