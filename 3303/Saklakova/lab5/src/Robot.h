#include "ros/ros.h"
#include <tf/transform_broadcaster.h> 
#include <tf/transform_listener.h> 
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"

using namespace std;

class Robot {
public:
	Robot(const char* name){
		ros::service::waitForService("gazebo/spawn_sdf_model");
		ros::ServiceClient add_robot = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
		gazebo_msgs::SpawnModel srv;
	 
		ifstream fin("/home/ubuntu/.gazebo/models/pioneer2dx/model.sdf"); 
	 
		string model;
		string buf;
		while(!fin.eof()){
			getline(fin, buf);
			model += buf + "\n";
		}
		srv.request.model_xml = model;
		srv.request.model_name = name;
		geometry_msgs::Pose pose;
		pose.position.x = (rand()%20) - 10;
		pose.position.y = (rand()%20) - 10;
		pose.orientation.z = 30;
		msg.pose = pose; 
		msg.model_name = name; 
		srv.request.initial_pose = pose;
		add_robot.call(srv);
		

		lab5_pub = nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
		robot = name;
		offset_x = offset_y = 0.0;
	}
	void move(double x, double y){	
		static tf::TransformBroadcaster br; 
		tf::Transform transform; 
		
		float angle = atan((getY() - y)/(getX() - x)); 
		if (getX() < x){ 
			angle += M_PI;
		}
		
		tf::Quaternion q(tf::Vector3(0, 0, 1), angle); 
		geometry_msgs::Quaternion q_odom;
		tf::quaternionTFToMsg(q, q_odom);
		msg.pose.orientation = q_odom;
		
		float dx = fabs(getX() - x) / 40.0; 
		float dy = fabs(getY() - y) / 40.0;
		dx *= getX() < x ? 1 : -1;
		dy *= getY() < y ? 1 : -1;
		ros::Rate rate(50);

		for (int i = 0; i < 40 && ros::ok(); i++){
			msg.pose.position.x += dx; 
			msg.pose.position.y += dy;
			lab5_pub.publish(msg);

			ros::spinOnce();
			rate.sleep();

			transform.setOrigin(tf::Vector3(getX(), getY(), 0.0)); 
			tf::Quaternion q; 
			q.setRPY(0, 0, 0); 
			transform.setRotation(q); 
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", robot)); 
		}
	}
	void move(float x, float y, float distance) { 
		float dx = fabs(getX() - x);
		float dy = fabs(getY() - y);

		dx = (dx < distance ? dx : distance)*(msg.pose.position.x < x ? 1 : - 1);
		dy = (dy < distance ? dy : distance)*(msg.pose.position.y < y ? 1 : - 1);

		move(getX() + dx, getY() + dy);
	}
	bool take_pos(const char* name){
		static tf::TransformListener listener;
		tf::StampedTransform transform;
		try{
			listener.lookupTransform(robot, name, ros::Time(0), transform);
			
			offset_x = transform.getOrigin().x() + getX();
			offset_y = transform.getOrigin().y() + getY();
			if (fabs(getX() - offset_x) < 3.0 && fabs(getY() - offset_y) < 3.0){
				return true;
			}
		}
		catch (tf::TransformException &ex){
			ROS_ERROR("%s", ex.what());
		}
		return false;
	}
	void broadcast_pos(float distance){
		if (distance > 0){
			move(offset_x, offset_y, distance);
		}else{		
			move(offset_x, offset_y);
		}
	}
	bool isPosition(float x, float y){
		if (fabs(getX() - x) < 0.1 && fabs(getY() - y) < 0.1){
			return true;
		}
		return false;
	}
	float getX(){
		return msg.pose.position.x;
	}
	float getY(){
		return msg.pose.position.y;
	}

protected:
	const char* robot;
	float offset_x;
	float offset_y;
	ros::NodeHandle nh;
	ros::Publisher lab5_pub; 
	gazebo_msgs::ModelState msg; 
};
