#include "ros/ros.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <math.h>
#include "sensor_msgs/LaserScan.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

using namespace std;
 

int sandId  ;
ros::Publisher pubgaz ;
ros::ServiceClient add_model;
gazebo_msgs::ModelState msg;
ros::Publisher cmd_vel_topic;
gazebo_msgs::SpawnModel srv;
//tf::TransformListener listener;


double PI = 3.1415926535897;

struct robot
{
	float p_x;	//position
	float p_y;
	float p_z;
	double roll;	//orientation
	double pitch;
	double yaw;
};
struct robot smart;

struct sand
{
	float p_x;	//position
	float p_y;
};
struct sand tmpSand;

void setPosition (struct robot *name, float p_x, float p_y, float p_z, double roll, double pitch, double yaw)
{
	name->p_x = p_x;
	name->p_y = p_y;
	name->p_z = p_z;
	name->roll = roll;
	name->pitch = pitch;
	name->yaw = yaw;
	ROS_INFO("robot: %.2f, %.2f, %.2f; rotation: %1.2f, %1.2f, %1.2f", name->p_x, name->p_y, name->p_z, name->roll, name->pitch, name->yaw); 
}

tf::StampedTransform transformPoint2(const tf::TransformListener &listener)  //listener tf smart_burger
{
	tf::StampedTransform transform;
	bool success = false;
	do
	{
		try{
		listener.lookupTransform("smart_burger/odom", "smart_burger/base_footprint", ros::Time(0), transform);
		tf::Quaternion q(transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		setPosition(&smart, floor(transform.getOrigin().x()*100)/100, floor(transform.getOrigin().y()*100)/100, floor(transform.getOrigin().z()*100)/100, roll, pitch, yaw);
		success = true;
		}
		catch(tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();	
		}
		sleep(1);
	}
	while(!success);
	return transform;
}



void addSand(float posX, float posY)
{
	int  timeAddSand =  rand() % 50;	
	ROS_INFO("timeAddSand: %d", timeAddSand);
	if  (timeAddSand%2 == 0)
	{
//Generate uniq name
		stringstream ss;
		ss<< sandId ;
		string st = ss.str();     
//Add sand	
       		srv.request.model_name = st;
        	geometry_msgs::Pose pose;
		pose.position.x = posX;
		pose.position.y = posY;
		pose.position.z = 0;
        	srv.request.initial_pose = pose;
       		add_model.call(srv);
        //Spawning finished

	//poinCoord.push_back(pose);

        
        	msg.model_name = st;
		msg.pose.position.x = posX;
		msg.pose.position.y = posY;
        	pubgaz.publish(msg);
		sandId +=1;
		ROS_INFO("sandId: %d", sandId);//cout<<"SandId"<< sandId <<"\n";
	}
}



void move (ros::Publisher cmd_vel_topic, bool stop)
{
	geometry_msgs::Twist vel_msg;
	float cur_x = round(smart.p_x);
	float cur_y = round(smart.p_y);
	if(stop)
	{
		vel_msg.linear.x = 0.0;
		
	}
	else {vel_msg.linear.x = 0.05;}
	
	vel_msg.linear.y = 0;
	vel_msg.linear.z = 0;
	vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
	vel_msg.angular.z = 0;
	cmd_vel_topic.publish(vel_msg);
	ROS_INFO("cur_x: %.2f", cur_x);
	sleep(1);
	tf::TransformListener listener;
	transformPoint2(listener);
	sleep(1);
	
}

void rotate(float angle, ros::Publisher cmd_vel_topic, bool clockwise)
{
	float speed = 10.0;
	//Converting from angles to radians
	float angular_speed = speed*2*PI/360;
	float relative_angle = angle*2*PI/360;
	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x=0;
        vel_msg.linear.y=0;
        vel_msg.linear.z=0;
        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
	if (clockwise) {vel_msg.angular.z = abs(angular_speed)*(-1);}
	else {vel_msg.angular.z = abs(angular_speed);}
	
	//Setting the current time for distance calculus
	float t0 = ros::Time::now().toSec();
	float current_angle = 0;
	if (relative_angle != 0) {
		while (current_angle < relative_angle)
		{
			cmd_vel_topic.publish(vel_msg);
			float t1 = ros::Time::now().toSec();
			ROS_INFO("My t1: %.2f", t1);
			ROS_INFO("My t0: %.2f", t0);
			current_angle = angular_speed *(t1 - t0);
			ROS_INFO("My current_angle: %.2f", current_angle);
			ROS_INFO("My relative_angle: %.2f", relative_angle);
			sleep(1);
		}
	}
	vel_msg.angular.z = 0;
	cmd_vel_topic.publish(vel_msg);
	tf::StampedTransform transform;
	tf::TransformListener listener;
	transformPoint2(listener);
	//ros::spinOnce();
}


void coordinates_sand()
{
	float cur_yaw = round(smart.yaw);	
	
	if( (cur_yaw >= 0) && (cur_yaw <= 180) )
	{
		tmpSand.p_x = smart.p_x;
		tmpSand.p_y = smart.p_y - 0.5;
	}
	if( (cur_yaw <= -0) && (cur_yaw >= -180) )
	{
		tmpSand.p_x = smart.p_x;
		tmpSand.p_y = smart.p_y + 0.5;
	}
	
	
	 ROS_INFO("sand_pose: %.2f, %.2f",tmpSand.p_x, tmpSand.p_y );	
	
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	tf::TransformListener listener;	
	bool dangerIsNear1 = false;
	bool dangerIsNear2 = false;
	bool stop = false;
	float sizeRange = scan->ranges.size();

	for (int i =0; i< 90; i++)
	{
		if (scan->ranges[i] < 1.3)
		{
			//Wall or dif object is near
			dangerIsNear1 = true;
			cout<<"Min size"<< scan->ranges[i] <<"\n";
		}
		else {dangerIsNear1 = false;}
	}
	ROS_INFO("dangerIsNear 1: %d",dangerIsNear1 );	
	
		
	for (int i =270; i< 360; i++)
	{
		if (scan->ranges[i] < 1.3)
		{
			//Wall or dif object is near
			dangerIsNear2 = true;
		}
		else {dangerIsNear2 = false;}
	}
	ROS_INFO("dangerIsNear 2: %d",dangerIsNear2 );
	if (dangerIsNear1 || dangerIsNear2) {stop = true;}
	ROS_INFO("stop: %d",stop);	
	if (!stop) 
		{
			transformPoint2(listener);
			coordinates_sand();			
			addSand(tmpSand.p_x, tmpSand.p_y);
			sleep(1);
			move (cmd_vel_topic, stop);
		}
	else {
		move (cmd_vel_topic, stop);		
		rotate(90.0, cmd_vel_topic, true);
	}
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "single_robot");
	ros::NodeHandle node;
	ros::Rate loop_rate(10);
	srand (time(NULL));
	int min = 1;
	int max = 5;
	sandId = 1; 
        pubgaz = node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
	add_model = node.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
	cmd_vel_topic = node.advertise<geometry_msgs::Twist>("smart_burger/cmd_vel", 1000);
	//ros::Subscriber sub = node.subscribe("smart_burger/odom", 100, chatterCallback);
	ifstream fin("/home/kris-grid/.gazebo/models/modelsand.sdf");
	//vector<geometry_msgs::Pose> poinCoord;//Vector with sands coord
        string model;
        string buf;
        while(!fin.eof()){
            getline(fin, buf);
            model += buf + "\n";
        }
        srv.request.model_xml = model;
	sleep(1.0);
	ros::Subscriber laserSub = node.subscribe("smart_burger/scan",30,scanCallback);
	ROS_INFO("-----------START----------");
//Add sand model.
	//gazebo_msgs::SpawnModel srv;
	

	/*while (ros::ok()){
		
//Add new sand - PosX and PosY -curr pos of our robot
		
		if (dangerIsNear) 
		{
			addSand(smart.p_x , smart.p_y, srv);
			move (cmd_vel_topic);
		}
		else {rotate(90.0, cmd_vel_topic, true);}
		ros::spin();
		
 	}*/
	ros::spin();
	return 0;
}
