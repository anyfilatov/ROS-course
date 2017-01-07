#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <sstream>
#include <cmath>
#include <string>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;

geometry_msgs::Pose pose;

//целевые координаты
float target_x;
float target_y;

//если x и y совпадают с целевыми -> true
bool at_x = false;
bool at_y = false;

//лучи лазер-скана
//значение крайнего левого
float left_ = 0;
//крайнего правого
float right_ = 0;
//центрального
float central_ = 0;
//дистанция до препятствия (минимум из трех значений лазер-скана)
float distance_ = 0;

float rotation = 0;

float condition = 0;

ros::Publisher pub;

geometry_msgs::Twist msg;


//callback лазер-скана
void LSCallBack(const sensor_msgs::LaserScan::ConstPtr & msg)
{
	left_ = msg->ranges[msg->ranges.size() - 1];
	central_ = msg->ranges[msg->ranges.size() / 2];
	right_ = msg->ranges[0];

	distance_ = min( min(left_, right_), central_);
}

//получаем координаты (pose) и поворот (rotation)
void BPCallBack(const nav_msgs::Odometry::ConstPtr & msg)
{
	pose = msg->pose.pose;

	//получаем кватерниорн
	tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);

	double roll, pitch, yaw;
	//переводим его в радианы
	m.getRPY(roll, pitch, yaw);
	
	//получаем поворот
	rotation = yaw;
}

//поворачиваемся по х
void NormalizeY()
{
	ros::Rate rate(10);

	msg.angular.z = 1;
	msg.linear.x = 0;
	condition = pose.position.y < target_y ? 1.6 : -1.6;


	while (!(fabs( round(rotation * 10) / 10 - condition ) < 0.1))
	{
		pub.publish(msg);
		rate.sleep();
		ros::spinOnce();
		condition = pose.position.y < target_y ? 1.6 : -1.6;
	}

	msg.angular.z = 0;
	msg.linear.x = 1;
}

//поворачиваемся по y
void NormalizeX()
{
	ros::Rate rate(10);
	msg.angular.z = -1;
	msg.linear.x = 0;
	condition = pose.position.x < target_x ? 0 : 3;


	while (!(fabs( round(rotation * 10) / 10 - condition ) < 0.1))
	{
		pub.publish(msg);
		rate.sleep();
		ros::spinOnce();
		condition = pose.position.x < target_x ? 0 : 3;
	}

	msg.angular.z = 0;
	msg.linear.x = 1;
}

//обход препятствия
void GetRound()
{
	ros::Rate rate(10);
	msg.angular.z = 1;
	msg.linear.x = 0;

	//пока препятсвие близко, вызываем callback, где меняется distance_ -> делаем поворот
	while (distance_ < 0.5)
	{
		pub.publish(msg);
		rate.sleep();
		ros::spinOnce();
	}

	msg.angular.z = 0;
	msg.linear.x = 1;

	//если нет препятсвий, вызываем callback, где меняется distance_ -> двигаемся вперед
	while (distance_ >= 0.5)
	{
		pub.publish(msg);
		rate.sleep();
		ros::spinOnce();
	}
}

//двигаемся вперед
void GoAhead()
{
	ros::Rate rate(10);
	pub.publish(msg);
	rate.sleep();
	ros::spinOnce();
	cout << "pose=(" << pose.position.x << "; " << pose.position.y << ")" << endl;
}

int main(int argc, char ** argv)
{
	/* INITIALIZATION */

	stringstream ss;
	for (int i = 1; i < argc; i++)
		ss << argv[i] << ' ';

	//считываем целевые координаты
	ss >> target_x >> target_y;

	ros::init(argc, argv, "lab4");

	ros::NodeHandle nodeHandler;

	ros::Rate rate(10);

	pub = nodeHandler.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Subscriber BSSub = nodeHandler.subscribe("/base_scan", 1000, LSCallBack);
	ros::Subscriber BPSub = nodeHandler.subscribe("/base_pose_ground_truth", 1000, BPCallBack);

	//если никто ничего не паблишит -> ждем
	while (BSSub.getNumPublishers() == 0 || BPSub.getNumPublishers() == 0)
		sleep(1);

	while (ros::ok())
	{
		//поворачиваемся по х
		NormalizeX();

		//если Хсы не совпадают
		while (ros::ok() && !at_x)
		{
			//если препятствие блико
			if (distance_ < 0.5)
			{
				//мы не дошли до цели
				at_x = at_y = false;
				//делаем поворот
				GetRound();
				break;
			}

			//чекаем х
			at_x = round(fabs(pose.position.x - target_x) * 10) / 10 == 0.1;

			//если нет
			if (!at_x)
				//идем вперед
				GoAhead();
			else
				break;
		}

		//если вдруг х не совпадает 
		if (!at_x) 
			continue;

		//для у аналогично
		NormalizeY();

		while (ros::ok() && !at_y)
		{
			if (distance_ < 0.5)
			{
				at_x = at_y = false;
				GetRound();
				break;
			}

			at_y = round(fabs(pose.position.y - target_y) * 10) / 10 == 0.1;

			if (!at_y)
				GoAhead();
			else
				break;
		}

		if (!at_y) 
			continue;

		if (at_x && at_y)
		{
			cout << "CONGRATULATIONS!! MISSION COMPLETED" << endl;
			ros::shutdown();
			return 0;
		}
	}

	return 0;
}