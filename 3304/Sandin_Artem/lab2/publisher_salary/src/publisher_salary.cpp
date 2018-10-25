#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <user_message/FirstMessage.h>

typedef std::pair<std::string, int> Salary;

int main(int argc, char **argv){
	
	ros::init(argc, argv, "publisher_salary");

	ROS_INFO("Publisher started\n");

	ros::NodeHandle node;
	ros::Publisher pub = node.advertise<user_message::FirstMessage>("Salary",100);
	
	std::vector<Salary> mesList;
	std::ifstream file("../workspace/src/lab2/publisher_salary/src/data.txt");
	std::string zp, name;
	if (!file.is_open())
	{
		std::cout << "File is not open\n";
	}
	else while(!file.eof())
	{
		std::getline(file, name, ' ');
		std::getline(file, zp);
		int money = atoi(zp.c_str());
		mesList.push_back(Salary(name, money));
	}
	file.close();

	mesList.push_back(Salary("Stop", -1));
	sleep(1);
	ros::Rate rate(1);
	for(int i=0; i < mesList.size(); i++)
	{
		user_message::FirstMessage message;
		message.surname = mesList[i].first;
		message.money = mesList[i].second;
		pub.publish(message);
 		rate.sleep();
	}
	
	ROS_INFO("Publisher finished");
	ros::spinOnce();
	return 0;
}	



		

