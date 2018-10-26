#include <ros/ros.h>
#include <user_message/FirstMessage.h>

typedef std::pair<std::string, int> TextNum;

std::string surname;
int max_money = -1;

void publish(const user_message::FirstMessage & message)
{
	if (message.money > max_money)
	{
		max_money = message.money;
		surname = message.surname;
	}
	
	if (message.money == -1)
	{
		if (max_money != -1) ROS_INFO("Worker %s is very happy", surname.c_str());
		else ROS_INFO("Data is empty");
		ros::shutdown();
	}
}

int main(int argc, char **argv){
	
	ros::init(argc, argv, "subscriber");
	ROS_INFO("Subscriber started\n");

	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("Salary", 100, publish);
	ros::spin();
	return 0;
}	



		

