#include "ros/ros.h"
#include "secret_message/SecretMessage.h"
#include <cstdlib>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

void recieve(const secret_message::SecretMessage& message) {
	ROS_INFO("Kat heard a rumble: %s", message.message.c_str());
	if((message.number % 2) == 0) {
		secret_message::SecretMessage m;
		m.message = "I understand you";
		m.number = rand() % 100;
		ROS_INFO("Kat said:%s", m.message.c_str());
		
	}
	else ROS_INFO("Kat grunted knowingly");
	return;
}

int main(int args,  char **argv)
{
		ros::init(args, argv, "accomplice");
		ros::NodeHandle n;
		ros::Subscriber sub = n.subscribe("scout_channel", 10, recieve);
		ros::spin();
		return 0;
}
