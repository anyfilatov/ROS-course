#include "ros/ros.h"
#include "secret_message/SecretMessage.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <vector>
#include <string>



int main(int args,  char **argv)
{
		ros::init(args, argv, "scout");
		ros::NodeHandle n;
		ros::Publisher pub = n.advertise<secret_message::SecretMessage>("scout_channel", 1000);
		while (0 == pub.getNumSubscribers()) {
          		ROS_INFO("Waiting for accomplice to connect");
          		ros::Duration(0.1).sleep();
      		}
		ros::Rate loop_rate(3);
		std::string messages[4];
		messages[0] = "No money, but you hold on!";
		messages[1] = "It seems to be starting to rain!";
		messages[2] = "We need to share toffee!";
		messages[3] = "And on our street will be a holiday!";
		for(int i = 0; i<4; i++)
		{
			secret_message::SecretMessage m;
			m.message = messages[i];
			m.number = rand() % 100;
			pub.publish(m);
			ROS_INFO("%s Stirlitz said", m.message.c_str());
			loop_rate.sleep();
			
		}
		sleep(3);
		ROS_INFO("At this Stirlets left");		
		return 0;
}
