#include <ros/ros.h>
#include <my_message/Message1.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>

typedef std::pair< std::string, int > TextNum;

int main(int argc, char **argv) {
	
	ros:: init(argc, argv, "writer");
	
	ROS_INFO_STREAM("Writer is ready.\n");
	
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<my_message::Message1>("Name",10);
	
	std::vector<TextNum> messageList;
	messageList.push_back(TextNum("Kokorin", 12000));
	messageList.push_back(TextNum("Mamaev", 9000));
	messageList.push_back(TextNum("Zolotov", 39500));
	messageList.push_back(TextNum("Petrov", 2100));
	messageList.push_back(TextNum("Boshirov", 2200));
	messageList.push_back(TextNum("Mishkin", 2100));
	messageList.push_back(TextNum("Chepiga", 2200));
	messageList.push_back(TextNum("Stop", -1));
	sleep(1);
	
	ros::Rate loop_rate(1);
	for(int i = 0; i < messageList.size(); i++) {
		my_message::Message1 message;
		message.text = messageList[i].first;
		message.number = messageList[i].second;
		pub.publish(message);
		
		ROS_INFO("%s, %d", messageList[i].first.c_str(), messageList[i].second);
		ros::spinOnce();
			loop_rate.sleep();
	}
	
	ROS_INFO_STREAM("Publishing is finished! \n");
	return 0;
}	