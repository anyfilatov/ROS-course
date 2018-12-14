#include "msg/msg.h"
#include "ros/ros.h"

void getMsg(const msg::msg& sentence){

	ROS_INFO_STREAM("msg: " << sentence.text);
	if(sentence.number % 2 == 0){
		ROS_INFO_STREAM("I received a message with text: " << sentence.text);	
	}	
}

int main (int argc, char *argv[]){
	ros::init(argc, argv, "subscriber_node");
	ROS_INFO_STREAM("Subscriber is ready.\n");
	ros::NodeHandle n;
	
	ros::Subscriber lab2_sub = n.subscribe("topic_name", 1000, &getMsg);
	
	ros::spin();
	return 0;
}
