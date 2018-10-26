#include <ros/ros.h>
#include <my_message/Message1.h>

std::string text;
my_message::Message1::_number_type maxNum = 0;
void reader(const my_message::Message1 & message) {
	my_message::Message1::_number_type number = message.number;
	
	ROS_INFO("Received a message with text %s. \n", message.text.c_str());
	
	if(number > maxNum) {
		maxNum = number;
		text = message.text;
	}
	
	if(message.number == -1) {
		ROS_INFO("The biggest salary is %d. Family of worker is %s. \n", maxNum, text.c_str());
		ros::shutdown();
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "reader");
	ROS_INFO_STREAM("Reader is ready \n");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("Name", 10, reader);
	ros::spin();
	return 0;
}
