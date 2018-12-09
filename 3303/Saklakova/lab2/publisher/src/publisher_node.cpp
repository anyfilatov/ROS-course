#include "msg/msg.h"
#include "ros/ros.h"
#include <ctime>
#include <string>
#include <cstdlib>

int RandomNumb(){
	std::srand(std::time(NULL));	
	int numb = rand()%60;
	return numb;
}

std::string RandomMsg(int len){
	std::string str = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
	std::string newStr;
	int pos;
	std::srand(std::time(NULL));
	while (newStr.size() < len){
		pos = rand()%str.length();
		newStr += str[pos]; 
	}
	return newStr;
}

int main (int argc, char *argv[]){
	ros::init(argc, argv, "publisher_node");
	ROS_INFO_STREAM("Publisher is ready.\n");
	ros::NodeHandle n;
	ros::Publisher lab2_pub = n.advertise<msg::msg>("topic_name", 1000);
	ros::Rate sleep_rate(1);	
	msg::msg sentence;

	int len = 10;
	while(ros::ok()){
		sentence.text = RandomMsg(len);
		sentence.number = RandomNumb();
		ROS_INFO_STREAM("Text: " << sentence.text << " Number: " << sentence.number);
		lab2_pub.publish(sentence);
	
		sleep_rate.sleep();
	}	
	ros::spinOnce();
	return 0;
}
