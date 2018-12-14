#include "ros/ros.h"
#include <sstream>
#include <the_message/Txt.h>

void speakerCallback(const the_message::Txt & msg)
{
	std::size_t found;
	std::string line = msg.text.c_str();
	found = line.find_last_of(" ");
	int control_number = stoi(line.substr(found+1));
	if (control_number%2 == 0){
		ROS_INFO("Message: [%s]", msg.text.c_str());
	}

}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "listener");
	ROS_INFO_STREAM("I will only output meaningful messages!\n");
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("Speaker", 1000, speakerCallback);
	ros::spin();
	return 0;
}

