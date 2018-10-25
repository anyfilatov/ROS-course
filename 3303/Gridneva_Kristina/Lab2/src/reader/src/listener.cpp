#include "ros/ros.h"
#include <sstream>
#include <one_message/Txt.h>

void speakerCallback(const one_message::Txt & msg)
{
	ROS_INFO("I heard: [%s]", msg.text.c_str());
}

int main(int argc, char **argv)
{
	setlocale(LC_CTYPE, "");
        ros::init(argc, argv, "listener");
	ROS_INFO_STREAM("Listener starts\n");
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("Speaker", 1000, speakerCallback);
	ros::spin();
	return 0;
}
