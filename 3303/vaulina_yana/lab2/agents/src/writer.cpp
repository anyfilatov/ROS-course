#include "ros/ros.h"
#include <my_message/Message1.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <ctime>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "writer");

	ROS_INFO_STREAM("Writer is ready.\n");
	ROS_INFO_STREAM("Writer began the attack.\n");
	
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<my_message::Message1>("Name", 30);
	sleep(1);


	srand( time( 0 ) );
	ros::Rate loop_rate(1);
	for(int i=0;i<20;i++){
		my_message::Message1 message;
		message.x = rand() % 5;
		message.y = rand() % 5;
		pub.publish(message);

		ROS_INFO("the coordinates of the rocket launch (%d,%d)",message.x ,message.y);
		loop_rate.sleep();
	}
	my_message::Message1 message;
	message.x = -1;
	message.y = -1;
	pub.publish(message);
	ROS_INFO_STREAM("Writer finished the attack\n");
	ros::spinOnce();
	return 0;
}
