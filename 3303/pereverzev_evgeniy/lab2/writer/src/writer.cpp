#include "ros/ros.h"
#include "msg/Message.h"
#include "time.h"
#include "sstream"

using namespace ros;
using namespace std;
int main(int argc, char **argv)
{
	srand(time(0));
	init(argc, argv, "writer");
	NodeHandle n;
	Publisher pub = n.advertise<msg::Message>("channel", 10);
	sleep(1);
	float cord_x, cord_y;
	stringstream ss;
	Rate loop_rate(1);
	for(int t = 0; t < 10; t++)
	{
		cord_x = (float)(rand())/RAND_MAX * 100 - 50;
		cord_y = (float)(rand())/RAND_MAX * 100 - 50;
		ss.str("");
		ss << cord_x << " " << cord_y;
		msg::Message ciphertext;
		ciphertext.cords = ss.str();
		ROS_INFO("Coordination send:\n"
			"x = %f y = %f\n\n",
			cord_x, cord_y);
		pub.publish(ciphertext);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
