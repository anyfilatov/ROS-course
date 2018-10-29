#include "ros/ros.h"
#include "msg/Message.h"
#include "time.h"
#include "sstream"

using namespace ros;
using namespace std;

void read_msg(const msg::Message & msg)
{
    float cord_x, cord_y;
    int currcoord = msg.cords.find(" ");
    cord_x = atof(msg.cords.substr(0, currcoord).c_str());
    cord_y = atof(msg.cords.substr(currcoord + 1, msg.cords.size() - 1 - currcoord).c_str());
    ROS_INFO("Coordination recieve:\n"
			"x = %f y = %f\n\n",
			cord_x, cord_y);
}

int main(int argc, char **argv)
{
	srand(time(0));
	init(argc, argv, "reader");
	NodeHandle n;
	Subscriber sub = n.subscribe("channel", 10, read_msg);
	ros::spin();
	return 0;
}
