#include <ros/ros.h>
#include <msg/msg.h>
#include <math.h>

typedef std::pair< std::string,long int > worker;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sender");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<msg::msg>("announcements", 100);

	ros::Rate loop_rate(1);

	std::vector< worker > workers;

	workers.push_back(worker("Victor", 74));
	workers.push_back(worker("Sergey", 46));
	workers.push_back(worker("Petr", 8070));
	sleep(1);

	for (unsigned int i = 0; i < workers.size(); i++) {
		msg::msg msg;
		msg.name = workers[i].first;
		msg.sal = workers[i].second;
		msg.fin = (i == (workers.size() - 1));
		pub.publish(msg);
		ROS_INFO("%s",workers[i].first.c_str());
		ros::spinOnce();
		loop_rate.sleep();
	}
}
