#include <ros/ros.h>
#include <msg/msg.h>
#include <math.h>

msg::msg::_sal_type maxSalary = -1;
msg::msg::_name_type maxName;

void recieverCallback(const msg::msg & msg)
{
	msg::msg::_sal_type salary = msg.sal;
	msg::msg::_name_type name = msg.name;

	if(salary > maxSalary) {
		maxSalary = salary;
		maxName = name;
	}

	if(msg.fin) {
		ROS_INFO("%s: Hurray! My salary - %d !", maxName.c_str(), maxSalary);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "reciever");
	ROS_INFO("reciver is listening...");
	ros::NodeHandle nh;
	ros::Subscriber announcementsSubscriber = nh.subscribe("announcements", 0, recieverCallback);
	ros::spin();
	return 0;
}
