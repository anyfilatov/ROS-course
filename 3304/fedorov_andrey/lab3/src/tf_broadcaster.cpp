#include <iostream>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
using namespace ros;
std::string NodeName;
void Callback(const visualization_msgs::Marker msg)
{
	static tf::TransformBroadcaster br;
  	tf::Transform transform;
  	transform.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0));
  	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", NodeName));
}
int main(int argc, char** argv)
{
	init(argc, argv, "tf_broadcaster");
  	if (argc != 2)
	{
		ROS_ERROR("Need robot name as argument");
		return -1;
	};
  	NodeName = argv[1];
  	NodeHandle node;
  	Subscriber sub = node.subscribe(NodeName+"_topic", 10, &Callback);
	spin();
  	return 0;
}
