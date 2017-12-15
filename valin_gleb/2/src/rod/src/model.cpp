#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <random>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "model");
	ros::NodeHandle node;
	ros::NodeHandle pnode("~");

	tf::TransformBroadcaster tfBroadcater;

	ros::Rate rate(30.0);

	std::default_random_engine gen;
	
	std::uniform_real_distribution<double> U1(-3.0, 3.0);
	tf::Quaternion rot(tf::Vector3(0, 1, 0), U1(gen));

	std::uniform_real_distribution<double> U2(0.0, 0.1);
	double speed = U2(gen);

	tf::Vector3 shift;

	if (ros::this_node::getName() != "/map") {
		shift = tf::Vector3(0, 0, 1);
	}

	while (node.ok())
	{
		tf::Transform transform;
		transform.setRotation(rot);
		transform.setOrigin(shift);

		rot = tf::Quaternion(tf::Vector3(0, 0, 1), speed) * rot;

		tf::StampedTransform stTr(
			transform,
			ros::Time::now(),
			ros::this_node::getName(),
			pnode.param("attached", std::string("default"))
		);

		tfBroadcater.sendTransform(stTr);

		rate.sleep();
	}
	
	return 0;
}
