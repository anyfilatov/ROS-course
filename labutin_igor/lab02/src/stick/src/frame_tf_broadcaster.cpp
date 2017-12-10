#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <sstream>

int main(int argc, char** argv){
  ros::init(argc, argv, "frame_tf_broadcaster");
  if (argc != 2) {
    ROS_ERROR("need id as argument");
    return -1;
  }
  std::string name = ros::this_node::getName();
  int id = atoi(argv[1]);
  std::stringstream ss;
  ss << id + 1;
  std::string next_id = ss.str();

  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;
  srand(time(NULL));
  tf::Quaternion rot = tf::createQuaternionFromRPY(0, 2 * M_PI * rand() / RAND_MAX, 0);
  double vel = rand() * 0.1 / RAND_MAX;

  ros::Rate rate(10.0);
  while (node.ok()) {
    transform.setRotation(rot);
    if (name != "/map") {
      transform.setOrigin(tf::Vector3(1.0, 0.0, 0.0));
    } else {
      transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    }
    rot = tf::Quaternion(tf::Vector3(1, 0, 0), vel) * rot;

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), name, "frame" + next_id));
    rate.sleep();
  }
  return 0;
};
