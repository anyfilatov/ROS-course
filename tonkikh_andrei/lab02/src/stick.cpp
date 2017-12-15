#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "stick.h"

namespace lab02 {

static double rand_norm() {
  return double(std::rand()) / RAND_MAX;
}


Stick::Stick(ros::NodeHandle node, double length, std::string name, std::string frame):
    node(node), length(length), name(name), frame(frame) {}


int Stick::run() {
  static constexpr int FPS = 60;
  static constexpr double MAX_SPEED = 5 * M_PI;

  ros::Rate r(FPS);

  tf::TransformBroadcaster transform_broadcaster;

  auto quat_y = tf::createQuaternionFromRPY(0, rand_norm() * (M_PI / 2), 0);
  auto quat_z = tf::createIdentityQuaternion();

  // number between (- MAX_SPEED / 2) and (MAX_SPEED / 2)
  auto rot_speed = rand_norm() * MAX_SPEED - MAX_SPEED / 2;
  auto delta_quat_z = tf::createQuaternionFromRPY(0, 0, rot_speed / FPS);

  auto transform = tf::Transform(quat_y, tf::Vector3(0, 0, 0));
  auto transform_top = tf::Transform(tf::createIdentityQuaternion(), tf::Vector3(0, 0, length));

  while (ros::ok()) {
    quat_z *= delta_quat_z;
    transform.setRotation(quat_z * quat_y);

    transform_broadcaster.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), frame, name));
    transform_broadcaster.sendTransform(
        tf::StampedTransform(transform_top, ros::Time::now(), name, name + "/top"));

    r.sleep();
  }
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "stick"/*, ros::init_options::AnonymousName*/);
  ros::NodeHandle node("~");
  std::string name = ros::this_node::getName();
  std::srand(std::time(0) + std::hash<std::string>()(name));

  std::string frame;
  if (!node.getParam("frame", frame) || frame.empty()) {
    ROS_ERROR("No frame provided");
    return -1;
  }

  double length;
  node.param("length", length, STICK_DEFAULT_LENGTH);

  ROS_INFO("Stick node started with name '%s', length '%lf' and frame '%s'",
      name.c_str(), length, frame.c_str());

  lab02::Stick stick(node, length, name, frame);

  return stick.run();
}
