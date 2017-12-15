#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "stick.h"


namespace lab02 {

class StickVisualizer {
public:
  static constexpr int FPS = 60;
  static constexpr double WIDTH = 0.1;

  StickVisualizer(ros::NodeHandle node, std::string frame, double length):
      node(node), frame(frame), length(length) {}

  int run() {
    ros::Rate r(FPS);

    auto stick_pub =
        node.advertise<visualization_msgs::Marker>(frame + "/stick_marker", 1);
    auto joint_pub =
        node.advertise<visualization_msgs::Marker>(frame + "/joint_marker", 1);

    // sticke marker
    visualization_msgs::Marker stick_marker;

    stick_marker.header.frame_id = frame;

    stick_marker.ns = frame;
    stick_marker.id = 0;

    stick_marker.type = visualization_msgs::Marker::CYLINDER;
    stick_marker.action = visualization_msgs::Marker::ADD;

    stick_marker.scale.x = WIDTH;
    stick_marker.scale.y = WIDTH;
    stick_marker.scale.z = length;

    stick_marker.color.r = double(rand()) / RAND_MAX;
    stick_marker.color.g = double(rand()) / RAND_MAX;
    stick_marker.color.b = double(rand()) / RAND_MAX;
    stick_marker.color.a = 1.0;

    stick_marker.lifetime = ros::Duration();

    stick_marker.pose.position.x = 0.0;
    stick_marker.pose.position.y = 0.0;
    stick_marker.pose.position.z = length / 2.0;
    stick_marker.pose.orientation.x = 0.0;
    stick_marker.pose.orientation.y = 0.0;
    stick_marker.pose.orientation.z = 0.0;
    stick_marker.pose.orientation.w = 1.0;

    // joint marker
    visualization_msgs::Marker joint_marker;

    joint_marker.header.frame_id = stick_marker.header.frame_id;

    joint_marker.ns = stick_marker.ns;
    joint_marker.id = stick_marker.id + 1;

    joint_marker.type = visualization_msgs::Marker::SPHERE;
    joint_marker.action = visualization_msgs::Marker::ADD;

    joint_marker.scale.x = 1.5 * WIDTH;
    joint_marker.scale.y = joint_marker.scale.x;
    joint_marker.scale.z = joint_marker.scale.x;

    joint_marker.color = stick_marker.color;

    stick_marker.lifetime = ros::Duration();

    while (ros::ok()) {
      stick_marker.header.stamp = ros::Time::now();
      stick_pub.publish(stick_marker);

      joint_marker.header.stamp = ros::Time::now();
      joint_pub.publish(joint_marker);

      r.sleep();
    }
  }

private:
  ros::NodeHandle node;
  const std::string frame;
  const double length;
};

}


int main(int argc, char** argv) {
  ros::init(argc, argv, "stick_visualizer", ros::init_options::AnonymousName);
  ros::NodeHandle node("~");
  std::srand(std::time(0) + std::hash<std::string>()(ros::this_node::getName()));

  std::string frame;
  if (!node.getParam("frame", frame)) {
    ROS_ERROR("No frame provided");
    return -1;
  }

  if (frame[0] != '/') {
    frame = "/" + frame;
  }

  double length;
  node.param(frame + "/length", length, STICK_DEFAULT_LENGTH);

  ROS_INFO("Stick Visualizer node started for frame '%s' with length '%lf'",
      frame.c_str(), length);
  lab02::StickVisualizer stick_visualizer(node, frame, length);

  return stick_visualizer.run();

}
