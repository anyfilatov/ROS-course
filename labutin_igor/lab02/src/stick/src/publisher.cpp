
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "publisher");
  if (argc != 2) {
    ROS_ERROR("need id as argument");
    return -1;
  }
  int id = atoi(argv[1]);

  ros::NodeHandle n;
  ros::Rate r(10.0);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  while (ros::ok())
  {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "frame" + std::string(argv[1]);
    marker.header.stamp = ros::Time::now();

    marker.ns = "publisher";
    marker.id = id;

    marker.type = visualization_msgs::Marker::CYLINDER;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0.5;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

    r.sleep();
  }
}
