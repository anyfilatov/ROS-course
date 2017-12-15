#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <time.h>
#include <random>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "stick");
    ros::NodeHandle n;
    ros::NodeHandle pnode("~"); // to get params

    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    std::default_random_engine generator;
    generator.seed(time(NULL));
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    visualization_msgs::Marker marker;

    marker.ns = "stick";
    marker.id = pnode.param("id", 0);

    marker.header.frame_id = pnode.param("frame", std::string(""));

    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = distribution(generator);
    marker.color.g = distribution(generator);
    marker.color.b = distribution(generator);
    marker.color.a = 0.9; // a bit of transparancy

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    // cylinder center should be half its legth above the start of the frame
    marker.pose.position.z = 0.49;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.lifetime = ros::Duration();

    ros::Rate r(25);
    while (ros::ok()) {
        marker.header.stamp = ros::Time::now(); // update marker

        while (marker_pub.getNumSubscribers() < 1) {
            if (!ros::ok())
                return 0;
            ROS_WARN_ONCE("Please create a subscriber to the marker");
            sleep(1);
        }
        marker_pub.publish(marker);

        r.sleep();
    }
}
