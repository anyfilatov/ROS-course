#include <string>
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "transformation.h"

void pub_helper(const ros::Publisher &pub, std::string frame_id, double x, double y)
{
	uint32_t shape = visualization_msgs::Marker::SPHERE;
    visualization_msgs::Marker msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = ros::Time::now();

    msg.ns = "helper_robot";
	msg.id = 0;
	msg.type = shape;

    msg.action = visualization_msgs::Marker::ADD;

    msg.pose.position.x = 0;
    msg.pose.position.y = 0;
    msg.pose.position.z = 0.5;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 1;
    

	msg.scale.x = 1.0;
    msg.scale.y = 1.0;
    msg.scale.z = 1.0;
    msg.color.r = 1.0f;
    msg.color.g = 1.0f;
    msg.color.b = 0.0f;
    msg.color.a = 1.0;

    while (pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return;
      }

      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    msg.points.push_back(point);
    pub.publish(msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "helper_node");
    ros::NodeHandle nh;
    ros::Publisher pub =
        nh.advertise<visualization_msgs::Marker>("pt_topic", 10, true);

    ROS_INFO_STREAM("Helper is started");

	std::string lost_frame_id = "/lost";
    std::string helper_frame_id = "/helper";
    std::string base_frame_id = "/world";

    const double max_sp = 0.31;
    const double exit_coord_x = 5.0;
    const double exit_coord_y = -5.0;
    double speed_x = 0.0;
    double speed_y = 0.0;
    double x = -4.0;
    double y = -4.0;

    ros::Rate r(30);
    double dt = (double)r.expectedCycleTime().toSec();
	
	bool isFound = false;
    while (ros::ok())
    {
        double lost_x, lost_y;
        broadcast_pose(x, y, base_frame_id, helper_frame_id);
        listen_pose(lost_x, lost_y, base_frame_id, lost_frame_id);

        if (!is_beside(x, y, lost_x, lost_y, 1.0))
        {
            speed_x = 0.1 * (x < lost_x ? max_sp : -max_sp);
            speed_y = 0.1 * (y < lost_y ? max_sp : -max_sp);
        }
        else
        {
            if (is_beside(x, y, exit_coord_x, exit_coord_y, 1.0))
            {
                break;
            }

            speed_x = 0.1 * (x < exit_coord_x ? max_sp : -max_sp);
            speed_y = 0.1 * (y < exit_coord_y ? max_sp : -max_sp);
        }

        x += speed_x;
        y += speed_y;

        pub_helper(pub, helper_frame_id, x, y);
        r.sleep();
    }

    ROS_INFO_STREAM("helper finished");
    return 0;
}
