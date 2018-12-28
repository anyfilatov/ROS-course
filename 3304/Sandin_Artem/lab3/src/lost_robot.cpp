#include <cstdlib>
#include <ctime>
#include <string>
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "transformation.h"

void pub_lost(const ros::Publisher &pub, std::string frame_id, double x, double y)
{
	uint32_t shape = visualization_msgs::Marker::SPHERE;    
	visualization_msgs::Marker msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = ros::Time::now();
    msg.ns = "lost_robot";
	msg.id = 1;
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
    msg.color.r = 0.8f;
    msg.color.g = 0.0f;
    msg.color.b = 0.5f;
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


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lost_node");
    ros::NodeHandle nh;
    ros::Publisher pub =
        nh.advertise<visualization_msgs::Marker>("pt_topic", 10, true);

    std::srand((unsigned)std::time(0));

    std::string lost_frame_id = "/lost";
    std::string helper_frame_id = "/helper";
    std::string base_frame_id = "/world";

    double max_sp = 0.3;
    double speed_x = 0.0;
    double speed_y = 0.0;
    double x = 0.0;
    double y = 0.0;

    ros::Rate r(30);

    while (ros::ok())
    {
        double help_x, help_y;
        broadcast_pose(x, y, base_frame_id, lost_frame_id);
        listen_pose(help_x, help_y, base_frame_id, helper_frame_id);

        if (!is_beside(x, y, help_x, help_y, 1.0))
        {
            speed_x = 0.6 * max_sp * (((double)std::rand() / RAND_MAX) - 0.5);
            speed_y = 0.6 * max_sp * (((double)std::rand() / RAND_MAX) - 0.5);
        }
        else
        {

            speed_x = 0.1 * (x < help_x ? max_sp : -max_sp);
            speed_y = 0.1 * (y < help_y ? max_sp : -max_sp);
	    if (is_beside(x, y, help_x, help_y, 0.9))
	      {
		speed_x = 0;
            	speed_y = 0;
	      }

        }

        x += speed_x;
        y += speed_y;
        pub_lost(pub, lost_frame_id, x, y);
        r.sleep();
    }
    return 0;
}
