#include <cstdlib>
#include <ctime>
#include <string>
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "utility.h"

void pub_marker(const ros::Publisher &pub, std::string frame_id, float x, float y);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lost_node");
    ros::NodeHandle nh;
    ros::Publisher pub =
        nh.advertise<visualization_msgs::Marker>("pt_topic", 10, true);

    std::srand((unsigned)std::time(0));

    std::string base_frame_id = "world";
    std::string frame_id = "/the_lost";
    std::string helper_frame_id = "/the_helper";

    const float max_speed = 0.5;
    float speed_x = 0.0;
    float speed_y = 0.0;
    float x = 0.0;
    float y = 0.0;

    ros::Rate r(30);
    float dt = (float)r.expectedCycleTime().toSec();

    while (ros::ok())
    {
        float sx, sy;
        broadcast_pose(x, y, base_frame_id, frame_id);
        take_pose(base_frame_id, helper_frame_id, sx, sy);

        if (!is_in_range(x, y, sx, sy, 1.0))
        {
            speed_x = 2.0 * max_speed * (((float)std::rand() / RAND_MAX) - 0.5);
            speed_y = 2.0 * max_speed * (((float)std::rand() / RAND_MAX) - 0.5);
        }
        else
        {
            speed_x = (x < sx ? max_speed : -max_speed);
            speed_y = (y < sy ? max_speed : -max_speed);
        }

        x += speed_x * dt;
        y += speed_y * dt;
        pub_marker(pub, frame_id, x, y);
        r.sleep();
    }
    return 0;
}


void pub_marker(const ros::Publisher &pub, std::string frame_id, float x, float y)
{
    visualization_msgs::Marker msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = ros::Time::now();
    msg.ns = "the_lost_point";
    msg.action = visualization_msgs::Marker::ADD;
    msg.pose.position.x = 0;
    msg.pose.position.y = 0;
    msg.pose.position.z = 0;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 1;
    msg.id = 0;
    msg.type = visualization_msgs::Marker::SPHERE;
    msg.scale.x = 0.5;
    msg.scale.y = 0.5;
    msg.scale.z = 0.5;
    msg.color.r = 1.0;
    msg.color.g = 0.0;
    msg.color.b = 0.0;
    msg.color.a = 1.0;
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    msg.points.push_back(p);

    while (pub.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
            return;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }

    pub.publish(msg);
}