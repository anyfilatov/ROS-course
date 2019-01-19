#include <cstdlib>
#include <ctime>
#include <string>
#include <fstream>
#include "transformation.h"
#include "ros/ros.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include "std_msgs/String.h"

bool is_exit = false;
bool is_found = false;

void pub_lost(const ros::Publisher &pub, double x, double y, double angle, std::string name)
{
    gazebo_msgs::ModelState msg;
    msg.model_name = name;
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.orientation.z = sin(angle / 2);
    msg.pose.orientation.w = cos(angle / 2);

    pub.publish(msg);
}

void exit_topic(const std_msgs::String& msg)
{
    ROS_INFO_STREAM("OK");
    is_exit = true;
}


int main(int argc, char **argv)
{
	std::srand((unsigned)std::time(0));
    ros::init(argc, argv, "lost_node");
    ros::NodeHandle nh;
    ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::ServiceClient add_robot =
        nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;
    std::ifstream fin("/home/gregory/.gazebo/models/pioneer2dx/model.sdf");

    std::string model;
    std::string buf;
    while (std::getline(fin, buf))
    {
        model += buf + "\n";
    }
    srv.request.model_xml = model;
    srv.request.model_name = "lost_robot";
    geometry_msgs::Pose pose;
    srv.request.initial_pose = pose;
    add_robot.call(srv);
    //Spawning finished

    ros::Publisher pub =
        nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10, true);

    ros::Subscriber sub = nh.subscribe("/ex_topic", 10, &exit_topic);

    std::string base_frame_id = "world";
    std::string frame_id = "/lost";
    std::string helper_frame_id = "/helper";

    double max_speed = 0.035;
    const int turn_count = 30;
    double speed_x = 0.0;
    double speed_y = 0.0;
    double x = 0.0;
    double y = 0.0;
    double dist = 0.0;
    double new_angle = 0.0;
    double old_angle = 0.0;
    int st = 0;
    int k = 30;
	double diff = 1.7;

    ros::Rate r(30);


    while (ros::ok() && !is_exit)
    {
        double help_x, help_y;
        broadcast_pose(x, y, base_frame_id, frame_id);

        try
        {
            take_pose(frame_id, helper_frame_id, help_x, help_y);
        }
        catch (const tf::TransformException &e)
        {
            r.sleep();
            continue;
        }

        dist = calc_distance(x, y, help_x + x, help_y + y);
        new_angle = calc_angle(help_x, help_y);
        if (!is_found)
		{
            st = (st + 1) % turn_count;
            if (st == 1)
            {
                speed_x = 0.3 * max_speed * (double)(std::rand() % 2);
                speed_y = 0.3 * max_speed * (double)(std::rand() % 2);
            }
            new_angle = calc_angle(speed_x, speed_y);

            if (std::abs(help_x) < diff && std::abs(help_y) < diff)
            {
                is_found = true;
            }
        }
         else
        {
            if (k > 0)
            {
                k--;
                speed_x = 0;
                speed_y = 0;
            }
            else
            {
                speed_x = max_speed * cos(new_angle);
                speed_y = max_speed * sin(new_angle);
			}
        }

        if (fabs(old_angle - new_angle) < M_PI / 7)
        {
            x += speed_x;
            y += speed_y;
            old_angle = new_angle;
        }
        else
        {
            if (new_angle > old_angle)
            {
                old_angle = (old_angle + M_PI / 24 > M_PI ? 0 : old_angle + M_PI / 24);
            }
            else
            {
                old_angle = (old_angle - M_PI / 24 < -M_PI ? 0 : old_angle - M_PI / 24);
            }
        }

        pub_lost(pub, x, y, old_angle, srv.request.model_name);
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
