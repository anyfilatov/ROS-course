#include <string>
#include <fstream>
#include "ros/ros.h"
#include "transformation.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <std_msgs/String.h>


void pub_helper(const ros::Publisher &pub, double x, double y, double angle, std::string name)
{
    gazebo_msgs::ModelState msg;
    msg.model_name = name;
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.orientation.z = sin(angle / 2);
    msg.pose.orientation.w = cos(angle / 2);

    pub.publish(msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "helper_node");
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
    srv.request.model_name = "helper_robot";
    const double exit_coord_x = 3.0;
    const double exit_coord_y = -8.0;
    geometry_msgs::Pose pose;
    pose.position.x = exit_coord_x;
    pose.position.y = exit_coord_y;
    srv.request.initial_pose = pose;
    add_robot.call(srv);
    //Spawning finished

    ros::Publisher pub =
        nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10, true);

    ros::Publisher pub_found =
        nh.advertise<std_msgs::String>("/ex_topic", 10);

    ROS_INFO_STREAM("helper started");

    std::string base_frame_id = "world";
    std::string frame_id = "/helper";
    std::string lost_frame_id = "/lost";

    const double max_speed = 0.035;
    double speed_x = 0.0;
    double speed_y = 0.0;
    double new_angle = 0.0;
    double dist = 0.0;
    double x = exit_coord_x;
    double y = exit_coord_y;
    double old_angle = 0.0;
	bool is_found = false;
	double diff = 1.2;
    ros::Rate r(30);

    while (ros::ok())
    {
        broadcast_pose(x, y, base_frame_id, frame_id);
		if (!is_found)
		{
                double lost_x, lost_y;
                try
                {
                    take_pose(frame_id, lost_frame_id, lost_x, lost_y);
                }
                catch (const tf::TransformException &e)
                {
                    r.sleep();
                    continue;
                }

                //dist = calc_distance(x, y, lost_x + x, lost_y + y);
                new_angle = calc_angle(lost_x, lost_y);
                
                if (std::abs(lost_x) < diff && std::abs(lost_y) < diff)
                {
                    speed_x = max_speed * cos(new_angle);
                    speed_y = max_speed * sin(new_angle);
                }
                else
                {
                    is_found = true;
                }
        }
        else
            {
                dist = calc_distance(x, y, exit_coord_x, exit_coord_y);
                new_angle = calc_angle(exit_coord_x - x, exit_coord_y - y);

                if (dist < 1)
                {
                    std_msgs::String msg;
                    msg.data = std::string("Exit");
                    ROS_INFO_STREAM("exit");
                    pub_found.publish(msg);
                    int k=0;
                    while(k<10)
                    {
                    	k++;
                    	r.sleep();
                    }
                    break;
                }

                speed_x = max_speed * cos(new_angle);
                speed_y = max_speed * sin(new_angle);
            }

        if (fabs(old_angle - new_angle) < M_PI / 6)
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

        pub_helper(pub, x, y, old_angle, srv.request.model_name);
        r.sleep();
        ros::spinOnce();
    }

    ROS_INFO_STREAM("helper finished");
    return 0;
}


