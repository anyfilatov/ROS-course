#include <string>
#include <fstream>
#include "ros/ros.h"
#include "utility.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <std_msgs/String.h>


void pub_marker(const ros::Publisher &pub, float x, float y, float angle);
void pub_found_topic(const ros::Publisher &pub);

enum class HelperState
{
    Chase,
    Convoy,
    AtExit
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "helper_node");
    ros::NodeHandle nh;
    ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::ServiceClient add_robot =
        nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;
    std::ifstream fin("/home/vlad/.gazebo/models/pioneer2dx/model.sdf");

    std::string model;
    std::string buf;
    while (std::getline(fin, buf))
    {
        model += buf + "\n";
    }
    srv.request.model_xml = model;
    srv.request.model_name = "helper_robot";
    geometry_msgs::Pose pose;
    srv.request.initial_pose = pose;
    add_robot.call(srv);
    //Spawning finished

    ros::Publisher pub =
        nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10, true);

    ros::Publisher pub_found =
        nh.advertise<std_msgs::String>("/found_topic", 10);

    ROS_INFO_STREAM("helper started");

    std::string base_frame_id = "world";
    std::string frame_id = "/the_helper";
    std::string lost_frame_id = "/the_lost";

    const float max_speed = 0.5;
    const float exit_x = 3.0;
    const float exit_y = -8.0;
    float speed_x = 0.0;
    float speed_y = 0.0;
    float angle = 0.0;
    float distance = 0.0;
    float x = exit_x;
    float y = exit_y;
    HelperState state = HelperState::Chase;

    ros::Rate r(30);
    float dt = (float)r.expectedCycleTime().toSec();

    while (ros::ok() && state != HelperState::AtExit)
    {
        broadcast_pose(x, y, base_frame_id, frame_id);
        switch (state)
        {
            case HelperState::Chase:
            {
                float lx, ly;
                try
                {
                    take_pose(frame_id, lost_frame_id, lx, ly);
                }
                catch (const tf::TransformException &e)
                {
                    r.sleep();
                    continue;
                }

                distance = range(x, y, lx + x, ly + x);
                angle = calcAngle(lx, ly);
                if (distance > 1.5)
                {
                    speed_x = max_speed * cos(angle);
                    speed_y = max_speed * sin(angle);
                }
                else
                {
                    state = HelperState::Convoy;
                }
                break;
            }
            case HelperState::Convoy:
            {
                distance = range(x, y, exit_x, exit_y);
                angle = calcAngle(exit_x - x, exit_y - y);
                if (distance < 2.0)
                {
                    state = HelperState::AtExit;
                    std_msgs::String msg;
                    msg.data = std::string("I'm at the exit");
                    pub_found.publish(msg);
                    break;
                }
                speed_x = max_speed * cos(angle);
                speed_y = max_speed * sin(angle);
                break;
            }
            case HelperState::AtExit:
            {
                speed_x = 0;
                speed_y = 0;
            }
        }

        x += speed_x * dt;
        y += speed_y * dt;
        pub_marker(pub, x, y, angle);
        r.sleep();
        ros::spinOnce();
    }

    ROS_INFO_STREAM("helper finished");
    return 0;
}

void pub_marker(const ros::Publisher &pub, float x, float y, float angle)
{
    gazebo_msgs::ModelState msg;
    msg.model_name = "helper_robot";
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.orientation.z = sin(angle / 2);
    msg.pose.orientation.w = cos(angle / 2);

    pub.publish(msg);
}
