#include <cstdlib>
#include <ctime>
#include <string>
#include <fstream>
#include "utility.h"
#include "ros/ros.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"

void pub_marker(const ros::Publisher &pub, std::string frame_id, float x, float y, float angle);

enum class LostState
{
    Waiting,
    Chase
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lost_node");
    ros::NodeHandle nh;
    ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::ServiceClient add_robot =
        nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;
    std::ifstream fin("/home/vlad/.gazebo/models/pioneer3at/model.sdf");

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

    std::srand((unsigned)std::time(0));

    std::string base_frame_id = "world";
    std::string frame_id = "/the_lost";
    std::string helper_frame_id = "/the_helper";

    const float max_speed = 1.0;
    float speed_x = 0.0;
    float speed_y = 0.0;
    float x = 0.0;
    float y = 0.0;
    float distance = 0.0;
    float angle = 0.0;
    LostState state = LostState::Waiting;

    ros::Rate r(30);
    float dt = (float)r.expectedCycleTime().toSec();

    while (ros::ok())
    {
        float sx, sy;
        broadcast_pose(x, y, base_frame_id, frame_id);
        try
        {
            take_pose(frame_id, helper_frame_id, sx, sy);
        }
        catch (const tf::TransformException &e)
        {
            continue;
        }
        distance = range(x, y, sx, sy);
         angle = calcAngle(sx - x, sy - y);
        switch (state)
        {
            case LostState::Waiting:
            {
                speed_x = 2.0 * max_speed * (((float)std::rand() / RAND_MAX) - 0.5);
                speed_y = 2.0 * max_speed * (((float)std::rand() / RAND_MAX) - 0.5);
                if (distance < 1.5)
                {
                    state = LostState::Chase;
                }
                break;
            }
            case LostState::Chase:
            {
                speed_x = max_speed * cos(angle);
                speed_y = max_speed * sin(angle);
                break;
            }
        }
        x += speed_x * dt;
        y += speed_y * dt;
        pub_marker(pub, x, y, angle);
        r.sleep();
    }
    return 0;
}

void pub_marker(const ros::Publisher &pub, float x, float y, float angle)
{
    gazebo_msgs::ModelState msg;
    msg.model_name = "lost_robot";
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.orientation.z = sin(angle / 2);
    msg.pose.orientation.w = cos(angle / 2);

    pub.publish(msg);
}