#include <cstdlib>
#include <ctime>
#include <string>
#include <fstream>
#include "utility.h"
#include "ros/ros.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include "std_msgs/String.h"

void pub_marker(const ros::Publisher &pub, float x, float y, float angle);
void found_topic_callback(const std_msgs::String& msg);

bool found_exit = false;

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

    ros::Subscriber sub = nh.subscribe("/found_topic", 10, &found_topic_callback);

    std::srand((unsigned)std::time(0));

    std::string base_frame_id = "world";
    std::string frame_id = "/the_lost";
    std::string helper_frame_id = "/the_helper";

    const float max_speed = 0.5;
    const int max_steps = 30;
    const float da = M_PI / 20;
    float speed_x = 0.0;
    float speed_y = 0.0;
    float x = 0.0;
    float y = 0.0;
    float distance = 0.0;
    float angle = 0.0;
    float a = 0.0;
    int wait = 60;
    int steps = 0;
    LostState state = LostState::Waiting;

    ros::Rate r(30);
    float dt = (float)r.expectedCycleTime().toSec();

    while (ros::ok() && !found_exit)
    {
        float sx, sy;
        broadcast_pose(x, y, base_frame_id, frame_id);
        try
        {
            take_pose(frame_id, helper_frame_id, sx, sy);
        }
        catch (const tf::TransformException &e)
        {
            r.sleep();
            continue;
        }
        distance = range(x, y, sx + x, sy + y);
        angle = calcAngle(sx, sy);
        switch (state)
        {
            case LostState::Waiting:
            {
                steps = (steps + 1) % max_steps;
                if (steps == 0)
                {
                    speed_x = 0.3 * max_speed * (float)(std::rand() % 2);
                    speed_y = 0.3 * max_speed * (float)(std::rand() % 2);
                }
                angle = calcAngle(speed_x, speed_y);
                if (distance < 1.8)
                {
                    state = LostState::Chase;
                }
                break;
            }
            case LostState::Chase:
            {
                if (wait > 0)
                {
                    wait--;
                    speed_x = 0;
                    speed_y = 0;
                }
                else
                {
                    speed_x = max_speed * cos(angle);
                    speed_y = max_speed * sin(angle);
                }
                break;
            }
        }
        if (fabs(a - angle) < 4 * da)
        {
            x += speed_x * dt;
            y += speed_y * dt;
            a = angle;
        }
        else
        {
            if (angle > a)
            {
                a = (a + da > M_PI ? 0 : a + da);
            }
            else
            {
                a = (a - da < -M_PI ? 0 : a - da);
            }
        }
        // ROS_INFO_STREAM("a=" << a << ", angle=" << angle);
        pub_marker(pub, x, y, a);
        r.sleep();
        ros::spinOnce();
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

void found_topic_callback(const std_msgs::String& msg)
{
    ROS_INFO_STREAM("I've heard: " << msg.data);
    found_exit = true;
}