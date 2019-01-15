#include <cstdlib>
#include <ctime>
#include <string>
#include <fstream>
#include "transformation.h"
#include "ros/ros.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include "std_msgs/String.h"

bool isExit = false;
bool isFound = false;

void pubLost(const ros::Publisher &pub, double x, double y, double angle, std::string name)
{
    gazebo_msgs::ModelState msg;
    msg.model_name = name;
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = 1.5;
    msg.pose.orientation.z = sin(angle / 2);
    msg.pose.orientation.w = cos(angle / 2);

    pub.publish(msg);
}

void exitTopic(const std_msgs::String& msg)
{
    ROS_INFO_STREAM("OK");
    isExit = true;
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
    std::ifstream fin("/home/gregory/.gazebo/models/quadrotor/model.sdf");

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
    

    ros::Publisher pub =
        nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10, true);

    ros::Subscriber sub = nh.subscribe("/ex_topic", 10, &exitTopic);

    std::string base_frame_id = "world";
    std::string frame_id = "/lost";
    std::string search_frame_id = "/search";

    double maxSpeed = 0.035;
    const int maxSteps = 20;

    double speedX = 0.0;
    double speedY = 0.0;
    double x = 1.0;
    double y = 1.0;
    double distance = 0.0;
    double newAngle = 0.0;
    double oldAngle = 0.0;
    int steps = 0;
    int k = 30;


    ros::Rate r(30);


    while (ros::ok() && !isExit)
    {
        double searchX, searchY;
        broadcastPose(x, y, base_frame_id, frame_id);

        try
        {
            takePose(frame_id, search_frame_id, searchX, searchY);
        }
        catch (const tf::TransformException &e)
        {
            r.sleep();
            continue;
        }

        distance = calcDistance(x, y, searchX + x, searchY + y);
        newAngle = calcAngle(searchX, searchY);
        if (!isFound)
		{
            steps = (steps + 1) % maxSteps;
            if (steps == 1)
            {
                speedX = 0.3 * maxSpeed * (double)(std::rand() % 2);
                speedY = 0.3 * maxSpeed * (double)(std::rand() % 2);
            }
            newAngle = calcAngle(speedX, speedY);

            if (distance < 1.8)
            {
                isFound = true;
            }
        }
         else
        {
            if (k > 0)
            {
                k--;
                speedX = 0;
                speedX = 0;
            }
            else
            {
                speedX = maxSpeed * cos(newAngle);
                speedY = maxSpeed * sin(newAngle);
			}
        }

        if (fabs(oldAngle - newAngle) < M_PI / 6)
        {
            x += speedX;
            y += speedX;
            oldAngle = newAngle;
        }
        else
        {
            if (newAngle > oldAngle)
            {
                oldAngle = (oldAngle + M_PI / 24 > M_PI ? 0 : oldAngle + M_PI / 24);
            }
            else
            {
                oldAngle = (oldAngle - M_PI / 24 < -M_PI ? 0 : oldAngle - M_PI / 24);
            }
        }

        pubLost(pub, x, y, oldAngle, srv.request.model_name);
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
