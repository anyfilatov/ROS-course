#include <string>
#include <fstream>
#include "ros/ros.h"
#include "transformation.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <std_msgs/String.h>


void pub_search(const ros::Publisher &pub, double x, double y, double angle, std::string name)
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

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "search_node");
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
    srv.request.model_name = "search_robot";
    const double exitCoordX = -4.0;
    const double exitCoordY = -4.0;
    geometry_msgs::Pose pose;
    pose.position.x = exitCoordX;
    pose.position.y = exitCoordY;
    pose.position.z = 1.5;
    srv.request.initial_pose = pose;
    add_robot.call(srv);
    //Spawning finished

    ros::Publisher pub =
        nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10, true);

    ros::Publisher pub_found =
        nh.advertise<std_msgs::String>("/ex_topic", 10);

    ROS_INFO_STREAM("search started");

    std::string base_frame_id = "world";
    std::string frame_id = "/search";
    std::string lost_frame_id = "/lost";

    const double maxSpeed = 0.035;
    const double da = M_PI / 20;
    double speedX = 0.0;
    double speedY = 0.0;
    double newAngle = 0.0;
    double dist = 0.0;
    double x = exitCoordX;
    double y = exitCoordY;
    double oldAngle = 0.0;
	bool isFound = false;
    ros::Rate r(30);

    while (ros::ok())
    {
        broadcastPose(x, y, base_frame_id, frame_id);
		if (!isFound)
		{
                double lostX, lostY;
                try
                {
                    takePose(frame_id, lost_frame_id, lostX, lostY);
                }
                catch (const tf::TransformException &e)
                {
                    r.sleep();
                    continue;
                }

                dist = calcDistance(x, y, lostX + x, lostY + y);
                newAngle = calcAngle(lostX, lostY);
                
                if (dist > 1)
                {
                    speedX = maxSpeed * cos(newAngle);
                    speedY = maxSpeed * sin(newAngle);
                }
                else
                {
                    isFound = true;
                }
        }
        else
            {
                dist = calcDistance(x, y, exitCoordX, exitCoordY);
                newAngle = calcAngle(exitCoordX - x, exitCoordY - y);

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

                speedX = maxSpeed * cos(newAngle);
                speedY = maxSpeed * sin(newAngle);
            }

        if (fabs(oldAngle - newAngle) < M_PI / 5)
        {
            x += speedX;
            y += speedY;
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

        pub_search(pub, x, y, oldAngle, srv.request.model_name);
        r.sleep();
        ros::spinOnce();
    }

    ROS_INFO_STREAM("search finished");
    return 0;
}


