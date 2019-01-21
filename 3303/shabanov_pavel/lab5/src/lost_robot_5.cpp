#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"
#include "utilities.h"

using namespace std;


int main(int argc, char **argv) {
    ros::init(argc, argv, "lost_robot_5");
    ros::NodeHandle n;
    ros::Rate r(25);

	lost_robot.name = "lost_robot";
    lost_robot.x = -5;
    lost_robot.y = -5;
    lost_robot.speed = 0.04;
    lost_robot.angle = 0;
	
    finder_robot.name = "finder_robot";

    ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::ServiceClient add_robot =
            n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;

    ifstream fin("/home/vadim/.gazebo/models/pioneer2dx/model.sdf");

    string model;
    string buf;
    while (!fin.eof()) {
        getline(fin, buf);
        model += buf + "\n";
    }
    srv.request.model_xml = model;
    srv.request.model_name = "lost_robot";
    geometry_msgs::Pose pose;
    update_pose(pose, lost_robot);
    srv.request.initial_pose = pose;
    add_robot.call(srv);

    ros::Publisher pub = n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);

    gazebo_msgs::ModelState msg;
    msg.model_name = "lost_robot";
    update_pose(msg.pose, lost_robot);
    pub.publish(msg);

    bool flag = false;
    while (!flag) {
        flag = get_robot_coordinates(finder_robot, lost_robot);
        publish_coordinates(lost_robot);
        r.sleep();
    }

    ROS_INFO("I'm waiting..");

    while (!isEqual(finder_robot.x, finder_robot.y, 0, 0, 1.5)) {
        publish_coordinates(lost_robot);
        get_robot_coordinates(finder_robot, lost_robot);
        r.sleep();
    }

    ROS_INFO("He found me! I am following him now..");
    
    float old_x;
    float old_y;
    do {
        old_x = lost_robot.x;
        old_y = lost_robot.y;

        get_robot_coordinates(finder_robot, lost_robot);
        calculate_new_coordinates(lost_robot, lost_robot.x + finder_robot.x, lost_robot.y + finder_robot.y, 1);
        publish_coordinates(lost_robot);

        ROS_INFO("My coordinates: ( %.2f ; %.2f )", lost_robot.x, lost_robot.y);

        update_pose(msg.pose, lost_robot);
        pub.publish(msg);

        r.sleep();
    } while (!isFinished(lost_robot.x, lost_robot.y, old_x, old_y, 0.01));

    ROS_INFO("I have finished!");

    return 0;
}

