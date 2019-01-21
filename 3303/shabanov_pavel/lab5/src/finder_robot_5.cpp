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
    ros::init(argc, argv, "finder_robot_5");

    ros::NodeHandle n;
    ros::Rate r(25);

	finder_robot.name = "finder_robot";
    finder_robot.x = 5;
    finder_robot.y = 5;
    finder_robot.speed = 0.04;
    finder_robot.angle = 0;
	
    lost_robot.name = "lost_robot";

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
    srv.request.model_name = "finder_robot";
    geometry_msgs::Pose pose;
    update_pose(pose, finder_robot);
    srv.request.initial_pose = pose;
    add_robot.call(srv);

    ros::Publisher pub = n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);

    gazebo_msgs::ModelState msg;
    msg.model_name = "finder_robot";
    update_pose(pose, finder_robot);
    pub.publish(msg);

    float start_x = finder_robot.x;
    float start_y = finder_robot.y;

    float old_x;
    float old_y;

    float lost_x;
    float lost_y;

    bool flag = false;
    while (!flag) {
        flag = get_robot_coordinates(lost_robot, finder_robot);
        publish_coordinates(finder_robot);
        r.sleep();
    }

    ROS_INFO("Moving towards the lost robot..");

    while (!is_equal(0, 0, lost_robot.x, lost_robot.y, 1)) {
        old_x = finder_robot.x;
        old_y = finder_robot.y;
        calculate_new_coordinates(finder_robot, finder_robot.x + lost_robot.x, finder_robot.y + lost_robot.y, 0);

        ROS_INFO("My coordinates: ( %.2f ; %.2f )", x, y);

        update_pose(pose, finder_robot);
        pub.publish(msg);

        publish_coordinates(finder_robot);

        r.sleep();
        get_robot_coordinates(lost_robot, finder_robot);
    }

    ROS_INFO("Moving towards the exit..");

    while (!is_equal(finder_robot.x, finder_robot.y, start_x, start_y, 0)) {
        old_x = finder_robot.x;
        old_y = finder_robot.y;
        calculate_new_coordinates(finder_robot, start_x, start_y, 0);

        ROS_INFO("My coordinates: %.2f, %.2f", x, y);

        update_pose(pose, finder_robot);
        pub.publish(msg);

        get_robot_coordinates(lost_robot, finder_robot);
        publish_coordinates(finder_robot);

        r.sleep();
    }

    ROS_INFO("Waiting for the lost robot");

    do {
        old_x = lost_robot.x;
        old_y = lost_robot.y;
        get_robot_coordinates(lost_robot, finder_robot);
        publish_coordinates(finder_robot);
        r.sleep();
    } while (is_finished(lost_robot.x, lost_robot.y, old_x, old_y, 0.01));

    ROS_INFO("I have finished");

    return 0;
}

