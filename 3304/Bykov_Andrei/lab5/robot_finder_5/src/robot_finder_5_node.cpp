#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"

using namespace std;

bool obtainLostRobotCoordinates(float &x, float &y) {
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        listener.lookupTransform("robot_finder", "lost_robot", ros::Time(0), transform);
        x = transform.getOrigin().x();
        y = transform.getOrigin().y();
        ROS_INFO("Lost robot coordinates: ( %.2f ; %.2f )", x, y);
        return true;
    }
    catch (tf::TransformException &ex) {
        return false;
    }
}

void publishCoordinates(float x, float y) {
    static tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot_finder"));
}

float getNewCoordinate(float cur, float dest, float speed) {
    if (cur + speed < dest) {
        return cur + speed;
    } else if (cur - speed > dest) {
        return cur - speed;
    }
    return dest;
}

void calculateNewCoordinates(float &x, float &y, float &angle, float dest_x, float dest_y, float speed) {
    float new_angle = atan2(dest_y - y, dest_x - x);
    if (new_angle != angle) {
        angle = getNewCoordinate(angle, new_angle, 0.1);
    } else {
        x = getNewCoordinate(x, dest_x, speed);
        y = getNewCoordinate(y, dest_y, speed);
    }
}

bool isEqual(float x, float y, float dest_x, float dest_y, float radius) {
    return sqrt(pow(dest_x - x, 2) + pow(dest_y - y, 2)) <= radius;
}

bool isFinished(float x, float y, float last_x, float last_y, float radius) {
    static int counter = 0;

    if (sqrt(pow(last_x - x, 2) + pow(last_y - y, 2)) <= radius) {
        counter++;
    } else {
        counter = 0;
    }

    return counter >= 100;
}

void updatePose(geometry_msgs::Pose &pose, float x, float y, float angle) {
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation.z = sin(angle / 2);
    pose.orientation.w = cos(angle / 2);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_finder_5");

    ros::NodeHandle n;
    ros::Rate r(25);

    float start_x = 5;
    float start_y = 5;
    float angle = 0;
    float speed = 0.04;

    ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::ServiceClient add_robot =
            n.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    gazebo_msgs::SpawnModel srv;

    ifstream fin("/home/andrey/.gazebo/models/pioneer2dx/model.sdf");

    string model;
    string buf;
    while (!fin.eof()) {
        getline(fin, buf);
        model += buf + "\n";
    }
    srv.request.model_xml = model;
    srv.request.model_name = "robot_finder";
    geometry_msgs::Pose pose;
    updatePose(pose, start_x, start_y, angle);
    srv.request.initial_pose = pose;
    add_robot.call(srv);

    ros::Publisher pub = n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);

    gazebo_msgs::ModelState msg;
    msg.model_name = "robot_finder";
    updatePose(msg.pose, start_x, start_y, angle);
    pub.publish(msg);

    float x = start_x;
    float y = start_y;

    float old_x;
    float old_y;

    float lost_x;
    float lost_y;

    ROS_INFO("Waiting for the lost robot start");

    while (!obtainLostRobotCoordinates(lost_x, lost_y)) {
        publishCoordinates(x, y);
        r.sleep();
    }

    ROS_INFO("I'm going for the lost robot");

    while (!isEqual(0, 0, lost_x, lost_y, 1)) {
        old_x = x;
        old_y = y;
        calculateNewCoordinates(x, y, angle, x + lost_x, x + lost_y, speed);

        ROS_INFO("My coordinates: ( %.2f ; %.2f )", x, y);

        updatePose(msg.pose, x, y, angle);

        pub.publish(msg);

        publishCoordinates(x, y);

        r.sleep();
        obtainLostRobotCoordinates(lost_x, lost_y);
    }

    ROS_INFO("I'm going to start");

    while (!isEqual(x, y, start_x, start_y, 0)) {
        old_x = x;
        old_y = y;
        calculateNewCoordinates(x, y, angle, start_x, start_y, speed);

        ROS_INFO("My coordinates: %.2f, %.2f", x, y);

        updatePose(msg.pose, x, y, angle);

        pub.publish(msg);

        obtainLostRobotCoordinates(lost_x, lost_y);
        publishCoordinates(x, y);

        r.sleep();
    }

    ROS_INFO("I'm waitnig til the lost robot finish");

    do {
        old_x = lost_x;
        old_y = lost_y;
        obtainLostRobotCoordinates(lost_x, lost_y);
        publishCoordinates(x, y);
        r.sleep();
    } while (isFinished(lost_x, lost_y, old_x, old_y, 0.01));

    ROS_INFO("I have finished");

    return 0;
}
