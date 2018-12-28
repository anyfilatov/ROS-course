#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"

using namespace std;

bool obtainRobotFinderCoordinates(float &x, float &y) {
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        listener.lookupTransform("lost_robot", "robot_finder", ros::Time(0), transform);
        x = transform.getOrigin().x();
        y = transform.getOrigin().y();
        ROS_INFO("Robot finder coordinates: ( %.2f ; %.2f )", x, y);
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
    broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "lost_robot"));
}

float getNewCoordinate(float cur, float dest, float speed) {
    if (cur + speed < dest) {
        return cur + speed;
    } else if (cur - speed > dest) {
        return cur - speed;
    }
    return dest;
}

void calculateNewCoordinates(float &x, float &y, float &angle, float dest_x, float dest_y, float speed, float radius) {
    float new_angle = atan2(dest_y - y, dest_x - x);
    if (new_angle != angle) {
        angle = getNewCoordinate(angle, new_angle, 0.1);
    } else {
        if (sqrt(pow(dest_x - x, 2) + pow(dest_y - y, 2)) > radius) {
            x = getNewCoordinate(x, dest_x, speed);
            y = getNewCoordinate(y, dest_y, speed);
        }
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
    ros::init(argc, argv, "lost_robot_5");
    ros::NodeHandle n;
    ros::Rate r(25);

    float x = -5;
    float y = -5;
    float speed = 0.04;
    float angle = 0;

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
    srv.request.model_name = "lost_robot";
    geometry_msgs::Pose pose;
    updatePose(pose, x, y, angle);
    srv.request.initial_pose = pose;
    add_robot.call(srv);

    ros::Publisher pub = n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);

    gazebo_msgs::ModelState msg;
    msg.model_name = "lost_robot";
    updatePose(msg.pose, x, y, angle);
    pub.publish(msg);

    float old_x;
    float old_y;

    float finder_x;
    float finder_y;

    ROS_INFO("Waiting for the robot finder start");

    while (!obtainRobotFinderCoordinates(finder_x, finder_y)) {
        publishCoordinates(x, y);
        r.sleep();
    }

    ROS_INFO("I'm waiting for the robot finder");

    while (!isEqual(finder_x, finder_y, 0, 0, 1.5)) {
        publishCoordinates(x, y);
        obtainRobotFinderCoordinates(finder_x, finder_y);
        r.sleep();
    }

    ROS_INFO("I'm following robot finder");

    do {
        old_x = x;
        old_y = y;

        obtainRobotFinderCoordinates(finder_x, finder_y);
        calculateNewCoordinates(x, y, angle, x + finder_x, y + finder_y, speed, 1);
        publishCoordinates(x, y);

        ROS_INFO("My coordinates: ( %.2f ; %.2f )", x, y);

        updatePose(msg.pose, x, y, angle);
        pub.publish(msg);

        r.sleep();
    } while (!isFinished(x, y, old_x, old_y, 0.01));

    ROS_INFO("I have finished!");

    return 0;
}
