#include <csignal>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "robot.h"

Robot* rescuer_robot;

void signalHandler(int signum) {

    if (rescuer_robot != NULL) {
        rescuer_robot->deleteModel();
        delete rescuer_robot;
    }

    exit(signum);
}

int main(int argc, char ** argv) {
    signal(SIGINT, signalHandler);

    ros::init(argc, argv, "rescuer_robot", ros::init_options::NoSigintHandler);

    ros::NodeHandle n;
    ros::Publisher publisher_found_topic = n.advertise<std_msgs::String>("found_topic", 10);
    ros::Rate rate(40);

    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;
    tf::StampedTransform lost_transform;
    tf::Transform transform;

    bool found = false;

    float x = 0.0;
    float y = 0.0;

    rescuer_robot = new Robot(n, 40, "rescuer_robot", "/home/ros/.gazebo/models/quadrotor/model-1_4.sdf", x, y);

    while(ros::ok()) {
        transform.setOrigin(tf::Vector3(x, y, 0.0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "rescuer_robot"));

        if (!found) {

		try{
            tf_listener.waitForTransform("rescuer_robot", "lost_robot", ros::Time(0), ros::Duration(0.5));
            tf_listener.lookupTransform("rescuer_robot", "lost_robot", ros::Time(0), lost_transform);
} catch (tf::TransformException &ex){
		rate.sleep();
		continue;
}

            float lost_x = lost_transform.getOrigin().x();
            float lost_y = lost_transform.getOrigin().y();

            if (fabs(lost_x) < 3 && fabs(lost_y) < 3) { //find
                found = true;
                std_msgs::String msg;
                publisher_found_topic.publish(msg);
            } else { // lost
                if (fabs(lost_x) >= fabs(lost_y)) {
                    if (lost_x > 0)
                        x += 1;
                    else
                        x -= 1;
                } else {
                    if (lost_y > 0)
                        y += 1;
                    else
                        y -= 1;
                }
            }
        } else {
            if (x != 0 || y != 0) { //finish
                if (fabsf(0.0 - x) >= fabsf(0.0 - y))
                    x += ((0.0 - x) / fabsf(0.0 - x));
                else
                    y += ((0.0 - y) / fabsf(0.0 - y));
            }
        }

        rescuer_robot->move(x, y);

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
