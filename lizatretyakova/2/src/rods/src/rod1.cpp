#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <string>


const std::string INIT = "frame0";

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_tf_broadcaster");
    
    if(argc != 2) {
        ROS_ERROR("need turtle name as argument"); 
        return -1;
    }
    std::string next_name = argv[1];
    std::string this_name = ros::this_node::getName();

    ROS_INFO("[ROD1.CPP] %d %s\n", argc, argv[1]);

    ros::NodeHandle node;

    int len = this_name == INIT ? 0 : 1;
    double angle = 0.5;
    double v = 0.1;
    tf::Vector3 axis_main(0, 0, len);
    tf::Vector3 axis_init(1, 0, 0);
    tf::Quaternion q_rotate(axis_init, angle);
    tf::Quaternion q_delta(axis_main, v);

    tf::TransformBroadcaster br;
    tf::Transform transform;
    ros::Rate rate(30);

    while (node.ok()){
        transform.setOrigin(axis_main);
        transform.setRotation(q_rotate);
        br.sendTransform(
                tf::StampedTransform(
                    transform, ros::Time::now(), this_name, next_name));
        q_rotate = q_delta * q_rotate;
        rate.sleep();
    }
    return 0;
}
