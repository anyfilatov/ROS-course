#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <random>
#include <string>
#include <time.h>

int main( int argc, char** argv )
{
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle n;
    ros::NodeHandle pnode("~"); // to get params

    tf::TransformBroadcaster br;
    tf::Transform transform;

    std::default_random_engine generator;
    generator.seed(time(NULL));
    std::uniform_real_distribution<double> angle_distribution(-1.0, 1.0); // +-60 deg max.
    double angle = angle_distribution(generator);
    std::uniform_real_distribution<double> speed_distribution(0.0, 0.2);
    double speed = speed_distribution(generator);

    tf::Vector3 offset;
    if (ros::this_node::getName() != "/map")
        offset = tf::Vector3(0, 0, 1); // stick is 1 unit long

    tf::Quaternion rt(tf::Vector3(1, 0, 0), angle); // from tf wiki

    ros::Rate r(25);
    while (n.ok()) {
        transform.setRotation(rt); // rotate
        transform.setOrigin(offset); // offset from prev frame

        rt = tf::Quaternion(tf::Vector3(0, 0, 1), speed) * rt; // from tf wiki

        tf::StampedTransform sTransform(
            transform,
            ros::Time::now(),
            ros::this_node::getName(),
            pnode.param("prev_frame_id", std::string("default"))
        );

        br.sendTransform(sTransform);

        r.sleep();
    }
}
