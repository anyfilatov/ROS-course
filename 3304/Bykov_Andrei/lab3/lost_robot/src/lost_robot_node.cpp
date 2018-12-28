#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

visualization_msgs::Marker createMarker(float x, float y, float z) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/point_on_map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "lost_robot";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    marker.points.push_back(p);
    return marker;
}

bool obtainRobotFinderCoordinates(float &x, float &y, float &z) {
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        listener.lookupTransform("world", "robot_finder", ros::Time(0), transform);
        x = transform.getOrigin().x();
        y = transform.getOrigin().y();
        z = transform.getOrigin().z();
        ROS_INFO("Robot finder coordinates: ( %.2f ; %.2f ; %.2f )", x, y, z);
        return true;
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("I am not seeing robot finder");
        return false;
    }
}

void publishCoordinates(float x, float y, float z) {
    static tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
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

void calculateNewCoordinates(float &x, float &y, float &z, float dest_x, float dest_y, float dest_z, float speed) {
    x = getNewCoordinate(x, dest_x, speed);
    y = getNewCoordinate(y, dest_y, speed);
    z = getNewCoordinate(z, dest_z, speed);
}

bool isFinished(float x, float y, float z, float last_x, float last_y, float last_z) {
    static int counter = 0;

    if (x == last_x && y == last_y && z == last_z) {
        counter++;
    } else {
        counter = 0;
    }

    return counter >= 5;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lost_robot");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("lost_robot_topic", 10, true);

    float speed = 1;
    float x = 10;
    float y = 10;
    float z = 0;

    float old_x;
    float old_y;
    float old_z;

    float finder_x;
    float finder_y;
    float finder_z;

    marker_pub.publish(createMarker(x, y, z));

    while (!obtainRobotFinderCoordinates(finder_x, finder_y, finder_z)) {
        publishCoordinates(x, y, z);

        r.sleep();
    }

    r.sleep();

    ROS_INFO("I'm following robot finder");

    do {
        old_x = x;
        old_y = y;
        old_z = z;

        obtainRobotFinderCoordinates(finder_x, finder_y, finder_z);
        calculateNewCoordinates(x, y, z, finder_x, finder_y, finder_z, speed);

        ROS_INFO("My coordinates: ( %.2f ; %.2f ; %.2f )", x, y, z);

        marker_pub.publish(createMarker(x, y, z));

        r.sleep();
    } while (!isFinished(x, y, z, old_x, old_y, old_z));

    ROS_INFO("I have finished!");

    return 0;
}