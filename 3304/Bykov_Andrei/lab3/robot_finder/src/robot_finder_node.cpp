#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

visualization_msgs::Marker createMarker(float x, float y, float z) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/point_on_map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "robot_finder";
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
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    marker.points.push_back(p);
    return marker;
}

bool obtainLostRobotCoordinates(float &x, float &y, float &z) {
    static tf::TransformListener listener;
    tf::StampedTransform transform;
    try {
        listener.lookupTransform("world", "lost_robot", ros::Time(0), transform);
        x = transform.getOrigin().x();
        y = transform.getOrigin().y();
        z = transform.getOrigin().z();
        ROS_INFO("Lost robot coordinates: ( %.2f ; %.2f ; %.2f )", x, y, z);
        return true;
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("Lost robot is not found");
        return false;
    }
}

void publishCoordinates(float x, float y, float z) {
    static tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
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

void calculateNewCoordinates(float &x, float &y, float &z, float dest_x, float dest_y, float dest_z, float speed) {
    x = getNewCoordinate(x, dest_x, speed);
    y = getNewCoordinate(y, dest_y, speed);
    z = getNewCoordinate(z, dest_z, speed);
}

bool isEqual(float x, float y, float z, float dest_x, float dest_y, float dest_z) {
    return x == dest_x && y == dest_y && z == dest_z;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_finder");

    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("robot_finder_topic", 10, true);

    ros::Rate r(1);

    float start_x = 0;
    float start_y = 0;
    float start_z = 0;
    float speed = 1;

    float x = start_x;
    float y = start_y;
    float z = start_z;

    marker_pub.publish(createMarker(x, y, z));

    float lost_x;
    float lost_y;
    float lost_z;

    while (!obtainLostRobotCoordinates(lost_x, lost_y, lost_z)) {
        r.sleep();
    }

    while (!isEqual(x, y, z, lost_x, lost_y, lost_z)) {
        calculateNewCoordinates(x, y, z, lost_x, lost_y, lost_z, speed);

        ROS_INFO("My coordinates: ( %.2f ; %.2f ; %.2f )", x, y, z);

        marker_pub.publish(createMarker(x, y, z));

        r.sleep();
    }

    ROS_INFO("I'm going to start");

    while (!isEqual(x, y, z, start_x, start_y, start_z)) {
        calculateNewCoordinates(x, y, z, start_x, start_y, start_z, speed);

        ROS_INFO("My coordinates: %.2f, %.2f, %.2f;", x, y, z);

        marker_pub.publish(createMarker(x, y, z));

        publishCoordinates(x, y, z);

        r.sleep();
    }

    ROS_INFO("I have finished");

    r.sleep();

    return 0;
}
