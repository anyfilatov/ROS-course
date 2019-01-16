#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>


void render(ros::Publisher& publisher, float x, float y);
void renderExit(ros::Publisher& publisher);
void sendTransform(float x, float y);
tf::StampedTransform listenTransform();


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rescuer");

    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<visualization_msgs::Marker>("coord_topic", 100);
    ros::Publisher publisherToFoundTopic = n.advertise<std_msgs::String>("found_topic", 10);
    ros::Rate rate(5);

    bool found = false;

    float x = 0.0;
    float y = 0.0;

    while(ros::ok()) {
        if (!found) {
            tf::StampedTransform transf = listenTransform();
            float lost_x = transf.getOrigin().x();
            float lost_y = transf.getOrigin().y();

            if (lost_x == x && lost_y == y) {
                found = true;
                std_msgs::String msg;
                publisherToFoundTopic.publish(msg);
            } else {
                if (fabsf(lost_x - x) >= fabsf(lost_y - y))		
                    x += (lost_x - x) / fabsf(lost_x - x);
                else
                    y += (lost_y - y) / fabsf(lost_y - y);
            }
        } else {
            if (0.0 == x && 0.0 == y) {
                return 0;
            } else {
                if (fabsf(0.0 - x) >= fabsf(0.0 - y))		
                    x += (0.0 - x) / fabsf(0.0 - x);
                else
                    y += (0.0 - y) / fabsf(0.0 - y);

                sendTransform(x, y);
            }
        }
        
        render(publisher, x, y);
        renderExit(publisher);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void render(ros::Publisher& publisher, float x, float y)
{
    visualization_msgs::Marker msg;
    msg.header.frame_id = "/world";
    msg.header.stamp = ros::Time::now();
    msg.ns = "";
    msg.action = visualization_msgs::Marker::ADD;
    msg.type = visualization_msgs::Marker::POINTS;
    msg.id = 1;
    
    msg.scale.x = 0.5;
    msg.scale.y = 0.5;
    msg.color.r = 0.0;
    msg.color.g = 1.0;
    msg.color.b = 0.0;
    msg.color.a = 1.0;

    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0;
    msg.points.push_back(p);
    publisher.publish(msg);
}

void renderExit(ros::Publisher& publisher)
{
    visualization_msgs::Marker msg;
    msg.header.frame_id = "/world";
    msg.header.stamp = ros::Time::now();
    msg.ns = "";
    msg.action = visualization_msgs::Marker::ADD;
    msg.type = visualization_msgs::Marker::POINTS;
    msg.id = 2;

    msg.scale.x = 0.0;
    msg.scale.y = 0.0;
    msg.color.r = 0.0;
    msg.color.g = 0.0;
    msg.color.b = 1.0;
    msg.color.a = 1.0;

    geometry_msgs::Point p;
    p.x = 1;
    p.y = 1;
    p.z = 0;
    msg.points.push_back(p);
    publisher.publish(msg);
}

void sendTransform(float x, float y)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    br.sendTransform(tf::StampedTransform(transform, ros::Time(4), "/world", "/rescue"));
}

tf::StampedTransform listenTransform()
{
    tf::TransformListener listener;
    tf::StampedTransform transform;

        ros::Time now = ros::Time::now();
        listener.waitForTransform("/world", "/lost", ros::Time(4), ros::Duration(1.0));
        listener.lookupTransform("/world", "/lost", ros::Time(4), transform);

    return transform;
}
