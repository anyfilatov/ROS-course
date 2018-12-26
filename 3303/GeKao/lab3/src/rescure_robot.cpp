#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "geometry_msgs/Twist.h"


visualization_msgs::Marker setMarker(float x,float y)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id="/world";
    marker.header.stamp=ros::Time::now();
    marker.ns="robots";
    marker.id=1;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    return marker;

}

visualization_msgs::Marker exitMarker(float exitStartX,float exitStartY,float exitSize)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id="/world";
    marker.header.stamp = ros::Time::now();
    marker.ns="exit";
    marker.id=2;
    marker.scale.x = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = exitStartX+exitSize/2;
    marker.pose.position.y = exitStartY+exitSize/2;
    geometry_msgs::Point p;
    p.x=-exitSize/2;
    p.y=-exitSize/2;
    marker.points.push_back(p);
    p.x = exitSize/2;
    marker.points.push_back(p);
    p.y = exitSize/2;
    marker.points.push_back(p);
    p.x = -exitSize/2;
    marker.points.push_back(p);
    p.y = -exitSize/2;
    marker.points.push_back(p); 
    return marker;
}

//send the TF
void sendTransform(float x,float y)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x,y,0.0));
    transform.setRotation(tf::Quaternion(0,0,0,1));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/rescue"));

}
//listen to the lost's TF
tf::StampedTransform transformPoint(const tf::TransformListener& listener)
{
    tf::StampedTransform transform;
    try
    {
        ros::Time now= ros::Time::now();
        // listener.waitForTransform("/rescue","/lost",ros::Time(0),ros::Duration(1.0));
        listener.lookupTransform("/world","/lost",ros::Time(0),transform);
        ROS_INFO("origin: %.2f, %.2f; rotation: %.2f, %.2f, %.2f", transform.getOrigin().x(), transform.getOrigin().y(),
        transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z());
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    return transform;
 
}


bool closed(float x1,float y1,float x2,float y2,float len)
{
    float distance = sqrt(pow(x1-x2,2)+pow(y1-y2,2));
    if (distance<len)
    {
        return true;
    }
    return false;
    
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rescure_robot");
    ros::NodeHandle node;
    ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("rescure_robot_topic",100,true);
    ros::Publisher publisherToFoundTopic = node.advertise<std_msgs::String>("found_topic",10);
    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::Rate rate(5);
 
    float exitStartX=-5;
    float exitStartY=-5;
    float exitSize=1;
    float startX = -4.5;
    float startY = -4.5;
    float step=0.1;
    float x_lost=0.0;
    float y_lost=0.0;

    sendTransform(startX,startY);
    while (true)
    {
        try
        {
            transform = transformPoint(listener);
        }
        catch (tf::TransformException &e)
        {
            ROS_ERROR("%s", e.what());
            sendTransform(startX,startY);
			marker_pub.publish(setMarker(startX,startY));
            marker_pub.publish(exitMarker(exitStartX,exitStartY,exitSize));
      		sleep(1);
      		continue;
        }
        x_lost = transform.getOrigin().x();
        y_lost = transform.getOrigin().y();

        float angle = atan2(y_lost,x_lost);
        startX+=step*cos(angle);
        startY+=step*sin(angle);
        sendTransform(startX,startY);
        marker_pub.publish(setMarker(startX,startY));
        marker_pub.publish(exitMarker(exitStartX,exitStartY,exitSize));
    
         
        if (closed(x_lost,y_lost,startX,startY,5))
        {
            ROS_INFO("Found!");
            std_msgs::String msg;
            msg.data = std::string("Follow me!");
            publisherToFoundTopic.publish(msg);
            break;
        }
        rate.sleep();
    
    }
    while (true)
    {
        float angle = atan2(exitStartY,exitStartY);
        startX+=step*cos(angle);
        startY+=step*sin(angle);
        if (closed(exitStartX,exitStartY,startX,startY,0.4))
        {
            ROS_INFO("Reacured!");
            std_msgs::String msg;
            msg.data = std::string("Reacured!");
            publisherToFoundTopic.publish(msg);                     
            break;
        }
        marker_pub.publish(setMarker(startX,startY));
        marker_pub.publish(exitMarker(exitStartX,exitStartY,exitSize));  
        sendTransform(startX,startY);
        

    }
    
    

    return 0;
}