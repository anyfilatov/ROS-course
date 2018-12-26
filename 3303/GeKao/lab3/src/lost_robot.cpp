#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <cstdlib>
#include <ctime>


visualization_msgs::Marker setMarker(float x,float y)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id="/world";
    marker.header.stamp=ros::Time::now();
    marker.ns="lost_rbot";
    marker.id=0;
    marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = x;
 	marker.pose.position.y = y;
	marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
	marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    return marker;

}

//send the TF
void sendTransform(float x,float y)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x,y,0.0));
    transform.setRotation(tf::Quaternion(0,0,0,1));
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"/world","/lost"));

}
//listen to the rescure's TF
tf::StampedTransform transformPoint(const tf::TransformListener& listener)
{
    tf::StampedTransform transform;
    try
    {
        ros::Time now= ros::Time::now();
        // listener.waitForTransform("/world","/rescue",ros::Time(0),ros::Duration(1.0));
        listener.lookupTransform("/world","/rescue",ros::Time(0),transform);
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

void foundCallback(const std_msgs::String& msg)
{
    ROS_INFO_STREAM("Rescuer says: " << msg.data);
}


float  GenerateStep(float  minStep,float  maxStep)
{
    return (maxStep-minStep)*((float)rand()/RAND_MAX)+minStep;
}

bool closed(float x1,float y1,float x2,float y2)
{
    float distance = sqrt(pow(x1-x2,2)+pow(y1-y2,2));
    if (distance<5)
    {
        return true;
    }
    return false;
    
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lost_robot");
    ros::NodeHandle node;
    ros::Rate rate(10);
    ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("lost_robot_topic",10,true);
    ros::Subscriber subscriber = node.subscribe("found_topic", 100, foundCallback);
    tf::TransformListener listener;
    tf::StampedTransform transform;

    srand(time(NULL));
    float minStep = -0.1;
    float maxStep = 0.1;
    float oneDirectionStepsCount=50;
    float StepForGoToExit = 0.1;
    float x =3.0;
    float y =3.0;
    float x_secure=0.0;
    float y_secure=0.0;

    int stepNumber = 0;
    float stepX = GenerateStep(minStep,maxStep);
    float stepY = GenerateStep(minStep,maxStep);
    sendTransform(x,y);
    while (true)
    {   
        
        try
        {
            transform = transformPoint(listener);
        }
        catch (tf::TransformException &e)
        {
            ROS_ERROR("%s", e.what());
            sendTransform(x,y);
            marker_pub.publish(setMarker(x,y));
            sleep(1);
      		continue;
        }
        
        if (stepNumber==oneDirectionStepsCount)
        {
            stepNumber =0;
            stepX = GenerateStep(minStep,maxStep);
            stepY = GenerateStep(minStep,maxStep);
        }
   
        x+=stepX;
        y+=stepY;
        
        sendTransform(x,y);
        marker_pub.publish(setMarker(x,y));
    
        x_secure = transform.getOrigin().x();
        y_secure = transform.getOrigin().y();
        if (closed(x_secure,y_secure,x,y))
        {
            ROS_INFO("Rescured!");
            break;
        }
        rate.sleep();
        stepNumber++; 
    }
    
    tf::TransformListener listener1;
    tf::StampedTransform transform1;
    while (true)
    {   
        
        try
        {
            transform1 = transformPoint(listener1);
        }
        catch (tf::TransformException &e)
        {
            ROS_ERROR("%s", e.what());
            sendTransform(x,y);
            marker_pub.publish(setMarker(x,y));
            sleep(1);
      		continue;
        }

        x_secure = transform1.getOrigin().x();
        y_secure = transform1.getOrigin().y();

        float angle = atan2(y_secure,x_secure);
        x+=StepForGoToExit*cos(angle);
        y+=StepForGoToExit*sin(angle);
        sendTransform(x,y);
        marker_pub.publish(setMarker(x,y));
        rate.sleep();
        
    }
    
    
    return 0;
}