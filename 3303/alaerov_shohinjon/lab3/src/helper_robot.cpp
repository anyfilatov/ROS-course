#include <math.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
using namespace ros;
double StartX = -4.5;
double StartY = -4.5;
double Step = 0.01;
double Distance(double x1, double y1, double x2, double y2)
{
	return sqrt(pow(x1-x2, 2)+pow(y1-y2, 2));
}
int main(int argc, char** argv)
{
	init(argc, argv, "helper_robot");
    	NodeHandle n;
    	Publisher publisher = n.advertise<visualization_msgs::Marker>("helper_robot_topic", 10);
      	visualization_msgs::Marker marker;
      	marker.header.frame_id = "frame";
      	marker.header.stamp = Time::now();
      	marker.ns = "helper_robot";
      	marker.id = 0;
      	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
      	marker.pose.position.x = StartX;
      	marker.pose.position.y = StartY;
      	marker.scale.x = 0.1;
      	marker.scale.y = 0.1;
      	marker.scale.z = 0.1;
      	marker.color.r = 1.0;
      	marker.color.g = 0.0;
      	marker.color.b = 0.0;
      	marker.color.a = 1.0;
 	tf::TransformListener listener;
   	Rate rate(100);
    	while(true)
    	{
      		while (publisher.getNumSubscribers() < 2)
      		{
        		ROS_WARN_ONCE("Please create a subscriber to the marker");
        		sleep(1);
      		}
    		tf::StampedTransform transform;
    		try
		{
      			listener.lookupTransform("/helper_robot", "/lost_robot", Time(0), transform);
    		}
    		catch (tf::TransformException &e) 
		{
      			ROS_ERROR("%s", e.what());
			publisher.publish(marker);
      			sleep(1);
      			continue;
    		}
		double angle = atan2(transform.getOrigin().y(), transform.getOrigin().x());
    		marker.pose.position.x+=Step*cos(angle);
		marker.pose.position.y+=Step*sin(angle);
    		publisher.publish(marker);
		if(Distance(transform.getOrigin().x(), transform.getOrigin().y(), 
				        marker.pose.position.x, marker.pose.position.y)<1)
			break;
 		rate.sleep();
    	}
    	while(true)
    	{
    		tf::StampedTransform transform;
    		try
		{
      			listener.lookupTransform("/helper_robot", "/exit", Time(0), transform);
    		}
    		catch (tf::TransformException &e) 
		{
      			ROS_ERROR("%s", e.what());
			publisher.publish(marker);
      			sleep(1);
      			continue;
    		}
		double angle = atan2(transform.getOrigin().y(), transform.getOrigin().x());
    		marker.pose.position.x+=Step*cos(angle);
		marker.pose.position.y+=Step*sin(angle);
    		publisher.publish(marker);
 		rate.sleep();
    	}
}
