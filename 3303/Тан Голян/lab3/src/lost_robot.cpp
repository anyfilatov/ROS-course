#include <cstdlib>
#include <ctime>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
using namespace ros;
double MinStep = -0.01;
double MaxStep = 0.01;
double OneDirectionStepsCount = 50;
double StepForGoToExit = 0.01;
double GenerateStep()
{
	return (MaxStep-MinStep) * ((double)rand()/RAND_MAX) + MinStep;
}
bool AreObjectsCloseEnough(double x1, double y1, double x2, double y2)
{
	double distance = sqrt(pow(x1-x2, 2)+pow(y1-y2, 2));
	if(distance < 10)
		return true;
	return false;
}
int main(int argc, char** argv)
{
	init(argc, argv, "lost_robot");
    	NodeHandle n;
    	Publisher publisher = n.advertise<visualization_msgs::Marker>("lost_robot_topic", 10);
      	visualization_msgs::Marker marker;
      	marker.header.frame_id = "frame";
      	marker.header.stamp = Time::now();
      	marker.ns = "lost_robot";
      	marker.id = 0;
      	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
      	marker.pose.position.x = 3;
      	marker.pose.position.y = 3;
      	marker.scale.x = 0.1;
      	marker.scale.y = 0.1;
      	marker.scale.z = 0.1;
      	marker.color.r = 0.0;
      	marker.color.g = 1.0;
      	marker.color.b = 0.0;
      	marker.color.a = 1.0;
	srand(time(NULL));
	tf::TransformListener listener;
    	Rate rate(50);
	int stepNumber = 0;
	double stepX = GenerateStep();
	double stepY = GenerateStep();
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
      			listener.lookupTransform("/lost_robot", "/helper_robot", Time(0), transform);
    		}
    		catch (tf::TransformException &e) 
		{
      			ROS_ERROR("%s", e.what());
			publisher.publish(marker);
      			sleep(1);
      			continue;
    		}
		if(stepNumber == OneDirectionStepsCount)
		{
			stepNumber = 0;
			stepX = GenerateStep();
			stepY = GenerateStep();
		}
		marker.pose.position.x+=stepX;
		marker.pose.position.y+=stepY;
      		publisher.publish(marker);
		if(AreObjectsCloseEnough(transform.getOrigin().x(), transform.getOrigin().y(), 
				        marker.pose.position.x, marker.pose.position.y))
			break;
      		rate.sleep();
		stepNumber++;
    	}
    	while(true)
    	{
    		tf::StampedTransform transform;
    		try
		{
      			listener.lookupTransform("/lost_robot", "/helper_robot", Time(0), transform);
    		}
    		catch (tf::TransformException &e) 
		{
      			ROS_ERROR("%s", e.what());
			publisher.publish(marker);
      			sleep(1);
      			continue;
    		}
		double angle = atan2(transform.getOrigin().y(), transform.getOrigin().x());
    		marker.pose.position.x+=StepForGoToExit*cos(angle);
		marker.pose.position.y+=StepForGoToExit*sin(angle);
    		publisher.publish(marker);
 		rate.sleep();
    	}
}
