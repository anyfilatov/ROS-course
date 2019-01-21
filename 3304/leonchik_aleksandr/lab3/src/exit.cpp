#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace ros;

double ExitStartX = -5;
double ExitStartY = -5;
double ExitSize = 1;

int main(int argc, char** argv)
{
	init(argc, argv, "exit");
    	NodeHandle n;
    	Publisher publisher = n.advertise<visualization_msgs::Marker>("exit_topic", 10);
      	visualization_msgs::Marker marker;
      	marker.header.frame_id = "frame";
      	marker.header.stamp = Time::now();
      	marker.ns = "exit";
      	marker.id = 0;
      	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
      	marker.scale.x = 0.1;
      	marker.color.r = 1.0;
      	marker.color.g = 0.0;
      	marker.color.b = 0.0;
      	marker.color.a = 1.0;
	marker.pose.position.x=ExitStartX+ExitSize/2;
	marker.pose.position.y=ExitStartY+ExitSize/2;
	geometry_msgs::Point point;
	point.x = -ExitSize/2;
	point.y = -ExitSize/2;
	marker.points.push_back(point);
	point.x = ExitSize/2;
	marker.points.push_back(point);
	point.y = ExitSize/2;
	marker.points.push_back(point);
	point.x = -ExitSize/2;
	marker.points.push_back(point);
	point.y = -ExitSize/2;
	marker.points.push_back(point);
	Rate rate(1);

	while(true)
	{
		publisher.publish(marker);
		rate.sleep();
	}
}
