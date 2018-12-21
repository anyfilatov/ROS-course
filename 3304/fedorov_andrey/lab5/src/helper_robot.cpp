#include <math.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <fstream>
#include <gazebo_msgs/ModelState.h>
using namespace ros;
double Step = 0.1;
double ExitX = -10.0;
double ExitY = 10.0;
double angleStep = 0.5;
bool AreObjectsCloseEnough(double x1, double y1, double x2, double y2)
{
	double distance = sqrt(pow(x1-x2, 2)+pow(y1-y2, 2));
	if(distance < 2)
		return true;
	return false;
}
int main(int argc, char *argv[])
{
	init(argc, argv, "helper_robot");
	NodeHandle n;
	sleep(19);
	Publisher publisher = n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    	gazebo_msgs::ModelState msg;
    	msg.model_name = "helper_robot";
    	msg.pose.position.x = ExitX;
    	msg.pose.position.y = ExitY;
    	msg.pose.orientation.z = sin(0);
    	msg.pose.orientation.w = cos(0);
	tf::TransformListener listener;
	Rate rate(100);
	double oldAngle, angle;
	while(true)
    	{
		tf::StampedTransform transform;
                try
                {
			listener.lookupTransform("/helper_robot", "/lost_robot", Time(0), transform);
                }
                catch (tf::TransformException &e)
                {
                        ROS_ERROR("%s", e.what());
			publisher.publish(msg);
      			sleep(1);
      			continue;
                }
		oldAngle=angle;
                angle = atan2(transform.getOrigin().y(), transform.getOrigin().x());
        	msg.pose.position.x+=Step*cos(angle);
        	msg.pose.position.y+=Step*sin(angle);
    		msg.pose.orientation.z=sin(angle/2);
    		msg.pose.orientation.w=cos(angle/2);
        	publisher.publish(msg);
                if(AreObjectsCloseEnough(transform.getOrigin().getX(), transform.getOrigin().getY(), msg.pose.position.x, 				msg.pose.position.y))
                {
			break;
                }
        	rate.sleep();
	}
	while(true)
	{	
		oldAngle=angle;
                angle = atan2(ExitY-msg.pose.position.y, ExitX-msg.pose.position.x);
        	msg.pose.position.x+=Step*cos(angle);
        	msg.pose.position.y+=Step*sin(angle);
		if(oldAngle<angle)
		{
			for(double middleAngle=oldAngle; middleAngle<angle; middleAngle+=angleStep)
			{
				msg.pose.orientation.z=sin(middleAngle/2);
    				msg.pose.orientation.w=cos(middleAngle/2);
        			publisher.publish(msg);
				rate.sleep();
			}
		}
		else
		{
			for(double middleAngle=oldAngle; middleAngle>angle; middleAngle-=angleStep)
			{
				msg.pose.orientation.z=sin(middleAngle/2);
    				msg.pose.orientation.w=cos(middleAngle/2);
        			publisher.publish(msg);
				rate.sleep();
			}
		}
    		msg.pose.orientation.z=sin(angle/2);
    		msg.pose.orientation.w=cos(angle/2);
        	publisher.publish(msg);
		if(AreObjectsCloseEnough(ExitX, ExitY, msg.pose.position.x, msg.pose.position.y))
                {
			break;
                }
        	rate.sleep();
    	}
	while(true)
		publisher.publish(msg);
}
