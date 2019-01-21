#include <cstdlib>
#include <ctime>
#include <ros/ros.h>
#include <gazebo_msgs/ModelState.h>
#include <tf/transform_listener.h>

using namespace ros;

double MinStep = -0.1;
double MaxStep = 0.1;
double OneDirectionStepsCount = 25;
double StepForGoToExit = 0.2;
double ExitX = -10.0;
double ExitY = 10.0;
double angleStep = 0.1;

double GenerateStep()
{
	return (MaxStep-MinStep) * ((double)rand()/RAND_MAX) + MinStep;
}

bool AreObjectsCloseEnough(double x1, double y1, double x2, double y2)
{
	double distance = sqrt(pow(x1-x2, 2)+pow(y1-y2, 2));
	if(distance < 15)
		return true;
	return false;
}

int main(int argc, char **argv)
{
	init(argc, argv, "lost_robot");
    	sleep(19);

    	NodeHandle n;
    	Publisher publisher = n.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    	gazebo_msgs::ModelState msg;
    	msg.model_name = "lost_robot";
    	msg.pose.position.x = 4;
    	msg.pose.position.y = -3;
    	msg.pose.orientation.z = sin(0);
    	msg.pose.orientation.w = cos(0);
    	srand(time(NULL));
	tf::TransformListener listener;
	Rate rate(50);
	int stepNumber = 0;
	double stepX = GenerateStep();
	double stepY = GenerateStep();
	double oldAngle, angle=0;

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
			publisher.publish(msg);
      			sleep(1);
            		continue;
       		}
		if(stepNumber == OneDirectionStepsCount)
		{
			stepNumber = 0;
			stepX = GenerateStep();
			stepY = GenerateStep();
		}
    		msg.pose.position.x += stepX;
    		msg.pose.position.y += stepY;
		oldAngle=angle;
		angle = atan2(stepY, stepX);
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
    		msg.pose.orientation.z = sin(angle/2);
    		msg.pose.orientation.w = cos(angle/2);
		publisher.publish(msg);
                if (AreObjectsCloseEnough(transform.getOrigin().x(), transform.getOrigin().y(), 
				        msg.pose.position.x, msg.pose.position.y))
                {
			break;
                }
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
			publisher.publish(msg);
      			sleep(1);
            		continue;
       		}
		oldAngle = angle;
         	angle = atan2(transform.getOrigin().y(), transform.getOrigin().x());
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
    		msg.pose.position.x+=StepForGoToExit*cos(angle);
    		msg.pose.position.y+=StepForGoToExit*sin(angle);
    		msg.pose.orientation.z = sin(angle/2);
    		msg.pose.orientation.w = cos(angle/2);
		publisher.publish(msg);
		if(sqrt(pow(ExitX-msg.pose.position.x, 2)+pow(ExitY-msg.pose.position.y, 2))<2)
			break;
        	rate.sleep();
    	}
	while(true)
		publisher.publish(msg);
}
