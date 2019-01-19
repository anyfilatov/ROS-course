#include <csignal>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "robot_utilis.h"

void foundCallBack(const std_msgs::String& msg);
bool found = false;
RobotCar* lost_robot_gazebo;
void deleteModelHandle(int exitnum)
{
    ROS_INFO("Finshed!");
    if (lost_robot_gazebo!=NULL)
    {
        lost_robot_gazebo->deleteModel();
        delete lost_robot_gazebo;
    }
    exit(exitnum);
    
}

int main(int argc, char *argv[])
{
    signal(SIGINT,deleteModelHandle);
    ros::init(argc, argv, "lost_robot_gazebo");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("found_topic",100,foundCallBack);
    tf::Rate rate(40);
    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;
    tf::StampedTransform rescure_trandform;
    tf::Transform trandform;

    srand(time(NULL));
    
    //random position inside 50*50 grid
    float x = (-25)+rand()%51;
    float y = (-25)+rand()%51;

    float rescure_x = 0.0;
    float rescure_y = 0.0;

    int wait_cycles = 40;

    lost_robot_gazebo = new RobotCar(node,40,"lost_robot_gazebo","/root/.gazebo/models/pioneer3at/model-1_4.sdf", x, y);

    while (ros::ok())
    {
        trandform.setOrigin(tf::vector3(x,y,0.0));
        trandform.setRotation(tf::Quaternion(0,0,0,1));
        broadcaster.sendTransform(tf::StampedTransform(trandform,ros::Time::now(),"world","lost_robot_gazebo"));
        
        if (!found)
        {
            //move in random direction
            int i = rand()%2;
            int j = rand()%2;
            if (i==1&&j==1)
            {
                x+=1;
            }else if (i==1&&j==0)
            {
                x-=1;
            }else if (i==0&&j==1)
            {
                y+=1;
            }else if (i==-&&j==0)
            {
                y-=1;
            }
            
        }else
        {
            //wait some time before following rescure
            while (wait_cycles>0)
            {
                rate.sleep();
                wait_cycles--;
            }

            try
            {
                listener.waitForTransform("lost_robot_gazebo","rescure_robot_gazebo",ros::Time(0),ros::Duration(0.5));
                listener.lookupTransform("lost_robot_gazebo","rescure_robot_gazebo",ros::Time(0),rescure_trandform);
            }
            catch (tf::TransformException &ex)
            {
                rate.sleep();
                continue;
            }

            rescure_x = rescure_trandform.getOrigin().x();
            rescure_y = rescure_trandform.getOrigin().y();
            if (fabs(x)<=1&&fabs(y)<=1)
            {
                ROS_INFO("Rescured!");
            }else
            {
                //follow rescure
                if (fabs(rescure_x)>=fabs(rescure_y))
                {
                    if (rescure_x>0)
                    {
                        x+=1;
                    }else
                    {
                        x-=1;
                    }
                    
                    
                }else
                {
                    if (rescure_y>0)
                    {
                        y+=1;
                    }else
                    {
                        y-=1;
                    }
                   
                }
               
            }
         
        }

        lost_robot_gazebo->move(x,y);
        rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}

void foundCallback(const std_msgs::String& msg)
{
    ROS_INFO_STREAM("Rescuer says: " << msg.data);
    found = true;
}

