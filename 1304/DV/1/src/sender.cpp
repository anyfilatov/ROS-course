#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sender");
  ROS_INFO("Node starting...");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  geometry_msgs::Twist base_cmd;
  base_cmd.linear.y = 0;
  base_cmd.linear.z = 0;
  base_cmd.angular.x = 0;
  base_cmd.angular.y = 0;
  
  while(nh.ok())
  {
    double time = ros::Time::now().toSec();
    base_cmd.linear.x = 2.34;
    base_cmd.angular.z = (exp(sin(time/1.3))) * 3.14;
    pub.publish(base_cmd);
    ros::Duration(0.5).sleep();
  }
}
