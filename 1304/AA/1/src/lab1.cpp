#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lab1");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    geometry_msgs::Twist msg;

	  msg.linear.x = 2;
    msg.angular.z = 2;

    chatter_pub.publish(msg);

    loop_rate.sleep();
  }


  return 0;
}
