#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "run_turtle_run");
    ROS_INFO_STREAM("The run_turtle_run node is ready!");
  
    ros::NodeHandle nodeHandle;

    ros::Publisher publ = nodeHandle.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  
    geometry_msgs::Twist cmd;
    cmd.linear.x = 1.0;
    cmd.angular.z = -2.8;
  
    while(ros::ok()) {
        publ.publish(cmd);

        ros::spinOnce();
    }
    
    return 0;
}
