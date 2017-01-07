#include <ros/ros.h>

int main(int argv, char **argc){
   ros::init(argv, argc, "hello_world");

   ros::NodeHandle nh;

   ROS_INFO_STREAM("Hello, ROS!");
}
