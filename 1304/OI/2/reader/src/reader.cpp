#include <ros/ros.h>
#include <std_msgs/String.h>
#include <message/percentage.h>

void _read(const message::percentage::ConstPtr &percent) {
    ROS_INFO("Received a message: %f", percent->percent);
    ROS_INFO("projectile coordinates: %f, %f, %f", percent->x, percent->y, percent->z);
}

int main(int argc, char **argv){
   ros::init(argc, argv, "reader");

   ros::NodeHandle n;
   
   ros::Subscriber s = n.subscribe("hit_probability", 1000, _read);

   ros::spin();

   ROS_INFO("Came here");
   return 0;
}
