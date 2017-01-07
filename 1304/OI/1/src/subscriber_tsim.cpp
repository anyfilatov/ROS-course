#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Color.h>

void color_callback(const turtlesim::Color::ConstPtr& col)
{
   ROS_INFO("red: %d, green: %d, blue %d", col->r, col->g, col->b);
}

void pose_callback(const turtlesim::Pose& msgIn){

   ROS_INFO_STREAM(std::setprecision(2) << std::fixed
   << "position=(" << msgIn.x << "," << msgIn.y << ")"
   << "direction=(" << msgIn.theta);
}

int main(int argc, char **argv){
   ros::init(argc, argv, "subTsim");
   
   ros::NodeHandle node;

   ros::Subscriber colorSubscriber = node.subscribe("/turtle1/color_sensor", 1000, color_callback);

   ros::Subscriber poseSubscriber = node.subscribe("/turtle1/pose", 1000, pose_callback);

   ros::spin();

   return 0;
}
