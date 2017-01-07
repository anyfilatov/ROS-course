#include <ros/ros.h>
#include <message/percentage.h>
#include <std_msgs/String.h>

void initMessage(message::percentage &);

int main(int argv, char** argc){
   ros::init(argv, argc, "writer");

   ros::NodeHandle wr;
  
   ros::Publisher writer = wr.advertise<message::percentage>("hit_probability", 1000);

   srand(time(NULL));
   ros::Rate loop_rate(1);
   message::percentage msg;
   initMessage(msg);

   while(ros::ok())
   {
	msg.x += 0.1;
   	msg.y += 0.05;
   	msg.z += 0.5; 

	ROS_INFO("projectile: %f, %f, %f", msg.x, msg.y, msg.z);
	writer.publish(msg);
	ros::spinOnce();
	loop_rate.sleep();
   }

   return 0;
}

void initMessage(message::percentage &msg)
{
   msg.percent = 25.0;
   msg.x = 0.2 + 0.1;
   msg.y = 0.8 + 0.05;
   msg.z = 0.5; 
}

