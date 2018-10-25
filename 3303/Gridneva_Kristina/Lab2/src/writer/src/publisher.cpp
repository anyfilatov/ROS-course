#include "ros/ros.h"
#include <one_message/Txt.h>
#include <sstream>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>


int main(int argc, char **argv)
{
   setlocale(LC_CTYPE, "");
   std::string line;
   std::ifstream in("/home/kristina/lab2_ws/src/writer/Charter.txt"); 

   ros::init(argc, argv, "publisher");
   ROS_INFO_STREAM("Publisher starts\n");
   ros::NodeHandle n;
   ros::Publisher pub = n.advertise<one_message::Txt>("Speaker", 1000);

   ros::Rate loop_rate(1);
   int count = 0;
   bool flag = false;
   if(in.is_open())
   {
    	while(getline(in, line))
        {
         one_message::Txt msg;
         if(count%5 == 0 && flag == true)
         {
         	msg.text ="Горького 5\n";
         }
         else
         {
         	msg.text = line;
         	flag = true;
         }
         
         ROS_INFO("%s", msg.text.c_str());
         pub.publish(msg);
         ros::spinOnce();
         loop_rate.sleep();
         ++count;
        }
   }
   in.close();
   ROS_INFO_STREAM("Publisher is finished!\n");
   return 0;
}

