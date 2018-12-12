#include "ros/ros.h"
#include <the_message/Txt.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>



int main(int argc, char **argv)
{
   std::string line;
   std::ifstream text_file("/home/vadim/lab/workspace/src/writer/Charter.txt"); 

   ros::init(argc, argv, "publisher");
   ROS_INFO_STREAM("Publisher is publishing\n");
   ros::NodeHandle n;
   ros::Publisher pub = n.advertise<the_message::Txt>("Speaker", 1000);

   ros::Rate loop_rate(1);

   if(text_file.is_open())
   {
    	while(getline(text_file, line))
        {
            the_message::Txt msg;
            std::string number = std::to_string(rand()%100);
            msg.text = line + " " + number;
            ROS_INFO("%s", msg.text.c_str());
            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }
        text_file.close();
   }

   ROS_INFO_STREAM("Publisher is done.\n");

   return 0;
}


