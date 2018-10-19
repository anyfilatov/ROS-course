#include "ros/ros.h"
#include "satellite_msg/SatelliteMessage.h"
#include <string>
#include <sstream>
#include <iostream>
#include <vector>

 
void ReceiveMsg(const satellite_msg::SatelliteMessage & msg)
{
	int coord_x;
	int coord_y;
	//start parse our message
	std::string str = msg.sat_msg;
	int pos_first_coord = str.find(" ");  
	int pos_second_coord = str.rfind(" "); 
	//first coordinate 
	std::string str1 = str.substr (0,pos_first_coord );
	//second coordinate 
	std::string str2 = str.substr (pos_second_coord,pos_second_coord +  str.size()-1);
	coord_x = atoi(str1.c_str());
	coord_y = atoi(str2.c_str());
	if (coord_x>=0 && coord_y>=0){

		std::cout <<" Coordinate x is  "<< coord_x <<" Coordinate y is  " <<coord_y <<std::endl;

	}
	else std::cout <<" Error: coordinates are incorrect !!! " <<std::endl;


}
 int main(int argc, char **argv)
{
  ros::init(argc, argv, "receiver");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("Name", 5, ReceiveMsg);
  ros::spin();
  return 0;
}
