#include "ros/ros.h"
#include "satellite_msg/SatelliteMessage.h"
#include <string>
#include <sstream>
#include <cstdlib> // funk for rand() and srand()
    
int main(int argc, char **argv)
{
  ros::init(argc, argv, "sender");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<satellite_msg::SatelliteMessage>("Name", 5);
  ros::Rate loop_rate(5);
  
  while (ros::ok())
  {
	
  //Generation of rand position coordinates 
  int pos_x = 1 + rand() % 1000;
  int pos_y = 1 + rand() % 1000;
  
  std::ostringstream str;
  str << pos_x << " "<< pos_y ;
  satellite_msg::SatelliteMessage msgCode; 
  msgCode.sat_msg = str.str();
  pub.publish(msgCode);
  ros::spinOnce();
  loop_rate.sleep();
   
  }
   return 0;
}
