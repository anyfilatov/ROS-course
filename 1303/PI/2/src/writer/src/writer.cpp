#include "ros/ros.h"
#include "std_msgs/String.h"
#include "message/location.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "writer");
    ros::NodeHandle nodeHandle;

    ros::Publisher writer = nodeHandle.advertise<message::location>("coordinates", 1000);

    message::location loc;
    loc.x = 5;
    loc.y = 6;
    loc.z = 14;
    while (ros::ok())
    {
        ROS_INFO("Rocket launched to coordinates: x= %f y= %f z= %f\n",loc.x,loc.y,loc.z);
        
        writer.publish(loc);
        
        ros::spinOnce();
        
        sleep(3);
    }

    return 0;
}