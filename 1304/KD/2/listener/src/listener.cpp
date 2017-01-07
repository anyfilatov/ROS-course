#include <ros/ros.h>
#include "smessage/smessage.h"
#include <string> 

using namespace std;

string unencrypt(string msg) {
    char keyToEncrypt = 's';
    
    for (int i = 0; i < msg.size(); i++) {
        msg[i] ^= keyToEncrypt;
    }
    
    return msg;
}

void callback(const smessage::smessage::ConstPtr& msg)
{
    ROS_INFO("Message: %s", unencrypt(msg->text).c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("topSecret", 1000, callback);
    ros::spin();

    return 0;
}