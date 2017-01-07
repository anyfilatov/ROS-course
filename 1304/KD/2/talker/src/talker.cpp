#include <ros/ros.h>
#include "smessage/smessage.h"
#include <string> 

using namespace std;

string encrypt(int x, int y) {
    string msg = std::to_string(x) + " " + std::to_string(y);
    char keyToEncrypt = 's';
    
    for (int i = 0; i < msg.size(); i++) {
        msg[i] ^= keyToEncrypt;
    }
    
    return msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle node;
    ros::Publisher pub = node.advertise<smessage::smessage>("topSecret", 1000);
    ros::Rate loopRate(10);
    
    while(ros::ok) {
        smessage::smessage msg;
        int x = rand();
        int y = rand();
        
        msg.text = encrypt(x, y);
        
        pub.publish(msg);
        loopRate.sleep();
    }
    
    return 0;
}