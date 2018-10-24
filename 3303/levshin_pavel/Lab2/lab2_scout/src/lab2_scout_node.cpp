#include "ros/ros.h"
#include "lab2_msg/Msg.h"
#include <ctime>

std::string generate_string(){
    std::string str = "";
    for(int i = 0; i < rand()%9 + 8; i++){
        str += ( rand() % 26 + 65);
    }
    return str;
}

int main(int argc, char **argv){

    srand(time(0));

    ros::init(argc, argv, "Scout");
    ROS_INFO("I give information!\n");

    uint32_t count_msgs = 10;

    ros::NodeHandle nh;
    ros::Publisher pub_scout = nh.advertise<lab2_msg::Msg>("Talk", count_msgs);
    sleep(1);

    ros::Rate loop(4);
    for (int i = 0; i < count_msgs; i++){
        lab2_msg::Msg msg;
        msg.text = generate_string() += (rand()%2 + 49);
        pub_scout.publish(msg);
        ROS_INFO("Msg: \"%s\"", msg.text.c_str());
        ros::spinOnce();
        loop.sleep();
    }
	return 0;
}	