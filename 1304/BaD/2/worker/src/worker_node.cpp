#include "worker.h"
#include <ros/console.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "worker");
    ros::NodeHandle nodeHandle;

    if (argc == 3) {
        Worker worker(nodeHandle, std::string(argv[1]), std::string(argv[2]));
        worker.listen();
    }
    else {
        ROS_ERROR("To few parameters (need write last_name and first_name)");
    }

    ros::spin();

    return 0;
}
