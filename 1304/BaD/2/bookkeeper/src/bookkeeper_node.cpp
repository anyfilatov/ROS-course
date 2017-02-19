#include "bookkeeper.h"
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <vector>

std::vector<std::string> split(std::string& str) {
    std::istringstream iss(str);
    return std::vector<std::string> { std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bookkeeper");
    ros::NodeHandle nodeHandle;

    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
       ros::console::notifyLoggerLevelsChanged();
    }

    if(argc == 3) {

        std::cout << "Run bookkeeper" << std::endl;
        Bookkeeper bookkeeper(nodeHandle, std::string(argv[1]), std::string(argv[2]));
        std::string line;
        std::ifstream input("/home/saydos/Desktop/ros/catkin_ws/src/bookkeeper/resources/worker_list.txt");
        if (input.is_open()) {

            while(std::getline(input, line)) {
                std::vector<std::string> v = split(line);
                bookkeeper.addWorkerInfo(v[0], v[1], std::atol(v[2].c_str()));
            }
            input.close();
            bookkeeper.reportSalaries();

        }
        else {
            ROS_ERROR("[Bookkeeper] can't read file");
        }

    }
    else {
        ROS_ERROR("To few parameters (need write last_name and first_name)");
    }

    ros::spin();

    return 0;
}
