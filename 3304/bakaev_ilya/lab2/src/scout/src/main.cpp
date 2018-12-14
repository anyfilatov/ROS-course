#include <string>
#include <random>
#include <ros/ros.h>
#include <scout_message/message.h>

std::string random_string(unsigned count);
int random_number(int min, int max);

int main(int argc, char** argv) {
    ros::init(argc, argv, "scout");

    ros::NodeHandle nodeHandle;
    ros::Publisher publisher = nodeHandle.advertise<scout_message::message>("/public_messages", 100);

    scout_message::message message;

    for (ros::Rate rate(1); ros::ok(); ros::spinOnce(), rate.sleep()) {
        message.message = random_string(10);
        message.valid = random_number(0, 10);

        publisher.publish(message);
    }

    return 0;
}

std::string random_string(unsigned count) {
    static std::random_device random_device;
    static std::uniform_int_distribution<char> lower('a', 'z');
    static std::uniform_int_distribution<char> upper('A', 'Z');
    static std::uniform_int_distribution<int> ducat(0, 1);

    std::string string;
    for (int i = 0; i < count; i++) {
        switch (ducat(random_device)) {
        case true:
            string.append(1, lower(random_device));
            break;
        case false:
            string.append(1, upper(random_device));
            break;
        default:
            break;
        }
    }

    return string;
}

int random_number(int min, int max) {
    static std::random_device random_device;
    static std::uniform_int_distribution<int> numeric(min, max);

    return numeric(random_device);
}
