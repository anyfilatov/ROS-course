#include <fstream>

#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo.hh>

#include "cleaner_controller.h"
#include "cleaner.h"

using std::string;

int16_t lastCommand = 0;

void commandReceived(const Int16::ConstPtr &command) {
    lastCommand = command->data;
}

int main(int argc, char **argv) {
    string modelName = string(argv[1]);
    string pathToModel = string(argv[2]);

    Time::init();

    gazebo::client::setup(argc, argv);
    init(argc, argv, modelName);

    NodeHandle nodeHandle;
    Subscriber subscriber = nodeHandle.subscribe("cleaner_controller_topic", 1000, commandReceived);

    Cleaner *cleaner = new Cleaner(pathToModel, modelName);

    for(Rate rate(10); ros::ok(); rate.sleep()) {
        switch (lastCommand) {
            case Command::CONTINUE:
                cleaner->makeStep();
                break;

            case Command::PAUSE:
                lastCommand = Command::NOTHING;
                break;

            case Command::ABORT:
                delete cleaner;
                gazebo::shutdown();
                return EXIT_SUCCESS;

            default: break;
        }
        gazebo::common::Time::MSleep(100);
        spinOnce();
    }
}
