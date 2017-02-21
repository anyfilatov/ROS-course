#include "cleaner_controller.h"

int main(int argc, char **argv) {
    init(argc, argv, "cleaner_controller");
    NodeHandle nodeHandle;
    Publisher publisher = nodeHandle.advertise<Int16>("cleaner_controller_topic", 1000);
    Rate loop(1);
    Rate rate(10);

    system("clear");
    for (Int16 command; ros::ok() && command.data != Command::ABORT;) {
        cout << "Press 'p' to pause"     << endl;
        cout << "Press 'c' to continue"  << endl;
        cout << "Press 'a' to abort"     << endl;
        
        command.data = readCommand();
        system("clear");

        switch (command.data) {
            case Command::PAUSE:
                cout << "Send signal pause...";
                break;

            case Command::CONTINUE:
                cout << "Send signal continue...";
                break;

            case Command::ABORT:
                cout << "Send signal abort...";
                break;

            default:
                cout << "Unrecognized command! Try again...";
                command.data = Command::NOTHING;
                continue;
        }
        cout << endl << "--------------------" << endl;
        publisher.publish(command);

        spinOnce();
        rate.sleep();
    }
    return EXIT_SUCCESS;
}

short readCommand() {
	char val;
        cin.clear();
        cin >> val;
	return (short)val;
}
