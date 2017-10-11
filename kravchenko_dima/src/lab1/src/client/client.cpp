#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <cstdio>

#include "lab1/ros_utils.h"

RosClient *ros_client;

static void init_ros(int argc, char *argv[])
{
    ros_client = new RosClient();
    ros_client->init_ros(argc, argv);
}

static char getch()
{
	static struct termios oldt, newt;
	tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // apply new settings

    int c = std::getchar();  // read character (non-blocking)

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return c;
}

static void send_command(char ch)
{
    ros_client->send_move(ch);
}

int main(int argc, char *argv[])
{
    init_ros(argc, argv);
    while (true) {
        char ch = getch();
        if (ch == EOF)
            break;
        send_command(ch);
    }
}
