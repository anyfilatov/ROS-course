#include <ros/ros.h>

#include <termios.h>
#include <unistd.h>
#include <iostream>

#include <client/Direction.h>

int getch()
{
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt); // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);               // disable buffering
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // apply new settings

    int c = getchar(); // read character (non-blocking)

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // restore old settings
    return c;
}

constexpr uint8_t UP = 65;
constexpr uint8_t DOWN = 66;
constexpr uint8_t LEFT = 68;
constexpr uint8_t RIGHT = 67;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "client");
    ros::NodeHandle node;
    ros::Publisher pub = node.advertise<client::Direction>("/roguelike/control", 10);

    while (true)
    {
        int c = getch();
        client::Direction msg;

        switch (c)
        {
        case UP:
            msg.dx = 0;
            msg.dy = -1;
            break;
        case DOWN:
            msg.dx = 0;
            msg.dy = 1;
            break;
        case LEFT:
            msg.dx = -1;
            msg.dy = 0;
            break;
        case RIGHT:
            msg.dx = 1;
            msg.dy = 0;
            break;
        default:
            continue;
        }

        pub.publish(msg);
    }

    return 0;
}
