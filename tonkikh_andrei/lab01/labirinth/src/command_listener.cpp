#include <termios.h>
#include <unistd.h>
#include <ros/ros.h>
#include <labirint/Velocity.h>
#include <iostream>

namespace labirint
{

constexpr int UP_CODE = 0x41;
constexpr int DOWN_CODE = 0x42;
constexpr int RIGHT_CODE = 0x43;
constexpr int LEFT_CODE = 0x44;

int getch()
{
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);            // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);   // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);   // restore old settings
  return c;
}

int listenCommands(ros::NodeHandle *node)
{
  auto velocity_publisher = node->advertise<Velocity>("labirint/velocity", 10);
  std::cout << "Press 'q' to exit" << std::endl;

  bool stop = false;
  while (ros::ok() && !feof(stdin) && !stop)
  {
    Velocity velocity;
    bool is_arrow = true;
    int ch = getch();

    switch (ch)
    {
      case UP_CODE:
        velocity.x = -1;
        break;
      case DOWN_CODE:
        velocity.x = 1;
        break;
      case RIGHT_CODE:
        velocity.y = 1;
        break;
      case LEFT_CODE:
        velocity.y = -1;
        break;
      case 'q':
        stop = true;
        // fall through
      default:
        is_arrow = false;
    }

    if (is_arrow) {
      velocity_publisher.publish(velocity);
      ros::spinOnce();
    }
  }
}

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "command_listener");
  ros::NodeHandle node;
  labirint::listenCommands(&node);
}

