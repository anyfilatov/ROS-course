#include "ros/ros.h"
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <client/PressedKey.h>

int getch() {
  static struct termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

int main(int argc, char ** argv) {
	ros::init(argc, argv, "client");
	ros::NodeHandle nodeHandle;
	ros::Publisher pub1isher = nodeHandle.advertise<client::PressedKey>("/lab01/listener", 10);

	while (true) {
		char key = getch();
		client::PressedKey msg;
		msg.key = key;
		pub1isher.publish(msg);
	}

	return 0;
}
