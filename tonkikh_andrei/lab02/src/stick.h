
#ifndef _XOSMIG_LAB02_STICK_H_
#define _XOSMIG_LAB02_STICK_H_

#define STICK_DEFAULT_LENGTH 0.4

namespace lab02 {

class Stick {
public:
  Stick(ros::NodeHandle node, double length, std::string name, std::string frame);

  int run();
private:
  ros::NodeHandle node;
  const double length;
  const std::string name;
  const std::string frame;
};

}

#endif  // _XOSMIG_LAB02_STICK_H_
