#include <ros/ros.h>
#include <labirint/Velocity.h>
#include <labirint/Pose.h>
#include <ncurses.h>
#include <iostream>
#include "utility/cleanup.h"

namespace labirint
{

class Screen
{
public:
  int init(ros::NodeHandle* node)
  {
    ros_node_ = node;
    pose_publisher_ = ros_node_->advertise<Pose>("labirint/pose", 10);
    velocity_listener_ = node->subscribe<Velocity>(
        "labirint/velocity", 10, &Screen::acceptVelocity, this);

    for (int x = 0; x < GRID_HEIGHT; x++) {
      for (int y = 0; y < GRID_WIDTH; y++) {
        if (grid_[x][y] == CHARACTER) {
          pose_.x = x;
          pose_.y = y;
        }
      }
    }
    return 0;
  }

  int show()
  {
    initscr();
    auto _ = cleanup([] {
      endwin();
    });

    ros::Rate rate(30);

    while (ros::ok())
    {
      move(0, 0);
      for (const auto& line: grid_) {
        printw(line);
        printw("\n");
      }

      refresh();
      ros::spinOnce();
      rate.sleep();
    }

    return 0;
  }

private:
  static constexpr char EMPTY = ' ';
  static constexpr char CHARACTER = '@';
  static constexpr char EXIT = 'o';
  static constexpr char GRID_HEIGHT = 8;
  static constexpr char GRID_WIDTH = 11;

  char grid_[GRID_HEIGHT][GRID_WIDTH] = {
      "##########",
      "#@#    #o#",
      "# # #### #",
      "#   #    #",
      "# ###  ###",
      "#   #    #",
      "#        #",
      "##########"
  };

  Pose pose_;
  ros::NodeHandle* ros_node_;
  ros::Publisher pose_publisher_;
  ros::Subscriber velocity_listener_;

  void acceptVelocity(const Velocity::ConstPtr& velocity)
  {
    ROS_DEBUG_STREAM("accept velocity: " << velocity);
    Pose new_pose = pose_;
    new_pose.x += velocity->x;
    new_pose.y += velocity->y;
    switch (grid_[new_pose.x][new_pose.y])
    {
      case EXIT:
        // TODO: win
        // fall through
      case EMPTY:
        grid_[pose_.x][pose_.y] = EMPTY;
        grid_[new_pose.x][new_pose.y] = CHARACTER;
        pose_ = new_pose;
        pose_publisher_.publish(pose_);
        break;
      default:
        break;
    }
  }
};

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "screen");
  ros::NodeHandle node;
  labirint::Screen screen;
  int err = screen.init(&node);
  if (err != 0) { return err; }
  return screen.show();
}
