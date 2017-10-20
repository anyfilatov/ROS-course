#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <client/PressedKey.h>

const int UP = 65;
const int DOWN = 66;
const int LEFT = 68;
const int RIGHT = 67;

const int MAXN = 100;
const int MAXM = 100;

int n, m;
char map[MAXN][MAXM];
int myX, myY;
int finalX, finalY;

bool makeMove(const client::PressedKey& msg) {
  int newX = myX, newY = myY;
  switch (msg.key) {
    case UP: newX--; break;
    case DOWN: newX++; break;
    case LEFT: newY--; break;
    case RIGHT: newY++; break;
    default: newX = -1;
  }

  if (newX >= 0 && newX < n && newY >= 0 && newY < m) {
    if (map[newX][newY] == '.') {
      myX = newX;
      myY = newY;
      return true;
    } else {
      if (map[newX][newY] == 'F') {
        std::cout << "You win!!!\n";
        exit(0);
      }
    }
  }
  return false;
}

void show() {
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      if (i == myX && j == myY) {
        std::cout << "@";
      } else {
        std::cout << map[i][j];
      }
    }
    std::cout << "\n";
  }
  std::cout << std::endl;
}

void cb(const client::PressedKey& msg) {
  if (makeMove(msg)) {
    show();
  }
}

void readMap(const char* fileName) {
  std::ifstream inputFile(fileName);
  if (inputFile.is_open()) {
    inputFile >> n >> m;
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < m; j++) {
        inputFile >> map[i][j];
        if (map[i][j] == '@') {
          map[i][j] = '.';
          myX = i;
          myY = j;
        }
        if (map[i][j] == 'F') {
          finalX = i;
          finalY = j;
        }
      }
    }
  } else {
    std::cerr << "Unable to open file\n";
    exit(1);
  }
}

int main(int argc, char ** argv) {
  readMap("resources/map_00.txt");
  show();

  ros::init(argc, argv, "server");
  ros::NodeHandle nodeHandle;
  ros::Subscriber sub = nodeHandle.subscribe("/lab01/listener", 10, cb);
  ros::spin();
  return 0;
}
