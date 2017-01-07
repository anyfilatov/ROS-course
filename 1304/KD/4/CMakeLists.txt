cmake_minimum_required(VERSION 2.8.3)
project(roaming_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop src/teleop.cpp)

target_link_libraries(teleop
  ${catkin_LIBRARIES}
)
