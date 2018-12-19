cmake_minimum_required(VERSION 2.8.3)
project(coursework)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  tf
  gazebo_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)

set (CMAKE_CXX_STANDARD 11)
add_executable(base src/base.cpp)
target_link_libraries(base ${catkin_LIBRARIES})
