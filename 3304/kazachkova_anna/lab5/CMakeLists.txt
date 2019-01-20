cmake_minimum_required(VERSION 2.8.3)
project(lab5)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  geometry_msgs
  tf
)

catkin_package(
  DEPENDS gazebo_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(lost src/lost.cpp)
add_executable(finder src/finder.cpp)

target_link_libraries(lost ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(finder ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
