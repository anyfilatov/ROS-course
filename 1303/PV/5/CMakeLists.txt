cmake_minimum_required(VERSION 2.8.3)
project(lab5)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  tf
  std_msgs
  gazebo_ros
)

catkin_package()

include_directories(
	  ${catkin_INCLUDE_DIRS}
  	  ${GAZEBO_INCLUDE_DIRS}
  	  ${SDFormat_INCLUDE_DIRS})

add_executable(c3po src/c3po.cpp)
target_link_libraries(c3po ${catkin_LIBRARIES})

add_executable(r2d2 src/r2d2.cpp)
target_link_libraries(r2d2 ${catkin_LIBRARIES})

