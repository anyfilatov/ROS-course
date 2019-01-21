cmake_minimum_required(VERSION 2.8.3)
project(lab5)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  geometry_msgs
  visualization_msgs
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

add_executable(lost src/lost.cpp)
target_link_libraries(lost ${catkin_LIBRARIES})

add_executable(rescuer src/rescuer.cpp)
target_link_libraries(rescuer ${catkin_LIBRARIES})
