cmake_minimum_required(VERSION 2.8.3)
project(Robots)


find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg gazebo_ros)

catkin_package(
  DEPENDS gazebo_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)

add_executable(FindRobot FindRobot.cpp)
target_link_libraries(FindRobot ${catkin_LIBRARIES})

add_executable(LostRobot LostRobot.cpp)
target_link_libraries(LostRobot ${catkin_LIBRARIES})




