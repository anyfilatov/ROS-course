cmake_minimum_required(VERSION 2.8.3)
project(lab3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

catkin_package(
   INCLUDE_DIRS include/
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(saver_robot src/saver_main.cpp src/robot.cpp src/saver_robot.cpp)
add_executable(lost_robot src/lost_main.cpp src/robot.cpp src/lost_robot.cpp)

target_link_libraries(saver_robot
  ${catkin_LIBRARIES}
)

target_link_libraries(lost_robot
  ${catkin_LIBRARIES}
)
