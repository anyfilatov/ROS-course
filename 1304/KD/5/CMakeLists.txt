cmake_minimum_required(VERSION 2.8.3)
project(gazebo_lab)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  tf
)

catkin_package(
 INCLUDE_DIRS include
 DEPENDS gazebo_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)

add_executable(saver_bot src/saver_main.cpp src/bot.cpp src/savebot.cpp)
add_executable(lost_bot src/lost_main.cpp src/bot.cpp src/lostbot.cpp)

target_link_libraries(saver_bot
  ${catkin_LIBRARIES}
)
target_link_libraries(lost_bot
  ${catkin_LIBRARIES}
)
