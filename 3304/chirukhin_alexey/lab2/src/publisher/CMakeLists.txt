cmake_minimum_required(VERSION 2.8.3)
project(lab2_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  lab2_msg
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(lab2_publisher src/publisher.cpp)

target_link_libraries(lab2_publisher ${catkin_LIBRARIES})
