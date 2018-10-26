cmake_minimum_required(VERSION 2.8.3)
project(lab2_reader)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  lab2_msg
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(lab2_reader src/reader.cpp)

target_link_libraries(lab2_reader ${catkin_LIBRARIES})
