cmake_minimum_required(VERSION 2.8.3)
project(lab_01)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
add_executable(lab_01 src/main.cpp)
target_link_libraries(lab_01 ${catkin_LIBRARIES})
