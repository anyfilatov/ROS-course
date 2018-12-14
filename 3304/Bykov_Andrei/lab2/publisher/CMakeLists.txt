cmake_minimum_required(VERSION 2.8.3)
project(publisher)

find_package(catkin REQUIRED COMPONENTS roscpp message)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node src/publisher_node.cpp)

target_link_libraries(${PROJECT_NAME}_node  ${catkin_LIBRARIES})

