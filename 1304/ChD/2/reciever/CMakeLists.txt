cmake_minimum_required(VERSION 2.8.3)
project(reciever)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  msg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(reciever src/reciever.cpp)
add_dependencies(reciever ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(reciever ${catkin_LIBRARIES})
