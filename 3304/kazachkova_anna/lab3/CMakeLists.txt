cmake_minimum_required(VERSION 2.8.3)
project(lab3)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  geometry_msgs
  visualization_msgs
  std_msgs
  tf
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(lost_robot src/lost.cpp)
target_link_libraries(lost_robot ${catkin_LIBRARIES})

add_executable(finder_robot src/finder.cpp)
target_link_libraries(finder_robot ${catkin_LIBRARIES})
