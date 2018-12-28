cmake_minimum_required(VERSION 2.8.3)
project(lab5)

set(CMAKE_CXX_FLAGS "-std=c++0x")
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    std_msgs
    gazebo_ros
    message_generation
)


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   status.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

catkin_package(
   #INCLUDE_DIRS include
   #LIBRARIES lab5
   CATKIN_DEPENDS roscpp tf std_msgs gazebo_ros message_runtime
   DEPENDS gazebo_ros
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)


add_executable(lost src/lost.cpp src/robot_info.cpp)
target_link_libraries(lost ${catkin_LIBRARIES})
add_dependencies(lost lab5_generate_messages_cpp)

add_executable(find src/find.cpp src/robot_info.cpp)
target_link_libraries(find ${catkin_LIBRARIES})
add_dependencies(find lab5_generate_messages_cpp)
#
