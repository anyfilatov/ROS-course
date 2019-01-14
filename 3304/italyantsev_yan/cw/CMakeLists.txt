cmake_minimum_required(VERSION 2.8.3)
project(cw)

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
   picked.msg
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

catkin_package(
   #INCLUDE_DIRS include
   #LIBRARIES cw
   CATKIN_DEPENDS roscpp tf std_msgs gazebo_ros message_runtime
   DEPENDS gazebo_ros
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)


add_executable(str src/supplies.cpp src/robot_info.cpp)
target_link_libraries(str ${catkin_LIBRARIES})
add_dependencies(str cw_generate_messages_cpp)

add_executable(survivor src/survivor.cpp src/robot_info.cpp)
target_link_libraries(survivor ${catkin_LIBRARIES})
add_dependencies(survivor cw_generate_messages_cpp)

##add_executable(bonus src/bonus.cpp src/bonus_info.cpp)
##target_link_libraries(bonus ${catkin_LIBRARIES})
##add_dependencies(bonus cw_generate_messages_cpp)

add_executable(zombies src/zombies.cpp src/robot_info.cpp)
target_link_libraries(zombies ${catkin_LIBRARIES})
add_dependencies(zombies cw_generate_messages_cpp)
#
