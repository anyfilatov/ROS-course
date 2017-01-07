cmake_minimum_required(VERSION 2.8.3)
project(rescuer_r)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    std_msgs
    visualization_msgs
)

generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(rescuer_r src/rescuer.cpp)

target_link_libraries(
   rescuer_r
   ${catkin_LIBRARIES}
)
