cmake_minimum_required(VERSION 2.8.3)
project(lab3)

    find_package(catkin REQUIRED COMPONENTS 
    roscpp
    visualization_msgs
    tf
    std_msgs
    )


##   * add "message_runtime" and every package in MSG_DEP_SET to
     catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(lobot src/publisher.cpp)
target_link_libraries(lobot ${catkin_LIBRARIES})
add_dependencies(lobot lab3_generate_messages_cpp)
add_executable(rebot src/subscriber.cpp)
target_link_libraries(rebot ${catkin_LIBRARIES})
add_dependencies(rebot lab3_generate_messages_cpp)





