cmake_minimum_required(VERSION 2.8.3)
project(lost_r)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    std_msgs
    visualization_msgs
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )


generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(lost_r src/lost.cpp)

target_link_libraries(
   lost_r
   ${catkin_LIBRARIES}
)
