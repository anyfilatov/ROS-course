cmake_minimum_required(VERSION 2.8.3)
project(talker)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    smessage
)

catkin_package()


include_directories(${catkin_INCLUDE_DIRS})

add_definitions(-std=c++11)

add_executable(talker src/talker.cpp)

add_dependencies(talker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(talker ${catkin_LIBRARIES})