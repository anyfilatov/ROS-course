cmake_minimum_required(VERSION 2.8.3)

project(lab1)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/lab1.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
