cmake_minimum_required(VERSION 2.8.3)
project(scout)

find_package(catkin REQUIRED COMPONENTS roscpp scout_message)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
