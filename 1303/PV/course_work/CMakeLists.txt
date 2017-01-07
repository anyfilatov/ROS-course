cmake_minimum_required(VERSION 2.8.3)
project(course_work)

# enable c++ 11
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	geometry_msgs 
    visualization_msgs
	std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(commander src/commander.cpp)
target_link_libraries(commander ${catkin_LIBRARIES})

add_executable(flagman src/flagman.cpp)
target_link_libraries(flagman ${catkin_LIBRARIES})

add_executable(world src/world.cpp)
target_link_libraries(world ${catkin_LIBRARIES})