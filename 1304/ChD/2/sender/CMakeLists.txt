cmake_minimum_required(VERSION 2.8.3)
project(sender)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	std_msgs
	msg
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(sender src/sender.cpp)
target_link_libraries(sender ${catkin_LIBRARIES})
add_dependencies(sender ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
