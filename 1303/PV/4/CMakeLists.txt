cmake_minimum_required(VERSION 2.8.3)
project(lab4)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	sensor_msgs
	geometry_msgs
	nav_msgs
	tf
)

find_package(tf REQUIRED)

catkin_package()

catkin_package(INCLUDE_DIRS include /usr/local/include)
include_directories(include ${catkin_INCLUDE_DIRS} ${tf_INCLUDE_DIRS})

add_executable(lab4 src/main.cpp)
target_link_libraries(lab4 ${catkin_LIBRARIES} ${tf_LIBRARIES})