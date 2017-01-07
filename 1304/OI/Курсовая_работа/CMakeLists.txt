cmake_minimum_required(VERSION 2.8.3)
project(robot_sim)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	geometry_msgs
	visualization_msgs
	std_msgs
	tf
	gazebo_ros)

catkin_package(
	DEPENDS gazebo_ros
)

include_directories(include ${catkin_INCLUDE_DIRS}
  							${GAZEBO_INCLUDE_DIRS}
  							${SDFormat_INCLUDE_DIRS})

add_executable(	roboFight src/main.cpp 
				src/robot.cpp
				src/steeringbehaviors.cpp
				src/movingentity.cpp
				src/entitymanager.cpp
				src/messagedispatcher.cpp
				src/maths.cpp
				src/battleteam.cpp
				src/robotbase.cpp )

add_definitions(-std=c++11 )

target_link_libraries(roboFight ${catkin_LIBRARIES})
