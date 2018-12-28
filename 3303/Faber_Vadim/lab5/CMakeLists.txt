cmake_minimum_required(VERSION 2.8.3)
project(lab5)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf
        geometry_msgs
	gazebo_ros
        )

catkin_package(
	DEPENDS gazebo_ros
	)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${gazebo_INCLUDE_DIRS}
        ${SDFormat_INCLUDE_DIRS}
        )

add_executable(lost_robot_5 src/lost_robot_5.cpp)
add_executable(finder_robot_5 src/finder_robot_5.cpp)

target_link_libraries(lost_robot_5 ${catkin_LIBRARIES})
target_link_libraries(finder_robot_5 ${catkin_LIBRARIES})
