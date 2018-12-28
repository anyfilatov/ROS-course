cmake_minimum_required(VERSION 2.8.3)
project(course_work)

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

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
add_executable(${PROJECT_NAME}_key src/${PROJECT_NAME}_key.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_key ${catkin_LIBRARIES})
