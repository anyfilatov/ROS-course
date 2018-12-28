cmake_minimum_required(VERSION 2.8.3)
project(cleaner)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    gazebo_ros
)

find_package(gazebo REQUIRED)

catkin_package(
	DEPENDS gazebo_ros
)

include_directories(
	${Boost_INCLUDE_DIR}
 	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
	${SDFormat_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(cleaner_node src/main.cpp)
target_link_libraries(cleaner_node ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} pthread)
