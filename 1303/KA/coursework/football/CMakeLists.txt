cmake_minimum_required(VERSION 2.8.3)
project(football)

find_package(catkin REQUIRED COMPONENTS
	roscpp
  	rospy
  	tf
  	gazebo_ros
    std_msgs 
    message_generation
    football_msg
)

#add_message_files(
#   FILES
#   ballstate.msg
# )

#generate_messages(
#   DEPENDENCIES
#   std_msgs  
# )


catkin_package(
#    CATKIN_DEPENDS message_runtime
  	DEPENDS gazebo_ros
)

include_directories(
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)


#catkin_package(
#   CATKIN_DEPENDS message_runtime
#)

add_executable(footballer src/footballer.cpp)
add_executable(ball src/ball.cpp)
add_executable(arbiter src/arbiter.cpp)

target_link_libraries(footballer
  ${catkin_LIBRARIES}
)

target_link_libraries(ball
  ${catkin_LIBRARIES}
)

target_link_libraries(arbiter
  ${catkin_LIBRARIES}
)