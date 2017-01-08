cmake_minimum_required(VERSION 2.8.3)
project(lab3)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	tf
	geometry_msgs
	visualization_msgs
	std_msgs
)

catkin_package()

catkin_package(INCLUDE_DIRS include)
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(main1 src/main1.cpp src/Bot.cpp src/smart_bot.cpp)
target_link_libraries(main1 ${catkin_LIBRARIES})

add_executable(main2 src/main2.cpp src/Bot.cpp src/stupid_bot.cpp)
target_link_libraries(main2 ${catkin_LIBRARIES})
