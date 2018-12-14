cmake_minimum_required(VERSION 2.8.3)
project(scout_message)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)


add_message_files(FILES message.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories()
