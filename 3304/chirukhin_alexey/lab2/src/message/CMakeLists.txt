cmake_minimum_required(VERSION 2.8.3)
project(lab2_msg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  my_message.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package()
