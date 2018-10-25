cmake_minimum_required(VERSION 2.8.3)
project(subscriber_salary)

find_package(catkin REQUIRED COMPONENTS roscpp user_message)
catkin_package()
include_directories(include  ${catkin_INCLUDE_DIRS})
add_executable(subscriber_salary src/subscriber_salary.cpp)
target_link_libraries(subscriber_salary ${catkin_LIBRARIES})

