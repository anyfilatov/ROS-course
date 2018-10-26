cmake_minimum_required(VERSION 2.8.3)
project(publisher_salary)

find_package(catkin REQUIRED COMPONENTS roscpp user_message)
catkin_package()
include_directories(include  ${catkin_INCLUDE_DIRS})
add_executable(publisher_salary src/publisher_salary.cpp)
target_link_libraries(publisher_salary ${catkin_LIBRARIES})

