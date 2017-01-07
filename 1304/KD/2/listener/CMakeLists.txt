cmake_minimum_required(VERSION 2.8.3)
project(listener)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    smessage
)

catkin_package()


include_directories(${catkin_INCLUDE_DIRS})

add_executable(listener src/listener.cpp)

add_dependencies(listener ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(listener ${catkin_LIBRARIES})

