cmake_minimum_required(VERSION 2.8.3)

project(course_explorer)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf
        std_msgs
        visualization_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        CATKIN_DEPENDS roscpp std_msgs
        #  DEPENDS system_lib
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(explorer
        src/explorer.cpp
        src/RvizVisualizer.cpp
        src/LaserScanProcessor.cpp
        src/RobotController.cpp
        src/PathPlanner.cpp
        src/GeomUtils.cpp src/BreakPointsExtractor.cpp include/BreakPointsExtractor.h)

## Specify libraries to link a library or executable target against
target_link_libraries(explorer ${catkin_LIBRARIES})

