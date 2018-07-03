cmake_minimum_required(VERSION 2.8.3)
project(youbot_pick_n_place)

find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS
    roscpp
    moveit_ros_planning_interface

    )

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${Boost_DIRECTORIES}
    )

## Call this before add_executable, else the executable might not be 
## visible to rosrun
catkin_package()

## Declare a C++ executable
add_executable(youbot_pick_n_place_v1 src/youbot_pick_n_place_v1.cpp)


## Specify libraries to link a library or executable target against

target_link_libraries(youbot_pick_n_place_v1
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    )
