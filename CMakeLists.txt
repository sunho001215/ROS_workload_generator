cmake_minimum_required(VERSION 3.0.2)
project(ros_workload_generator)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs roslib)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(entry_node src/entry_node.cpp)
target_link_libraries(entry_node ${catkin_LIBRARIES})

add_executable(middle_node src/middle_node.cpp)
target_link_libraries(middle_node ${catkin_LIBRARIES})

add_executable(leaf_node src/leaf_node.cpp)
target_link_libraries(leaf_node ${catkin_LIBRARIES})
