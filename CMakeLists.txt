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

## Nodes for Simple Chain
add_executable(node2 src/node2.cpp)
target_link_libraries(node2 ${catkin_LIBRARIES})

add_executable(node3 src/node3.cpp)
target_link_libraries(node3 ${catkin_LIBRARIES})

add_executable(node4 src/node4.cpp)
target_link_libraries(node4 ${catkin_LIBRARIES})

add_executable(node5 src/node5.cpp)
target_link_libraries(node5 ${catkin_LIBRARIES})