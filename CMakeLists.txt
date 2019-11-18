cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories( include ${catkin_INCLUDE_DIRS}
)

add_executable(turtleba src/main.cpp src/TurtleBa.cpp)
target_link_libraries(turtleba ${catkin_LIBRARIES})
# add_dependencies(turtleba gazebo_tests_generate_messages_cpp)


