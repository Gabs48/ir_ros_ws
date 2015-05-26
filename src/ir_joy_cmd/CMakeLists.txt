# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(ir_joy_cmd)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS joy roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
# add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
# generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(joy_list src/listener.cpp)
target_link_libraries(joy_list ${catkin_LIBRARIES})
# add_dependencies(listener beginner_tutorials_generate_messages_cpp)

# %EndTag(FULLTEXT)%