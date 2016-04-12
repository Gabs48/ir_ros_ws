cmake_minimum_required(VERSION 2.8.3)
project(ir_fsm_test)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
)

catkin_python_setup()
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS bin/ir_fsm_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
