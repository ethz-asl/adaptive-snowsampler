cmake_minimum_required(VERSION 3.0.2)
project(ssp_bridge)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  snowsampler_msgs
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS rospy std_msgs snowsampler_msgs
)

# Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Declare Python scripts to be installed
catkin_install_python(PROGRAMS
  src/ssp_bridge/ssp_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install all other scripts
install(PROGRAMS
  scripts/*
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)