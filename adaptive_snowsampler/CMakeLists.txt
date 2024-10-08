cmake_minimum_required(VERSION 3.8)
project(adaptive_snowsampler)
add_definitions(-std=c++17)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")


# find dependencies
find_package(Eigen3)
find_package(GeographicLib REQUIRED)
include(CheckGeographicLibDatasets)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  grid_map_geo
  grid_map_ros
  grid_map_geo_msgs
  eigen_catkin
  interactive_markers
  planner_msgs
  snowsampler_msgs
)

execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -m pip install --no-warn-script-location -r "${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt"
    RESULT_VARIABLE _pip_install_result
    OUTPUT_VARIABLE _pip_install_output
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES adaptive_snowsampler
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS 
  scripts/snowsampler_lac.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

include_directories(
  include
)

add_executable(adaptive_snowsampler
  src/main.cpp
  src/adaptive_snowsampler.cpp)
target_link_libraries(adaptive_snowsampler ${catkin_LIBRARIES} ${planner_msgs_TARGETS} ${GeographicLib_LIBRARIES})
add_dependencies(adaptive_snowsampler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
