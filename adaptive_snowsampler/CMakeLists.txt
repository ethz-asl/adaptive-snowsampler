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
  tf
  grid_map_geo
  grid_map_ros
  eigen_catkin
  interactive_markers
  planner_msgs
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


include_directories(
  include
)

add_executable(adaptive_snowsampler
  src/main.cpp
  src/adaptive_snowsampler.cpp)
target_link_libraries(adaptive_snowsampler ${catkin_LIBRARIES} ${planner_msgs_TARGETS} ${GeographicLib_LIBRARIES})
add_dependencies(adaptive_snowsampler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
