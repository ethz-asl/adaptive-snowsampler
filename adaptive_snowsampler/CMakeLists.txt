cmake_minimum_required(VERSION 3.8)
project(adaptive_snowsampler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(snowsampler_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(planner_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_geo REQUIRED)

if(CMAKE_SYSTEM_NAME STREQUAL Linux)
  # Workaround for the supplied find script not being on CMake path in Ubuntu.
  # https://bugs.launchpad.net/ubuntu/+source/geographiclib/+bug/1805173
  if(IS_DIRECTORY /usr/share/cmake/geographiclib)
    list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib)
  endif()
endif()
find_package(GeographicLib REQUIRED)


include_directories(
  include
)

add_executable(adaptive_snowsampler
  src/main.cpp
  src/adaptive_snowsampler.cpp)
ament_target_dependencies(adaptive_snowsampler snowsampler_msgs rclcpp std_msgs geometry_msgs px4_msgs tf2 tf2_ros grid_map_core grid_map_ros grid_map_geo visualization_msgs)
target_include_directories(adaptive_snowsampler PUBLIC "${Eigen3_INCLUDE_DIRS}")
target_link_libraries(adaptive_snowsampler ${planner_msgs_TARGETS} ${GeographicLib_LIBRARIES})

install(
  TARGETS
  adaptive_snowsampler
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  resources
  DESTINATION share/${PROJECT_NAME}/
)

ament_python_install_package(snowsampler_lac)

install(PROGRAMS
  scripts/snowsampler_lac.py
  scripts/lac.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
