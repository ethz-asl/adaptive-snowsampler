cmake_minimum_required(VERSION 3.8)
project(ssp_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(geometry_msgs REQUIRED)
find_package(snowsampler_msgs REQUIRED)

if(CMAKE_SYSTEM_NAME STREQUAL Linux)
  # Workaround for the supplied find script not being on CMake path in Ubuntu.
  # https://bugs.launchpad.net/ubuntu/+source/geographiclib/+bug/1805173
  if(IS_DIRECTORY /usr/share/cmake/geographiclib)
    list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib)
  endif()
endif()
find_package(GeographicLib REQUIRED)

include_directories(include)

add_executable(ssp_bridge src/ssp_bridge_node.cpp src/ssp_bridge.cpp)
ament_target_dependencies(ssp_bridge rclcpp std_msgs snowsampler_msgs geometry_msgs px4_msgs)
target_include_directories(ssp_bridge PUBLIC "${Eigen3_INCLUDE_DIRS}")
target_link_libraries(ssp_bridge ${GeographicLib_LIBRARIES})
target_compile_features(ssp_bridge PUBLIC c_std_99 cxx_std_17) # Require C99 and
                                                               # C++17

install(TARGETS ssp_bridge DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY params DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
